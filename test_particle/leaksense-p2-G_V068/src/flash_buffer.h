/*
 * flash_buffer.h  -  Raw-report ring buffer for cloud outages (V057).
 *
 * WHY THIS EXISTS
 * ---------------
 * The trust boundary is the Photon: the PIC only discards a batch once we have
 * ACKed it, so once we ACK, the data is ours to keep. If the cloud is then
 * unreachable we must not lose it - hence this ring of raw report blocks on the
 * device's own file system.
 *
 * A block holds one report exactly as it came off the wire: the RSP_DATA header
 * fields plus the raw 16-bit samples, plus the local end-of-span time we derived
 * for it. Nothing is pre-computed, so a recovery can rebuild the time axis with
 * the same code path a live report uses.
 *
 * THREE DIFFERENT FAILURES, THREE DIFFERENT ANSWERS (doc 05 section 6.5)
 * ---------------------------------------------------------------------
 *   1. Cloud unreachable, PIC series fine
 *        -> store the raw block. Continuity with the PIC is intact, so on
 *           recovery the stored blocks and the current one are one continuous
 *           series and are binned together.
 *   2. PIC series not received at all (link failure, while the PIC still runs)
 *        -> continuity is broken: we cannot know what happened in the gap or
 *           where the axis went.
 *
 *           V064 P-4: this NO LONGER deletes what is buffered. The hazard was
 *           never "old data exists" - it was "unrelated data joined end-to-end
 *           as if it were continuous", which produces a plausible but wrong
 *           hourly series. Deleting stops the bad join by destroying the input,
 *           trading a possible error for a certain loss. A marker stops exactly
 *           the same join and keeps the data:
 *
 *               [ ... block k ][ GAP ][ block k+1 ... ]
 *
 *           The carry is cut at the marker and never carried across it, and the
 *           runs on either side are each independently valid. The marker rides
 *           in the NEXT stored block's header (gapBefore), so no special block
 *           type and no second file format is needed.
 *
 *           The cost asymmetry drove this: a run wrongly kept can still be
 *           discarded later, a run wrongly deleted cannot be recovered.
 *   3. The PIC reports its own time as invalid
 *        -> its start_time cannot be trusted; the caller falls back to the last
 *           known-good report time. Handled in the binning layer, not here.
 *
 * Capacity is FLASH_BUFFER_BLOCKS (0..4). 0 disables buffering: each report then
 * stands alone and a failed publish is dropped, which is the simplest possible
 * behaviour and a legitimate configuration.
 */

#pragma once
#include "Particle.h"
#include "app_config.h"
#include "pic_link.h"

#define FLASH_BLOCK_MAGIC   0x4C534233UL   /* "LSB3" - bumped from LSB2 for the V064 P-4 gapBefore field */

// Stored header, written ahead of the raw samples in each block file.
struct FlashBlockHeader {
  uint32_t magic;
  uint32_t seq;              // monotonic: fixes the replay order after a reboot
  uint8_t  gapBefore;        // V064 P-4: 1 = a confirmed discontinuity sits between the
                             //   PREVIOUS block and this one. Replay must not carry an
                             //   unfinished bucket across it; the two sides are separate
                             //   valid runs, not one series with a hole in the middle.
  uint32_t startLocal;       // span start we resolved for this report
  uint8_t  startValid;       // 0 = the start was back-computed, not PIC-supplied
  uint32_t endLocal;         // span end (the PIC wake instant) - meaningful only if endValid
  uint8_t  endValid;         // Appendix F.3: 0 = no absolute end (no cloud time); do NOT
                             //   treat endLocal as an epoch on replay. Symmetric with startValid.
  uint32_t sampleCount;      // samples stored after this header
  uint32_t totalCaptures;    // PIC hardware totals, preserved for reconciliation
  uint32_t totalImpulses;
  uint16_t overflowFfff;
  uint32_t sampleIntervalMs;
};

// Store one report. Returns false if buffering is disabled or the write failed.
bool flashBufferStore(const FlashBlockHeader &hdr, const PicSample *s, uint32_t n);

// How many blocks are currently held.
uint16_t flashBufferCount();

// Load the i-th oldest block. 'maxSamples' bounds the caller's array.
// Returns the sample count, or a negative value on failure.
int flashBufferLoad(uint16_t oldestIndex, FlashBlockHeader *hdr,
                    PicSample *out, uint32_t maxSamples);

// Drop everything.
//
// V064 P-3/P-4: this is now called on exactly ONE occasion - a report that was
// confirmed DELIVERED to the cloud, where the blocks have served their purpose -
// plus the timebase-change path, where the old blocks belong to an axis that no
// longer exists. It is NO LONGER the response to a PIC link failure; that case
// records a marker instead (see flashBufferMarkGap). Preservation is the default,
// and deletion now requires a positive verdict rather than a missing one.
void flashBufferClear(const char *reason);

// V064 P-4: record a CONFIRMED discontinuity at the current end of the ring.
// Held blocks are untouched; the next block stored is stamped gapBefore=1, so a
// later replay knows the two sides must not be joined. Idempotent: marking twice
// with no intervening store leaves one marker, because a run of dead sessions is
// still just one hole in the series.
void flashBufferMarkGap(const char *reason);

// True while a marker is recorded but not yet attached to a block (i.e. no block
// has been stored since). Exposed for logging and for the host tests.
bool flashBufferGapPending();

// Restore the ring bookkeeping at boot.
void flashBufferBegin();
