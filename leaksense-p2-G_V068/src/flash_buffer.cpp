/*
 * flash_buffer.cpp  -  Raw-report ring buffer for cloud outages (see flash_buffer.h).
 *
 * One file per block on the device file system. LittleFS handles the physical
 * 4 kB sectors and wear levelling, so a block is simply a logical file; a full
 * report is 23 bytes of header plus at most 1000 x 2 bytes of samples, which
 * fits comfortably inside one sector.
 */

#include "flash_buffer.h"
#include <fcntl.h>
#include <unistd.h>

#if FLASH_BUFFER_BLOCKS > 0

// The ring lives as one file per block on the device file system. The paths are
// absolute on the device ("/lsblkring.bin", "/lsblk0.bin", ...). A compile-time
// prefix lets the host regression test (Appendix F.7.3) redirect them into a
// scratch directory without touching production behaviour: with the default
// empty prefix the paths are byte-for-byte identical to before.
#ifndef FLASH_BUFFER_PATH_PREFIX
#define FLASH_BUFFER_PATH_PREFIX ""
#endif

static const char *RING_PATH      = FLASH_BUFFER_PATH_PREFIX "/lsblkring.bin";

// Ring bookkeeping, mirrored to flash so a power cut cannot lose the order.
struct RingMeta {
  uint32_t magic;
  uint16_t count;      // blocks currently held (0..FLASH_BUFFER_BLOCKS)
  uint16_t head;       // slot the NEXT block will be written to
  uint32_t nextSeq;    // monotonic sequence for ordering
  uint8_t  gapPending; // V064 P-4: a discontinuity has been confirmed and is waiting
                       //   to be stamped onto the next block stored. Persisted with
                       //   the rest of the bookkeeping because the session that
                       //   confirms the gap is usually ended by the PIC cutting
                       //   power - the marker has to survive that to be worth
                       //   anything at all.
  uint8_t  pad[3];     // explicit padding: this struct is written to flash verbatim
};
static RingMeta s_ring = {FLASH_BLOCK_MAGIC, 0u, 0u, 1u, 0u, {0u, 0u, 0u}};

static void ringSave() {
  int fd = open(RING_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (fd < 0) return;
  ssize_t w = write(fd, &s_ring, sizeof(s_ring));
  (void)w;
  close(fd);
}

void flashBufferBegin() {
  RingMeta m;
  int fd = open(RING_PATH, O_RDONLY);
  if (fd >= 0) {
    bool ok = (read(fd, &m, sizeof(m)) == (int)sizeof(m));
    close(fd);
    if (ok && m.magic == FLASH_BLOCK_MAGIC &&
        m.count <= FLASH_BUFFER_BLOCKS && m.head < FLASH_BUFFER_BLOCKS) {
      s_ring = m;
      Log.info("[DAT] flash buffer: %u/%u blocks held (nextSeq=%lu, gapPending=%u)",
               (unsigned)s_ring.count, (unsigned)FLASH_BUFFER_BLOCKS,
               (unsigned long)s_ring.nextSeq, (unsigned)s_ring.gapPending);
      return;
    }
  }
  s_ring.magic = FLASH_BLOCK_MAGIC;
  s_ring.count = 0u; s_ring.head = 0u; s_ring.nextSeq = 1u; s_ring.gapPending = 0u;
  ringSave();
  Log.info("[DAT] flash buffer: fresh (capacity %u blocks)", (unsigned)FLASH_BUFFER_BLOCKS);
}

uint16_t flashBufferCount() { return s_ring.count; }

bool flashBufferStore(const FlashBlockHeader &hdrIn, const PicSample *s, uint32_t n) {
  if (n > PIC_MAX_SAMPLES) return false;

  char path[64];
  snprintf(path, sizeof(path), FLASH_BUFFER_PATH_PREFIX "/lsblk%u.bin", (unsigned)s_ring.head);

  // Appendix H.17: if the ring is full, the slot we are about to overwrite holds
  // the OLDEST block, whose samples are LOST when we truncate it. Read its header
  // first and warn LOUDLY - the old code overwrote it silently, so a dropped
  // block during a long outage left no trace. seq / n / endValid identify exactly
  // which data is gone.
  if (s_ring.count >= FLASH_BUFFER_BLOCKS) {
    FlashBlockHeader old;
    int ofd = open(path, O_RDONLY);
    if (ofd >= 0) {
      if (read(ofd, &old, sizeof(old)) == (int)sizeof(old) && old.magic == FLASH_BLOCK_MAGIC) {
        Log.warn("[FLASH] block seq=%lu EVICTED (ring full) - n=%lu, endValid=%u, "
                 "its samples are LOST", (unsigned long)old.seq,
                 (unsigned long)old.sampleCount, (unsigned)old.endValid);
      }
      close(ofd);
    }
  }

  FlashBlockHeader hdr = hdrIn;
  hdr.magic       = FLASH_BLOCK_MAGIC;
  hdr.seq         = s_ring.nextSeq;
  hdr.sampleCount = n;
  // V064 P-4: a discontinuity confirmed since the last store belongs BETWEEN the
  // previous block and this one, so it rides on this block's header. Stamping it
  // here (rather than writing a marker block of its own) keeps the ring one file
  // per report and leaves the replay order untouched.
  hdr.gapBefore   = s_ring.gapPending ? 1u : 0u;

  int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (fd < 0) { Log.error("[DAT] flash buffer: cannot open %s", path); return false; }

  bool ok = (write(fd, &hdr, sizeof(hdr)) == (int)sizeof(hdr));
  for (uint32_t i = 0; ok && i < n; i++) {          // samples as plain 16-bit pulses
    uint16_t v = s[i].pulses;
    ok = (write(fd, &v, sizeof(v)) == (int)sizeof(v));
  }
  close(fd);
  if (!ok) { Log.error("[DAT] flash buffer: write failed (%s)", path); return false; }

  // Appendix E.4 (g): list the seqs currently held (before this store) and the
  // seq this batch is being written as, so the 4-sector ring can be followed.
  // Held blocks carry the monotonic seqs [nextSeq-count .. nextSeq-1].
  char heldbuf[48]; int hp = 0;
  for (uint16_t i = 0; i < s_ring.count; i++) {
    uint32_t hs = s_ring.nextSeq - s_ring.count + i;
    hp += snprintf(heldbuf + hp, sizeof(heldbuf) - hp, (i == 0u) ? "%lu" : ",%lu",
                   (unsigned long)hs);
  }
  if (s_ring.count == 0u) snprintf(heldbuf, sizeof(heldbuf), "none");
  Log.info("[DAT] flash %u/%u blocks held (seq=%s) - this batch stored as seq=%lu",
           (unsigned)s_ring.count, (unsigned)FLASH_BUFFER_BLOCKS, heldbuf,
           (unsigned long)hdr.seq);

  // Ring advance. Once full the oldest block is overwritten, which is the
  // intended behaviour: recent data matters more than an unbounded backlog.
  if (hdr.gapBefore) {
    Log.warn("[FLASH] GAP MARKER attached to block seq=%lu - the series before it is "
             "NOT continuous with it (carry must not cross)", (unsigned long)hdr.seq);
    s_ring.gapPending = 0u;                        // the marker now lives in the block
  }
  s_ring.head = (uint16_t)((s_ring.head + 1u) % FLASH_BUFFER_BLOCKS);
  if (s_ring.count < FLASH_BUFFER_BLOCKS) {
    s_ring.count++;
  } else {
    // The slot we just wrote had held the oldest block; its seq was
    // nextSeq - FLASH_BUFFER_BLOCKS (before we bump nextSeq below).
    uint32_t overwritten = s_ring.nextSeq - FLASH_BUFFER_BLOCKS;
    Log.warn("[DAT] flash ring wrapped: seq=%lu overwritten", (unsigned long)overwritten);
  }
  s_ring.nextSeq++;
  ringSave();

  Log.info("[DAT] flash buffer: stored block seq=%lu n=%lu span=%lu..%lu (%u/%u held)",
           (unsigned long)hdr.seq, (unsigned long)n,
           (unsigned long)hdr.startLocal, (unsigned long)hdr.endLocal,
           (unsigned)s_ring.count, (unsigned)FLASH_BUFFER_BLOCKS);
  // Appendix H.17: the ring state in one line - held / nextSeq / oldest..newest -
  // so the 4-sector cycle can be followed without reconstructing it from the
  // per-store lines. Held blocks carry seqs [nextSeq-count .. nextSeq-1].
  if (s_ring.count > 0u) {
    uint32_t oldest = s_ring.nextSeq - s_ring.count;
    uint32_t newest = s_ring.nextSeq - 1u;
    Log.info("[FLASH] ring state: held=%u/%u nextSeq=%lu oldest=seq%lu newest=seq%lu",
             (unsigned)s_ring.count, (unsigned)FLASH_BUFFER_BLOCKS,
             (unsigned long)s_ring.nextSeq, (unsigned long)oldest, (unsigned long)newest);
  }
  return true;
}

int flashBufferLoad(uint16_t oldestIndex, FlashBlockHeader *hdr,
                    PicSample *out, uint32_t maxSamples) {
  if (oldestIndex >= s_ring.count) return -1;

  // head points one past the newest, so the oldest held block sits
  // 'count' slots behind it.
  uint16_t slot = (uint16_t)((s_ring.head + FLASH_BUFFER_BLOCKS - s_ring.count + oldestIndex)
                             % FLASH_BUFFER_BLOCKS);
  char path[64];
  snprintf(path, sizeof(path), FLASH_BUFFER_PATH_PREFIX "/lsblk%u.bin", (unsigned)slot);

  int fd = open(path, O_RDONLY);
  if (fd < 0) return -2;

  FlashBlockHeader h;
  if (read(fd, &h, sizeof(h)) != (int)sizeof(h) || h.magic != FLASH_BLOCK_MAGIC) {
    close(fd); return -3;
  }
  if (h.sampleCount > maxSamples) { close(fd); return -4; }

  for (uint32_t i = 0; i < h.sampleCount; i++) {
    uint16_t v = 0;
    if (read(fd, &v, sizeof(v)) != (int)sizeof(v)) { close(fd); return -5; }
    out[i].pulses = v;
    out[i].index  = 0u;
  }
  close(fd);
  if (hdr) *hdr = h;
  return (int)h.sampleCount;
}

void flashBufferClear(const char *reason) {
  if (s_ring.count == 0u && s_ring.gapPending == 0u) return;
  Log.info("[DAT] flash buffer cleared (%u blocks): %s",
           (unsigned)s_ring.count, reason ? reason : "");
  s_ring.count = 0u;
  s_ring.head  = 0u;
  // A pending marker only exists to keep two runs apart. With the ring emptied
  // there is no earlier run left to keep anything apart FROM, so the marker has
  // done its job and is dropped with it.
  s_ring.gapPending = 0u;
  ringSave();
}

// V064 P-4: record a confirmed discontinuity WITHOUT touching the held blocks.
void flashBufferMarkGap(const char *reason) {
  if (s_ring.gapPending) {
    // Already marked and not yet attached. Consecutive dead sessions are still
    // one hole in the series, so a second marker would say nothing new.
    Log.info("[DAT] GAP MARKER already pending (%s) - not duplicated",
             reason ? reason : "");
    return;
  }
  s_ring.gapPending = 1u;
  ringSave();
  Log.warn("[DAT] GAP MARKER recorded: %s - %u block(s) RETAINED, carry cut at the "
           "marker (V064 P-4: data preserved, the bad join is what is blocked)",
           reason ? reason : "", (unsigned)s_ring.count);
}

bool flashBufferGapPending() { return s_ring.gapPending != 0u; }

#else   /* FLASH_BUFFER_BLOCKS == 0 : buffering disabled ---------------------- */

void     flashBufferBegin() { Log.info("[DAT] flash buffer disabled (0 blocks)"); }
uint16_t flashBufferCount() { return 0u; }
bool     flashBufferStore(const FlashBlockHeader &, const PicSample *, uint32_t) { return false; }
int      flashBufferLoad(uint16_t, FlashBlockHeader *, PicSample *, uint32_t) { return -1; }
void     flashBufferClear(const char *) {}
void     flashBufferMarkGap(const char *) {}
bool     flashBufferGapPending() { return false; }

#endif
