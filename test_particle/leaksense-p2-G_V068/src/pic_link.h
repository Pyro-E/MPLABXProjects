/*
 * pic_link.h  -  Framed packet link to the PIC18F06Q40 flow meter.
 *                Photon V063, matching PIC firmware V022 (ForPhotonV063) +
 *                Appendix E/F/G/H. The frame format is unchanged since V061; the
 *                PIC needs no rebuild for the Appendix H Photon changes (H.13.2).
 *
 * V061 WIRE CHANGE (Appendix E, PIC is authoritative):
 *   - The old REQ_SET_PARAM (0x03), which distinguished its meaning by LENGTH
 *     (len 8 = leak only, len 16 = leak + report grid), is SPLIT into two
 *     fixed-length write packets so length can never again stand in for meaning:
 *
 *       REQ_SET_LEAK (0x03, len 8 FIXED) : leak1_counts, leak1_window_s,
 *                                          leak2_counts, leak2_window_s (4x u16 BE)
 *       REQ_SET_GRID (0x0F, len 8 FIXED) : align_anchor (u32 BE),
 *                                          report_interval (u32 BE)
 *
 *     0x03 keeps its number and is byte-for-byte the old 8-byte leak write.
 *     0x0F is newly assigned (was an unused code). Both are ALWAYS answered:
 *     RSP_ACK(0x8E) echoing the func on success, RSP_NAK(0x8F) reason
 *     NAK_BAD_LEN(0x02) on a wrong length. Nothing is silently dropped anymore.
 *     The len-16 REQ_SET_PARAM form and its 8-byte fallback are GONE; the PIC
 *     now NAKs a 16-byte write.
 *
 * V057 WIRE FACTS (verified against the PIC sources, still current):
 *   - RSP_DATA header is 23 bytes / 7 fields:  start_time, start_time_valid,
 *     sample_count, total_captures, total_impulses, overflow_ffff,
 *     sample_interval_ms. (FlowReport.c: XHDR_BYTES = 23, SEND_HEADER.)
 *   - RSP_PARAM is 16 bytes: the 4 leak u16 plus align_anchor(u32) and
 *     report_interval(u32), both seconds, big-endian. (FlowReport_SendParam.)
 *     REQ_GET_PARAM (0x02) -> RSP_PARAM (0x82) len 16 is UNCHANGED by Appendix E:
 *     reads have no atomicity problem and one round-trip confirms both halves.
 *   - REQ_GET_LOCO (0x0E) -> RSP_LOCO (0x8A), 10 bytes, Q7 (128 = 1.0).
 *                (The "V040" labels below refer to the protocol generation that
 *                introduced framing; the wire format is unchanged since then
 *                apart from the 16-bit sample format and funcs 0x08..0x0D.)
 *
 * Implements the "PIC <-> Photon2 Interface Specification" (Flow-Meter Packet
 * Protocol, WAKE handshake, Leak/Valve control). This REPLACES the old raw
 * single-byte link (0xF0 wake + 0xAA -> raw COUNT+samples) and the home-grown
 * 0xC0 config frame. From V040 everything after wake-up is a CRC-framed packet.
 *
 *   Link        : UART 38400 8N1 on Serial1, 3.3 V. PIC = peripheral, P2 = host.
 *   Frame       : AA 55 | func | len_hi len_lo | data[len] | crc_hi crc_lo
 *   Endianness  : every multi-byte field is big-endian / MSB-first.
 *   CRC         : CRC-16/MODBUS (poly 0xA001, init 0xFFFF, no final XOR),
 *                 over func+len+data (AA 55 excluded), sent big-endian.
 *
 * POWER-GATING MODEL (PIC firmware V048): the D10 WAKE handshake is GONE. RC4 on
 * the PIC no longer signals "comms ready" on D10; it now drives a P-MOS that
 * switches the Photon's SUPPLY (RC4 LOW -> powered, RC4 HIGH -> unpowered). So
 * "being powered" already means "the PIC wants a session -- talk now". The
 * Photon therefore never checks D10, never sends 0xF0, and cannot wake itself.
 * It ends each session with PKT_PHOTON_OFF_REQ (func 0x07) so the PIC cuts power.
 *
 * Send rule: send immediately (the PIC is powered and listening whenever we run).
 * wakeIsHigh()/waitWakeHigh()/ensureWake() are kept only so old callers compile;
 * they now always report "ready". One REQ at a time: wait for its RSP before
 * sending anything else; resend on timeout/bad CRC. D10 stays pulled up, unused.
 *
 * BEGINNER NOTE:
 *   This header DECLARES (announces) a class called PicLink and the data
 *   structures it uses. The actual code that DOES the work lives in
 *   pic_link.cpp. A header is like a table of contents / promise list; the .cpp
 *   is where the promises are kept.
 *
 *   "Framed packet" means every message is wrapped in a fixed envelope so the
 *   receiver can tell where a message starts, how long it is, and whether it
 *   arrived intact (that is the job of the CRC check at the end).
 */

#pragma once             // Include this header only once per build (avoids duplicate definitions).
#include "Particle.h"    // Device-OS library: gives us Serial1, digitalRead(), millis(), etc.
#include "app_config.h"  // Our own settings file (pin numbers, timeouts, default parameters).

// 10-bit sample index -> at most 1024; the protocol caps a batch at 1000.
#define PIC_BYTES_PER_SAMPLE 2      // 16-bit pulses only, 2 bytes/sample (COMPRESS_METHOD_NOCOMP_16).
#define PIC_MAX_SAMPLES      1000u  // We never expect more than 1000 samples in one batch ('u' = unsigned).

// RSP_DATA payload = this fixed header, then sample_count * 2 bytes.
// PIC side: FlowReport.c  #define XHDR_BYTES.
//
// V064 P-5: with PHOTON_BATCH_SEQ_ENABLE the header grows by ONE byte -
// data[23] = batch_seq - and must be matched by the PIC's PCFG_BATCH_SEQ_ENABLE.
// The two settings are a PAIR; a mismatch is rejected as PIC_ERR_BAD_FRAME
// rather than misread (see app_config.h).
#if PHOTON_BATCH_SEQ_ENABLE
  #define PIC_RSP_DATA_HDR_BYTES 24u
#else
  #define PIC_RSP_DATA_HDR_BYTES 23u
#endif
// Offset of batch_seq inside the header (only present when the switch is on).
#define PIC_RSP_DATA_BATCHSEQ_OFF 23u
// RSP_PARAM payload length. 16 = current PIC (leak 4xu16 + anchor + interval).
// 8 = an older PIC that only knows the leak parameters; still accepted.
#define PIC_RSP_PARAM_BYTES_V2 16u
#define PIC_RSP_PARAM_BYTES_V1 8u
// RSP_LOCO payload length (10 bytes, big-endian, Q7 scale where 128 = 1.0).
#define PIC_RSP_LOCO_BYTES     10u
// Q7 fixed-point scale used by every LOCO field (applied_k = 128 -> factor 1.0).
#define PIC_LOCO_Q7_ONE        128.0f

// ---- Frame constants --------------------------------------------------------
// An "enum" here just gives readable names to fixed byte values.
enum {
  PIC_MARK0 = 0xAA,        // First marker byte that begins every frame (hex AA = 170).
  PIC_MARK1 = 0x55,        // Second marker byte (hex 55 = 85). "AA 55" together = "a frame starts here".
  PIC_WAKE_BYTE = 0xF0     // (Unused under power-gating.) Formerly the wake byte; the PIC is always
                           //   powered/listening whenever we run, so we never send 0xF0 anymore.
};

// ---- Function codes (spec 3) ------------------------------------------------
// These name the "type" of each message. REQ_* = we ask; RSP_* = the PIC answers.
enum {
  REQ_DATA         = 0x01,   // -> RSP_DATA      : ask the PIC for its stored flow samples.
  REQ_GET_PARAM    = 0x02,   // -> RSP_PARAM     : read leak params + report grid (16 B). Unchanged.
  REQ_SET_LEAK     = 0x03,   // 4xu16 (len 8) -> RSP_ACK/NAK : write the 4 leak parameters ONLY.
                             //   Appendix E: same number and bytes as the old 8-byte SET_PARAM.
  REQ_GET_VALVE    = 0x04,   // -> RSP_VALVE     : ask the PIC for valve status.
  REQ_VALVE_UNLOCK = 0x05,   // 1B flags -> RSP_ACK/NAK : tell the PIC to clear a valve lock.
  PKT_SYS_RESET    = 0x06,   // (no reply)       : tell the PIC to reset itself.
  PKT_PHOTON_OFF_REQ = 0x07, // (no reply)       : Photon -> PIC "cut my power, I'm done".
  REQ_POWER_STATE  = 0x08,   // -> RSP_POWER_STATE : ask if the PIC is in its initial power-hold (no payload).
  REQ_PHOTON_CFG   = 0x09,   // -> RSP_PHOTON_CFG : ask the PIC for our timing+debug config (no payload).
  PKT_KEEPALIVE    = 0x0A,   // (no reply)       : Photon -> PIC "alive, still connecting"; keeps PIC power up.
  PKT_DATA_RECEIVED = 0x0B,  // (no reply)       : Photon -> PIC "RSP_DATA received & stored OK" -> PIC COMMITS
                             //                    (consume-on-ACK). Send ONLY after CRC ok AND stored.
  PKT_TIME_SYNC    = 0x0C,   // -> TIME_RECEIVED : Photon -> PIC absolute cloud time for LOCO. 5 bytes:
                             //                    data[0]=time_valid, data[1..4]=T_now epoch BE.
  PKT_TIME_RECEIVED = 0x0D,  // PIC -> Photon    : "TIME_SYNC received & applied" (dedicated ack; len 0).
  REQ_GET_LOCO     = 0x0E,   // -> RSP_LOCO      : read the PIC's oscillator-calibration state (no payload).
  REQ_SET_GRID     = 0x0F,   // anchor u32 + interval u32 (len 8) -> RSP_ACK/NAK : write the report grid ONLY.
                             //   Appendix E: newly assigned code; report grid is independent of leak logic.
  RSP_DATA         = 0x81,   // The PIC's reply carrying flow samples.
  RSP_PARAM        = 0x82,   // The PIC's reply carrying its 4 leak parameters.
  RSP_VALVE        = 0x84,   // The PIC's reply carrying valve status.
  RSP_POWER_STATE  = 0x88,   // PIC reply to REQ_POWER_STATE: data[0] = 0 INITIAL / 1 NORMAL.
  RSP_PHOTON_CFG   = 0x89,   // PIC reply to REQ_PHOTON_CFG: 13-byte config block (see PicPhotonCfg).
  RSP_LOCO         = 0x8A,   // PIC reply to REQ_GET_LOCO: 10-byte LOCO status block (see PicLocoStatus).
  RSP_ACK          = 0x8E,   // The PIC's "OK, request accepted" reply.
  RSP_NAK          = 0x8F    // The PIC's "request rejected" reply (reason byte follows).
};

// ---- NAK reasons (RSP_NAK data[0], spec 3) ----------------------------------
// When the PIC says "rejected" (NAK), it includes one of these reason codes.
enum {
  NAK_BAD_CRC  = 0x01,   // The PIC thought our message was corrupt (CRC mismatch).
  NAK_BAD_LEN  = 0x02,   // The PIC thought our length field was wrong.
  NAK_BAD_FUNC = 0x03,   // The PIC did not recognize our function code.
  NAK_BUSY     = 0x04    // The PIC is busy and cannot serve the request right now.
};

// ---- Result codes (negative = failure) --------------------------------------
// Our own functions return one of these. 0 means success; negatives mean errors.
enum {
  PIC_OK            =  0,   // Everything worked.
  PIC_ERR_NO_WAKE   = -1,   // WAKE never went HIGH after 0xF0  (PIC never woke up).
  PIC_ERR_TIMEOUT   = -2,   // no (complete) response in time   (PIC stayed silent).
  PIC_ERR_BAD_CRC   = -3,   // response framed but CRC failed    (reply arrived corrupted).
  PIC_ERR_BAD_FRAME = -4,   // marker/len garbage                (reply was malformed).
  PIC_ERR_WRONG_RSP = -5,   // valid frame, unexpected func      (got a reply of the wrong type).
  PIC_ERR_NAK       = -6,   // PIC returned RSP_NAK (see lastNak())  (request understood but rejected).
  PIC_ERR_OVERFLOW  = -7    // response longer than caller buffer (reply too big to store).
};

// ---- Payload structs --------------------------------------------------------
// These structs describe the DECODED contents of messages, in easy-to-use form.
struct PicSample {           // decoded 10-14 packed sample
                             //   One flow reading after we unpack it from 3 raw bytes.
  uint16_t index;            // DEPRECATED: no per-sample index on the wire anymore (16-bit format).
                             //   Always 0. Place samples by their position in the batch instead.
  uint16_t pulses;           // 0..65535 (16-bit)        : how many flow pulses were counted.
};

// RSP_DATA header, decoded from the 23-byte prefix that precedes the samples.
// PIC source of truth: FlowReport.c SEND_HEADER (all fields big-endian).
//
// Why three separate totals exist: when the PIC's 1024-slot ring overruns (a
// report was delayed), the surviving sample series carries only part of the
// period, but total_captures / total_impulses are hardware totals and stay
// correct. The difference is exactly the volume the series could not carry.
struct PicReportInfo {
  uint32_t startTime;        // 1: epoch at which THIS report's span STARTS.
                             //    The PIC echoes the timebase we gave it in
                             //    TIME_SYNC, so this is a LOCAL epoch here.
  uint8_t  startTimeValid;   // 2: 0 -> startTime is meaningless, back-compute it.
  uint32_t sampleCount;      // 3: number of samples that follow the header.
  uint32_t totalCaptures;    // 4: captures in this report period (>= sampleCount
                             //    if the ring overran).
  uint32_t totalImpulses;    // 5: true pulses in this report period (>= the sum
                             //    of the received samples if the ring overran).
  uint16_t overflowFfff;     // 6: samples clamped at 0xFFFF (cell limit).
  uint32_t sampleIntervalMs; // 7: the interval the PIC INTENDED between captures,
                             //    already LOCO-corrected. Milliseconds.
  // 8 (V064 P-5, only when PHOTON_BATCH_SEQ_ENABLE): the PIC's identifier for
  //    THIS batch. It stays the same across every retransmission of the batch
  //    and only advances once the batch has been ACKed, so comparing it against
  //    the last one we stored is what makes storage idempotent. batchSeqValid
  //    is 0 on a 23-byte header, where no identifier exists and the duplicate
  //    test must therefore be skipped rather than guessed at.
  uint8_t  batchSeq;
  uint8_t  batchSeqValid;
};

struct PicParams {           // payload of REQ_SET_LEAK / leak part of RSP_PARAM (4xu16 BE)
                             //   The PIC's four leak-detection settings.
  uint16_t leak1_counts;     // alert 1 threshold (counts)  -> temporary lock
  uint16_t leak1_window_s;   // alert 1 window (seconds)
  uint16_t leak2_counts;     // alert 2 threshold (counts)  -> permanent lock
  uint16_t leak2_window_s;   // alert 2 window (seconds)
};

// Payload of REQ_SET_GRID / grid part of RSP_PARAM. These two define WHEN the PIC
// reports: report times are the grid points  anchor + n * interval.  They are
// completely independent of each other and of the Photon's bucket width.
struct PicGridParams {
  uint32_t anchorSec;        // grid origin as seconds-of-day (HH*3600+MM*60+SS).
                             //   Midnight = 0. Compared by the PIC against
                             //   epoch % 86400, so it is LOCAL seconds-of-day.
  uint32_t intervalSec;      // seconds between report grid points (48 h = 172800).
};

// RSP_LOCO (0x8A) - the PIC's oscillator self-calibration state. Bench readout.
// Every *_precision / applied_k field is Q7: divide by 128.0 for the factor.
struct PicLocoStatus {
  uint16_t hfPrecision;      // factor measured by the HFINTOSC-vs-LFINTOSC pass
  uint8_t  hfCalibrated;     // 1 = that one-shot measurement has run
  uint8_t  hfInRange;        // 1 = its result passed the +/-15 % trust gate
  uint8_t  cloudCalibrated;  // 1 = at least one cloud-time evaluation was trusted
  uint16_t cloudPrecision;   // factor after the most recent cloud evaluation
  uint8_t  cloudInRange;     // 1 = the latest cloud evaluation passed the gate
  uint16_t appliedK;         // the factor actually in use (128 = 1.0)
};

// ---- config the PIC hands us at boot (RSP_PHOTON_CFG, 0x89) ------------------
// The PIC is the single source of truth for the sample TIMING (so PIC and Photon
// can never disagree -> no more GPM/gallon mismatch) and for the DEBUG toggles
// (so we retune from the PIC's fast ~30 s build instead of reflashing the Photon).
// If provided==false the PIC told us to use our own compiled defaults.
struct PicPhotonCfg {
  bool     provided;          // true = PIC supplied these; false = use our defaults
  uint8_t  version;           // wire-format version
  uint32_t captureIntervalMs; // real per-sample window in ms (rate divisor x1000)
  uint16_t samplesPerReport;  // PIC's APP_SAMPLES_PER_REPORT
  bool     fastBench;         // true = skip cloud (virtual clock, sim publish)
  bool     debugDataseries;   // true = stream per-sample lines over USB-CDC
  uint8_t  missedFillMode;    // 0 = ZERO, 1 = AVERAGE
  uint16_t serialDelayMs;     // boot delay before the log burst
};

struct PicValve {            // RSP_VALVE payload (8 B)
                             //   The current state of the PIC's motorized valve.
  uint8_t  pwr_pin;          // VALVE_PWR  level (0/1)     : is valve power on?
  uint8_t  ctrl_pin;         // VALVE_CTRL level (0/1)     : valve direction signal.
  uint8_t  motion;           // 0..6 (see spec 4.3)        : current valve motion state.
  uint8_t  lock_flags;       // bit0=temp, bit1=perm (0x03=both) : which locks are active.
  uint32_t temp_lock_count;  // cumulative # of temporary locks   : how many times temp-locked so far.
};

// Valve lock bit layout (shared by lock_flags and the UNLOCK command).
// These single-bit values can be combined (added) to mean "both".
enum {
  VALVE_LOCK_TEMP = 0x01,    // Bit 0 set = a TEMPORARY lock is active (auto-clears after ~10 min).
  VALVE_LOCK_PERM = 0x02,    // Bit 1 set = a PERMANENT lock is active (needs unlock or reset).
  VALVE_LOCK_BOTH = 0x03     // Both bits set (0x01 | 0x02 = 0x03) = both locks at once.
};

// ---- PKT_PHOTON_OFF_REQ reasons (data[0], power-gating model) ---------------
// The PIC now switches the Photon's SUPPLY through a P-MOS on RC4, so the Photon
// ends every session by telling the PIC "cut my power". data[0] says why.
enum PhotonOffReason : uint8_t {
  OFF_REASON_DONE       = 0x00,   // all comms finished normally
  OFF_REASON_CLOUD_FAIL = 0x01    // could not reach the cloud
};

// ---- REQ_POWER_STATE / RSP_POWER_STATE (boot power-hold handshake) ----------
// At a cold power-up the PIC (V049) may hold the Photon FULLY POWERED for an initial
// ~10-minute window so the flow meter can be watched live. RSP_POWER_STATE data[0]
// tells the Photon which mode it is in. In INITIAL the Photon must STAY POWERED and
// must NOT send func 0x07 -- the PIC owns the window and cuts power when it ends.
enum {
  POWER_STATE_INITIAL = 0,   // Cold-boot hold active: stay powered; the PIC cuts power at ~10 min.
  POWER_STATE_NORMAL  = 1    // Normal operation (or a WDT/soft reset): run a normal session.
};

// A "class" bundles related data and the functions that act on it. PicLink is
// our object that knows how to talk to the PIC chip.
class PicLink {
public:                                              // "public" = usable from outside the class.
  void begin(unsigned long baud = 38400);            // Set up the serial port + D10 pin. Default speed 38400.
  bool wakeIsHigh();                                 // Power-gating: always true (D10 no longer a comms signal).

  // REQ_DATA -> RSP_DATA. Decodes samples into out[]. Returns sample count
  // (>=0) or a PIC_ERR_* code (<0). Works for both host-initiated and
  // PIC-initiated uploads (the WAKE handshake no-ops when WAKE is already HIGH).
  int requestData(PicSample *out, uint16_t maxSamples, PicReportInfo *info = nullptr);  // Ask for flow data; fill out[] and (optionally) the V051 header fields.

  // REQ_GET_PARAM -> RSP_PARAM. true on success (out populated). UNCHANGED by
  // Appendix E: the current PIC answers 16 bytes - the 4 leak values plus the
  // report grid (anchor + interval). Pass 'grid' to receive the grid part; it is
  // left untouched (and gridValid is set false) if an older 8-byte PIC answers.
  // One read confirms both halves, so this is the read-back used to prove that a
  // SET_GRID actually landed (an ACK only proves the frame parsed).
  bool getParams(PicParams &out, PicGridParams *grid = nullptr,
                 bool *gridValid = nullptr);

  // REQ_SET_LEAK (0x03, len 8 FIXED) -> RSP_ACK/NAK. true only on ACK.
  // Writes the 4 leak parameters and nothing else. This is byte-for-byte the old
  // 8-byte SET_PARAM; only its name changed. A wrong length is NAKed
  // (NAK_BAD_LEN), never silently dropped.
  bool setLeak(const PicParams &in);

  // REQ_SET_GRID (0x0F, len 8 FIXED) -> RSP_ACK/NAK. true only on ACK.
  // Writes the report grid (anchor + interval) and nothing else. Independent of
  // the leak parameters, so it can be sent and confirmed on its own.
  //
  // An ACK means PARSED, not APPLIED: a PIC built with PIC_USE_OWN_TIMING ACKs
  // the frame but keeps its own grid (Appendix E.2). Callers that must know
  // whether the grid actually landed read it back with getParams() and compare;
  // see syncGridWithPic() in leaksense.cpp.
  bool setGrid(const PicGridParams &grid);

  // REQ_GET_LOCO -> RSP_LOCO (10 B). Reads the PIC's oscillator-calibration
  // state for the bench log. true on success (out populated).
  bool getLocoStatus(PicLocoStatus &out);

  // REQ_GET_VALVE -> RSP_VALVE. true on success (out populated).
  bool getValve(PicValve &out);                      // Read the valve status into 'out'.
  // REQ_VALVE_UNLOCK -> RSP_ACK/NAK. flags: VALVE_LOCK_*. true only on ACK.
  bool unlockValve(uint8_t flags);                   // Clear one or both valve locks.

  // PKT_SYS_RESET. No reply expected (the PIC resets and clears the perm lock).
  void sysReset();                                   // Tell the PIC to reboot.

  // PKT_PHOTON_OFF_REQ (func 0x07). One-way, no reply (like sysReset). Tells the
  // PIC "I'm done, cut my power". data[0]=reason (OFF_REASON_*). The PIC then
  // drives RC4 HIGH -> P-MOS off -> Photon unpowered. true if the frame was sent.
  bool sendPhotonOff(uint8_t reason);                // func 0x07, len 1, data[0]=reason; no reply.

  // func 0x0B, no reply. Call ONLY after a RSP_DATA batch was received with a
  // good CRC AND durably stored (published or written to flash). Tells the PIC the
  // batch is safe so it advances its FIFO read pointer (consume-on-ACK). If never sent,
  // the PIC retransmits the same batch on the next REQ_DATA.
  //
  // V064 P-5: with PHOTON_BATCH_SEQ_ENABLE the ACK carries len 1 = the batch_seq
  // being acknowledged, echoed back exactly as received, so the PIC can tell WHICH
  // batch was accepted instead of assuming it was the newest. With the switch off
  // it is len 0, byte-for-byte the V063 frame.
  //
  // Send exactly once per batch - EXCEPT for a detected duplicate, which must be
  // re-ACKed without being stored again. An un-ACKed batch is retransmitted by the
  // PIC forever, so skipping the re-ACK would deadlock the link at that batch.
  bool sendDataReceived(uint8_t batchSeq = 0u, bool seqValid = false);

  // func 0x0C -> waits for PKT_TIME_RECEIVED (0x0D), retransmitting on loss so the PIC
  // reliably gets the cloud time. len 5: time_valid(1) + T_now epoch BE(4). Send
  // time_valid=0 when no cloud time is available (PIC then skips LOCO for that session).
  bool sendTimeSync(bool timeValid, uint32_t epochNow);

  // PKT_KEEPALIVE (func 0x0A). One-way, no reply (like sysReset). Sent periodically
  // while waiting for the cloud so the PIC's ACTIVE idle backstop does not cut our
  // power mid-connect. Carries no payload/meaning beyond "I'm alive". true if sent.
  bool sendKeepalive();                              // func 0x0A, len 0; no reply.

  // REQ_POWER_STATE (func 0x08) -> RSP_POWER_STATE (0x88). Asks whether the PIC is in
  // its initial cold-boot power-hold. On success sets *out to the state byte
  // (POWER_STATE_INITIAL / POWER_STATE_NORMAL) and returns PIC_OK; otherwise returns a
  // PIC_ERR_* code and leaves *out unchanged. Sends no payload.
  int getPowerState(uint8_t *out);                   // Ask the PIC if it's in the initial power-hold.
  int getPhotonConfig(PicPhotonCfg *out);            // Ask the PIC for our timing+debug config (0x09).

  // Throw away any leftover bytes sitting in the receive buffer.
  //
  // V066: promoted from private to public. It was internal-only because the only
  // callers were sendFrame() (drop stale bytes before listening) and transact()
  // (start a retry from a known-empty path). setup() now needs it too: after the
  // PIC_QUIET_MS silence that follows power-up, the RX FIFO may hold garbage
  // clocked in by the supply transition, and that garbage must go before the
  // first CFG exchange rather than at the moment of the first transmit.
  //
  // Safe to expose: it takes no arguments, holds no invariant, and only drains
  // Serial1. It does NOT touch WAKE, _lastTxMs, or any framing state, so calling
  // it outside a transaction cannot desynchronise the link layer.
  void flushRx();

  // Reason byte from the most recent RSP_NAK (0 if none).
  uint8_t lastNak() const { return _lastNak; }       // Tiny helper: report why the last request was rejected.
                                                     //   ("const" = this function does not change the object.)

  // millis() of the last frame we put on the wire. The PIC cuts our power after
  // TIMEOUT_NO_MORE_MSG_MS (20 s) without a valid packet, so any long host-side
  // activity (cloud publish bursts) must watch this and pump a KEEPALIVE.
  uint32_t lastTxMillis() const { return _lastTxMs; }

  // Appendix H.7.3: diagnostics from the most recent readFrame(), so a "no valid
  // reply" can be told apart on the wire - a timeout with rx 0 bytes (line/power/
  // timing) vs a CRC/wrong-func with rx N bytes (signal quality). Correlated with
  // the PIC's txdrops= counter these split the fault to one side in one round.
  uint16_t lastRxBytes() const { return _lastRxBytes; }   // bytes consumed after the AA 55 marker
  uint8_t  lastRxFunc()  const { return _lastRxFunc; }     // func of the last framed reply (0 if none)
  uint16_t lastRxLen()   const { return _lastRxLen; }      // declared payload len of the last frame

  // Human-readable name for a PIC_ERR_* code (Appendix H.7.3). Static so callers
  // can decode a return value without an instance.
  static const char *picErrName(int code) {
    switch (code) {
      case PIC_OK:            return "ok";
      case PIC_ERR_NO_WAKE:   return "no-wake";
      case PIC_ERR_TIMEOUT:   return "timeout";
      case PIC_ERR_BAD_CRC:   return "CRC";
      case PIC_ERR_BAD_FRAME: return "bad-frame";
      case PIC_ERR_WRONG_RSP: return "wrong-func";
      case PIC_ERR_NAK:       return "NAK";
      case PIC_ERR_OVERFLOW:  return "overflow";
      default:                return "unknown";
    }
  }

private:                                             // "private" = internal-only; not callable from outside.
  // --- low level ---
  static uint16_t crc16(const uint8_t *p, uint16_t n);   // Compute the CRC-16 checksum over n bytes at p.
                                                         //   "static" = belongs to the class, not one object.
  bool ensureWake();                                   // spec 5.3 send rule : make sure the PIC is awake first.
  bool sendFrame(uint8_t func, const uint8_t *data, uint16_t len);  // Build + transmit one framed packet.
  // Read one framed packet; resynchronises on AA 55. Returns PIC_OK or PIC_ERR_*.
  int  readFrame(uint8_t *func, uint8_t *data, uint16_t cap,
                 uint16_t *outLen, uint32_t timeoutMs);  // Receive one packet and verify its CRC.
  // One full transaction with retry (spec 5.4 / 7): send REQ, wait for the
  // matching RSP. wantFunc is the expected reply func. Returns PIC_OK/PIC_ERR_*.
  int  transact(uint8_t reqFunc, const uint8_t *reqData, uint16_t reqLen,
                uint8_t wantFunc, uint8_t *rspData, uint16_t rspCap,
                uint16_t *rspLen, uint32_t timeoutMs);   // Send a request and wait for its reply, with retries.

  bool readByte(uint8_t *out, uint32_t timeoutMs);   // Read exactly one byte (or time out).
  bool waitWakeHigh(uint32_t timeoutMs);             // Wait until WAKE goes HIGH (or time out).
                                                     // (V066: flushRx() moved to the public section above.)

  uint8_t  _lastNak = 0;                             // Stores the most recent NAK reason (0 = none yet).
  uint32_t _lastTxMs = 0;                            // millis() of the last frame sent (keepalive pump).
                                                     //   (The leading underscore is a naming style for "private".)
  uint16_t _lastRxBytes = 0;                         // Appendix H.7.3: bytes read after the marker on the last readFrame
  uint8_t  _lastRxFunc  = 0;                          // func of the last frame that reached the header
  uint16_t _lastRxLen   = 0;                          // declared payload length of the last frame

  // Scratch for assembling outgoing frames (max payload = RSP_DATA-ish; the
  // host only ever SENDS small payloads, so 16 B is plenty).
  uint8_t _txbuf[7 + 16];                            // Temporary buffer to build a frame: 7 overhead + 16 data bytes.
};                                                   // End of the class (note the required ";").