/*
 * pic_link.h  -  Framed packet link to the PIC18F06Q40 flow meter (V040).
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
 *   WAKE line   : PIC_WAKE_PIN (D10) is NO LONGER read by firmware. D10 is now
 *                 wired as a one-time hardware power-enable line that the PIC
 *                 drives LOW to turn the Photon on; it carries no protocol
 *                 meaning once the Photon is running.
 *   Wake byte   : 0xF0 (host -> PIC) sent before every frame, to make sure a
 *                 PIC that is idle/asleep on the UART wakes its receiver up.
 *
 * Send rule (post-D10-repurpose): every send unconditionally writes 0xF0 then
 * the frame -- there is no GPIO handshake to wait on anymore. One REQ at a
 * time: wait for its RSP before sending anything else; resend on timeout/bad
 * CRC. Since D10 no longer signals "PIC has data", the host now asks for data
 * (REQ_DATA, i.e. "TR/TX") on its own timer instead of reacting to a WAKE edge.
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
#define PIC_BYTES_PER_SAMPLE 3      // Each flow sample is packed into exactly 3 bytes on the wire.
#define PIC_MAX_SAMPLES      1000u  // We never expect more than 1000 samples in one batch ('u' = unsigned).

// ---- Frame constants --------------------------------------------------------
// An "enum" here just gives readable names to fixed byte values.
enum {
  PIC_MARK0 = 0xAA,        // First marker byte that begins every frame (hex AA = 170).
  PIC_MARK1 = 0x55,        // Second marker byte (hex 55 = 85). "AA 55" together = "a frame starts here".
  PIC_WAKE_BYTE = 0xF0     // The single byte we send to wake a sleeping PIC (hex F0 = 240).
};

// ---- Function codes (spec 3) ------------------------------------------------
// These name the "type" of each message. REQ_* = we ask; RSP_* = the PIC answers.
enum {
  REQ_DATA         = 0x01,   // -> RSP_DATA      : ask the PIC for its stored flow samples.
  REQ_GET_PARAM    = 0x02,   // -> RSP_PARAM     : ask the PIC for its 4 leak parameters.
  REQ_SET_PARAM    = 0x03,   // 4xu16 -> RSP_ACK/NAK : send the PIC new leak parameters.
  REQ_GET_VALVE    = 0x04,   // -> RSP_VALVE     : ask the PIC for valve status.
  REQ_VALVE_UNLOCK = 0x05,   // 1B flags -> RSP_ACK/NAK : tell the PIC to clear a valve lock.
  PKT_SYS_RESET    = 0x06,   // (no reply)       : tell the PIC to reset itself.
  RSP_DATA         = 0x81,   // The PIC's reply carrying flow samples.
  RSP_PARAM        = 0x82,   // The PIC's reply carrying its 4 leak parameters.
  RSP_VALVE        = 0x84,   // The PIC's reply carrying valve status.
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
  uint16_t index;            // 0..1023 (high 10 bits)   : which time-slot this sample belongs to.
  uint16_t pulses;           // 0..16383 (low 14 bits)   : how many flow pulses were counted.
};

struct PicParams {           // REQ_SET_PARAM / RSP_PARAM payload (4xu16 BE)
                             //   The PIC's four leak-detection settings.
  uint16_t leak1_counts;     // alert 1 threshold (counts)  -> temporary lock
  uint16_t leak1_window_s;   // alert 1 window (seconds)
  uint16_t leak2_counts;     // alert 2 threshold (counts)  -> permanent lock
  uint16_t leak2_window_s;   // alert 2 window (seconds)
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

// A "class" bundles related data and the functions that act on it. PicLink is
// our object that knows how to talk to the PIC chip.
class PicLink {
public:                                              // "public" = usable from outside the class.
  void begin(unsigned long baud = 38400);            // Set up the serial port. Default speed 38400. (D10/WAKE is no longer touched here.)

  // REQ_DATA -> RSP_DATA ("TR/TX"). Decodes samples into out[]. Returns sample
  // count (>=0) or a PIC_ERR_* code (<0). Always host-initiated now (called on
  // a timer) since D10 no longer signals "PIC has data".
  int requestData(PicSample *out, uint16_t maxSamples);  // Ask for flow data; fill the out[] array.

  // REQ_GET_PARAM -> RSP_PARAM. true on success (out populated).
  bool getParams(PicParams &out);                    // Read the PIC's 4 leak parameters into 'out'.
  // REQ_SET_PARAM -> RSP_ACK/NAK. true only on ACK.
  bool setParams(const PicParams &in);               // Write 4 leak parameters from 'in' to the PIC.

  // REQ_GET_VALVE -> RSP_VALVE. true on success (out populated).
  bool getValve(PicValve &out);                      // Read the valve status into 'out'.
  // REQ_VALVE_UNLOCK -> RSP_ACK/NAK. flags: VALVE_LOCK_*. true only on ACK.
  bool unlockValve(uint8_t flags);                   // Clear one or both valve locks.

  // PKT_SYS_RESET. No reply expected (the PIC resets and clears the perm lock).
  void sysReset();                                   // Tell the PIC to reboot.

  // Reason byte from the most recent RSP_NAK (0 if none).
  uint8_t lastNak() const { return _lastNak; }       // Tiny helper: report why the last request was rejected.
                                                     //   ("const" = this function does not change the object.)

private:                                             // "private" = internal-only; not callable from outside.
  // --- low level ---
  static uint16_t crc16(const uint8_t *p, uint16_t n);   // Compute the CRC-16 checksum over n bytes at p.
                                                         //   "static" = belongs to the class, not one object.
  bool ensureWake();                                   // sends the UART wake byte before every frame (no GPIO wait; see .cpp).
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
  void flushRx();                                    // Throw away any leftover bytes in the receive buffer.

  uint8_t _lastNak = 0;                              // Stores the most recent NAK reason (0 = none yet).
                                                     //   (The leading underscore is a naming style for "private".)

  // Scratch for assembling outgoing frames (max payload = RSP_DATA-ish; the
  // host only ever SENDS small payloads, so 16 B is plenty).
  uint8_t _txbuf[7 + 16];                            // Temporary buffer to build a frame: 7 overhead + 16 data bytes.
};                                                   // End of the class (note the required ";").