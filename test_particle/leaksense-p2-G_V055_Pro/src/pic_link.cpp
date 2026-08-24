/*
 * pic_link.cpp  -  Framed packet link implementation (V040 protocol).
 *
 * See pic_link.h for the wire format. CRC-16/MODBUS verified against every
 * example frame in spec section 8 (REQ_DATA..PKT_SYS_RESET).
 *
 * BEGINNER NOTE:
 *   This .cpp file contains the real working code for the PicLink class that
 *   pic_link.h promised. Here is where bytes are actually sent and received
 *   over the serial wire (Serial1) to the PIC chip, and where we check that
 *   nothing got corrupted in transit (the CRC check).
 */

#include "pic_link.h"    // Bring in the declarations (the class shape, enums, structs) we are implementing.

// Grow the Serial1 RX buffer so a full RSP_DATA frame can land before we drain
// it:  7 (frame overhead) + 18 (V051 extended header) + 1000*3 (samples) = 3025
// bytes. Spec 7 asks for >= 4096.
// This special function name (acquireSerial1Buffer) is recognized by Device OS:
// if we define it, the OS uses OUR bigger buffers for Serial1 instead of tiny defaults.
hal_usart_buffer_config_t acquireSerial1Buffer(void) {
  static uint8_t txBuf[256];     // 256-byte send buffer ("static" = created once, kept for the whole program).
  static uint8_t rxBuf[4096];    // 4096-byte receive buffer, big enough for the largest reply.
  hal_usart_buffer_config_t cfg = {   // Fill in a config struct describing those buffers.
      .size           = sizeof(hal_usart_buffer_config_t),  // Tell the OS how big this config struct is.
      .rx_buffer      = rxBuf,                               // Point it at our receive buffer.
      .rx_buffer_size = sizeof(rxBuf),                       // ...and its size (4096).
      .tx_buffer      = txBuf,                               // Point it at our send buffer.
      .tx_buffer_size = sizeof(txBuf)};                      // ...and its size (256).
  return cfg;                    // Hand the config back to Device OS.
}

// PicLink::begin -> one-time setup for the link. "PicLink::" means "this function
// belongs to the PicLink class".
void PicLink::begin(unsigned long baud) {
  pinMode(PIC_WAKE_PIN, INPUT_PULLUP);     // D10 unused now (power-gating); hold it HIGH so it never floats.
  Serial1.begin(baud, SERIAL_8N1);         // Start the hardware serial port: 'baud' speed, 8 data bits, No parity, 1 stop bit.
}

// Power-gating model: if we are running, the PIC has powered us and is listening,
// so "ready" is always true. D10 is no longer a comms signal. Kept so callers compile.
bool PicLink::wakeIsHigh() { return true; }

// Empty the receive buffer by reading and discarding every waiting byte.
void PicLink::flushRx() { while (Serial1.available()) Serial1.read(); }    // Keep reading until none are left.

// Power-gating model: there is no WAKE line to wait on -- being powered already
// means the PIC is listening. Return true immediately. Kept so callers compile.
bool PicLink::waitWakeHigh(uint32_t timeoutMs) {
  (void)timeoutMs;                                // Argument unused now.
  return true;
}

// Read exactly one byte into *out, waiting up to 'timeoutMs'. Returns false on timeout.
bool PicLink::readByte(uint8_t *out, uint32_t timeoutMs) {
  uint32_t start = millis();                      // Record when we started waiting.
  while (!Serial1.available()) {                  // While no byte has arrived yet...
    if (millis() - start >= timeoutMs) return false;   // ...give up if we have waited too long.
  }
  *out = (uint8_t)Serial1.read();                 // A byte is here: read it and store it where 'out' points.
  return true;                                    // Success.
}

// CRC-16/MODBUS over func+len+data (marker excluded). Returns host-order value;
// transmitted big-endian as (crc>>8) then (crc&0xFF).
// A CRC is a math fingerprint of the bytes; if even one byte changes, the CRC changes.
uint16_t PicLink::crc16(const uint8_t *p, uint16_t n) {
  uint16_t crc = 0xFFFF;                          // Start value required by the MODBUS CRC standard.
  while (n--) {                                   // Loop over all n bytes (n-- counts down to 0).
    crc ^= *p++;                                  // XOR the current byte into the low part of crc; advance p.
    for (uint8_t i = 0; i < 8; i++)               // Process each of the 8 bits in that byte...
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
                                                  // If the lowest bit is 1: shift right and XOR the polynomial 0xA001.
                                                  // If it is 0: just shift right. (This is the standard CRC loop.)
  }
  return crc;                                     // Return the finished 16-bit fingerprint.
}

// Power-gating model: if we are running, the PIC is powered and listening, so
// there is nothing to wake. Never send 0xF0, never wait on D10 -- just succeed.
bool PicLink::ensureWake() {
  return true;   // Powered == comms allowed. sendFrame() therefore always proceeds.
}

// Build "AA 55 func len_hi len_lo data crc_hi crc_lo" and put it on the wire.
bool PicLink::sendFrame(uint8_t func, const uint8_t *data, uint16_t len) {
  if ((uint32_t)len + 7 > sizeof(_txbuf)) return false;   // host frames are tiny
                                                  //   Refuse if the payload + 7 overhead bytes won't fit our buffer.
  if (!ensureWake()) return false;                // Make sure the PIC is awake; bail out if it never woke.

  uint16_t i = 0;                                 // 'i' is our write position inside the transmit buffer.
  _txbuf[i++] = PIC_MARK0;                         // Byte 0: first marker 0xAA. (i++ stores then advances.)
  _txbuf[i++] = PIC_MARK1;                         // Byte 1: second marker 0x55.
  _txbuf[i++] = func;                              // Byte 2: the function code (what kind of message this is).
  _txbuf[i++] = (uint8_t)(len >> 8);               // Byte 3: high byte of the length (top 8 bits).
  _txbuf[i++] = (uint8_t)(len & 0xFF);             // Byte 4: low byte of the length (bottom 8 bits).
  for (uint16_t k = 0; k < len; k++) _txbuf[i++] = data[k];  // Bytes 5..: copy the payload in, byte by byte.

  uint16_t crc = crc16(&_txbuf[2], (uint16_t)(3 + len));   // func+len+data
                                                  //   Compute CRC over func(1)+len(2)+data(len) = the bytes from index 2 onward.
  _txbuf[i++] = (uint8_t)(crc >> 8);               // Append CRC high byte (big-endian: high byte first).
  _txbuf[i++] = (uint8_t)(crc & 0xFF);             // Append CRC low byte.

  flushRx();                                       // drop anything stale before we listen
                                                  //   Clear old/leftover received bytes so the upcoming reply is clean.
  Serial1.write(_txbuf, i);                        // Send all 'i' bytes of the finished frame at once.
  Serial1.flush();                                 // Wait until every byte has physically left the port.
  return true;                                     // Frame sent successfully.
}

// Resynchronise on AA 55, then read func + len + data + crc and verify CRC.
// `cap` is the data[] capacity. On success *outLen = payload length.
int PicLink::readFrame(uint8_t *func, uint8_t *data, uint16_t cap,
                       uint16_t *outLen, uint32_t timeoutMs) {
  uint32_t deadline = millis() + timeoutMs;       // Absolute time by which the whole frame must arrive.

  // Hunt for the AA 55 marker.
  uint8_t prev = 0, b = 0;                         // 'prev' = previous byte, 'b' = current byte.
  bool synced = false;                             // Becomes true once we find the AA 55 start marker.
  while ((int32_t)(deadline - millis()) > 0) {     // Keep scanning while there is still time left.
    if (!readByte(&b, 5)) continue;                // Try to read a byte (5 ms each); if none, loop again.
    if (prev == PIC_MARK0 && b == PIC_MARK1) { synced = true; break; }  // Found "AA 55" -> a frame begins.
    prev = b;                                      // Slide the window: this byte becomes "previous" for next time.
  }
  if (!synced) return PIC_ERR_TIMEOUT;             // Never found a start marker in time -> timeout error.

  uint32_t left = (int32_t)(deadline - millis()) > 0 ? deadline - millis() : 1;
                                                  // How much time remains for the rest of the frame (at least 1 ms).

  uint8_t hdr[3];                                  // func, len_hi, len_lo
                                                  //   Small array to hold the 3 header bytes after the marker.
  for (uint8_t k = 0; k < 3; k++)                  // Read those 3 header bytes...
    if (!readByte(&hdr[k], left)) return PIC_ERR_TIMEOUT;   // ...failing with timeout if any is missing.

  uint16_t len = ((uint16_t)hdr[1] << 8) | hdr[2]; // Rebuild the 16-bit length: high byte<<8 OR low byte.
  if (len > cap) return PIC_ERR_OVERFLOW;          // If the payload is bigger than our buffer, refuse it.

  for (uint16_t k = 0; k < len; k++)               // Read exactly 'len' payload bytes...
    if (!readByte(&data[k], left)) return PIC_ERR_TIMEOUT; // ...timeout if any byte is missing.

  uint8_t crcb[2];                                 // Buffer for the 2 incoming CRC bytes.
  for (uint8_t k = 0; k < 2; k++)                  // Read both CRC bytes...
    if (!readByte(&crcb[k], left)) return PIC_ERR_TIMEOUT; // ...timeout if missing.
  uint16_t rxCrc = ((uint16_t)crcb[0] << 8) | crcb[1];   // Rebuild the received CRC (big-endian: high then low).

  // CRC covers func+len+data. Recompute over a small contiguous buffer.
  uint16_t crc = 0xFFFF;                            // Start a fresh CRC at the MODBUS init value.
  uint8_t  head[3] = {hdr[0], hdr[1], hdr[2]};      // Copy the 3 header bytes into a small contiguous array.
  crc = crc16(head, 3);                            // Run the CRC over those 3 header bytes first.
  // continue rolling over data
  {                                                // (Inner block just to keep the helper variable 'c' local.)
    uint16_t c = crc;                              // Continue from the header's CRC value.
    for (uint16_t k = 0; k < len; k++) {           // Now fold every payload byte into the CRC...
      c ^= data[k];                                // XOR this data byte into the running CRC.
      for (uint8_t i = 0; i < 8; i++)              // Process its 8 bits (same standard CRC step as before)...
        c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
    }
    crc = c;                                       // Save the final computed CRC.
  }
  if (crc != rxCrc) return PIC_ERR_BAD_CRC;        // If our computed CRC differs from the one received -> corrupted.

  *func   = hdr[0];                                // Tell the caller which function code this frame carried.
  *outLen = len;                                   // Tell the caller how many payload bytes are in data[].
  return PIC_OK;                                   // Frame received and verified successfully.
}

// Send a REQ and wait for the matching RSP, retrying on timeout/bad CRC up to
// PHOTON_RETRY_COUNT (spec 5.4 / 7). RSP_NAK is reported as PIC_ERR_NAK and is
// NOT retried (the request was understood; the data was rejected).
int PicLink::transact(uint8_t reqFunc, const uint8_t *reqData, uint16_t reqLen,
                      uint8_t wantFunc, uint8_t *rspData, uint16_t rspCap,
                      uint16_t *rspLen, uint32_t timeoutMs) {
  int last = PIC_ERR_TIMEOUT;                      // Remember the most recent error to return if all tries fail.
  for (uint8_t attempt = 0; attempt <= PHOTON_RETRY_COUNT; attempt++) {  // Original try + up to N retries.
    if (!sendFrame(reqFunc, reqData, reqLen)) { last = PIC_ERR_NO_WAKE; continue; }
                                                  //   Could not send (PIC never woke) -> record error and retry.

    uint8_t  rf = 0;                               // Will hold the received reply's function code.
    uint16_t rl = 0;                               // Will hold the received reply's payload length.
    int r = readFrame(&rf, rspData, rspCap, &rl, timeoutMs);  // Wait for one reply frame.
    if (r != PIC_OK) { last = r; continue; }       // timeout/CRC/overflow -> retry
                                                  //   Bad/no reply -> remember why and try again.

    if (rf == RSP_NAK) {                           // The PIC explicitly rejected our request.
      _lastNak = (rl >= 1) ? rspData[0] : 0;       // Save the reason byte (if any) for lastNak().
      return PIC_ERR_NAK;                          // understood but rejected
                                                  //   Do NOT retry a NAK; report it to the caller.
    }
    if (rf != wantFunc) { last = PIC_ERR_WRONG_RSP; continue; }   // Reply was a valid frame but the wrong type -> retry.

    if (rspLen) *rspLen = rl;                       // If the caller wants the length, give it to them.
    return PIC_OK;                                  // We got exactly the reply we wanted -> success.
  }
  return last;                                      // All attempts failed -> return the last error seen.
}

// ---- REQ_DATA -> RSP_DATA (V051 extended header) ---------------------------
// Ask the PIC for its stored flow samples, then unpack them into out[].
// V051 payload = 18-byte header + COUNT*3 sample bytes:
//   u32 impulse_since_report | u32 captures_since_report | u32 impulse_of_span
//   | u16 overflow_count | u32 COUNT | then COUNT * 3-byte samples.
int PicLink::requestData(PicSample *out, uint16_t maxSamples, PicReportInfo *info) {
  static const uint16_t XHDR = 18;                 // fixed extended-header size
  static uint8_t rx[XHDR + PIC_MAX_SAMPLES * PIC_BYTES_PER_SAMPLE];  // header + up to 1000*3 sample bytes.
  uint16_t rl = 0;                                 // Will receive the actual reply length.
  int r = transact(REQ_DATA, nullptr, 0, RSP_DATA, // Send REQ_DATA (no payload) and expect RSP_DATA back.
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_DATA_MS);
  if (r != PIC_OK) return r;                       // If the transaction failed, pass the error code up.
  if (rl < XHDR) return PIC_ERR_BAD_FRAME;         // Must contain at least the 18-byte extended header.

  // --- decode the 18-byte extended header (all big-endian) ---
  uint32_t f_imp  = ((uint32_t)rx[0]  << 24) | ((uint32_t)rx[1]  << 16) | ((uint32_t)rx[2]  << 8) | rx[3];
  uint32_t f_cap  = ((uint32_t)rx[4]  << 24) | ((uint32_t)rx[5]  << 16) | ((uint32_t)rx[6]  << 8) | rx[7];
  uint32_t f_span = ((uint32_t)rx[8]  << 24) | ((uint32_t)rx[9]  << 16) | ((uint32_t)rx[10] << 8) | rx[11];
  uint16_t f_ovf  = ((uint16_t)rx[12] << 8)  | rx[13];
  uint32_t count  = ((uint32_t)rx[14] << 24) | ((uint32_t)rx[15] << 16) | ((uint32_t)rx[16] << 8) | rx[17];

  if (count > PIC_MAX_SAMPLES) return PIC_ERR_BAD_FRAME;   // Sanity: never more than 1000 samples.
  if ((uint32_t)XHDR + count * PIC_BYTES_PER_SAMPLE != rl) return PIC_ERR_BAD_FRAME;
                                                  //   Length must be exactly 18 + count*3, or the frame is wrong.
  if (count > maxSamples) return PIC_ERR_OVERFLOW; // The caller's out[] array is too small to hold them all.

  if (info) {                                      // Hand the header fields back to the caller.
    info->impulseSinceReport  = f_imp;
    info->capturesSinceReport = f_cap;
    info->impulseOfSpan       = f_span;
    info->overflowCount       = f_ovf;
    info->count               = count;
  }

  const uint8_t *p = &rx[XHDR];                    // 'p' points at the first sample byte (past the 18-byte header).
  for (uint32_t i = 0; i < count; i++, p += 3) {   // For each sample (3 bytes each), advance p by 3...
    uint32_t word = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];  // Combine 3 bytes into one 24-bit value.
    out[i].index  = (uint16_t)((word >> 14) & 0x03FF);   // high 10 bits  : top 10 bits = the time-slot index.
    out[i].pulses = (uint16_t)( word        & 0x3FFF);   // low 14 bits   : bottom 14 bits = the pulse count.
  }
  return (int)count;                               // Return how many samples we decoded (0 or more).
}

// ---- REQ_GET_PARAM -> RSP_PARAM --------------------------------------------
// Read the PIC's four leak parameters into 'out'. Returns true on success.
bool PicLink::getParams(PicParams &out) {
  uint8_t  rx[8];                                  // The reply is exactly 8 bytes (4 values * 2 bytes each).
  uint16_t rl = 0;                                 // Will receive the reply length.
  int r = transact(REQ_GET_PARAM, nullptr, 0, RSP_PARAM,   // Send REQ_GET_PARAM, expect RSP_PARAM.
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  if (r != PIC_OK || rl != 8) return false;        // Fail if the transaction failed or length wasn't 8.
  out.leak1_counts   = ((uint16_t)rx[0] << 8) | rx[1];   // Bytes 0-1 -> leak1 counts (big-endian).
  out.leak1_window_s = ((uint16_t)rx[2] << 8) | rx[3];   // Bytes 2-3 -> leak1 window seconds.
  out.leak2_counts   = ((uint16_t)rx[4] << 8) | rx[5];   // Bytes 4-5 -> leak2 counts.
  out.leak2_window_s = ((uint16_t)rx[6] << 8) | rx[7];   // Bytes 6-7 -> leak2 window seconds.
  return true;                                     // Successfully read all four values.
}

// ---- REQ_SET_PARAM -> RSP_ACK/NAK ------------------------------------------
// Write four leak parameters from 'in' to the PIC. Returns true only if ACKed.
bool PicLink::setParams(const PicParams &in) {
  uint8_t tx[8];                                   // We need to send 8 bytes (4 values * 2 bytes).
  tx[0] = (uint8_t)(in.leak1_counts   >> 8); tx[1] = (uint8_t)(in.leak1_counts);   // leak1 counts -> high, low byte.
  tx[2] = (uint8_t)(in.leak1_window_s >> 8); tx[3] = (uint8_t)(in.leak1_window_s); // leak1 window -> high, low.
  tx[4] = (uint8_t)(in.leak2_counts   >> 8); tx[5] = (uint8_t)(in.leak2_counts);   // leak2 counts -> high, low.
  tx[6] = (uint8_t)(in.leak2_window_s >> 8); tx[7] = (uint8_t)(in.leak2_window_s); // leak2 window -> high, low.

  uint8_t  rx[2];                                  // The ACK/NAK reply is small; 2 bytes is enough.
  uint16_t rl = 0;                                 // Will receive the reply length.
  int r = transact(REQ_SET_PARAM, tx, sizeof(tx), RSP_ACK,   // Send the 8-byte payload, expect an ACK.
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  return (r == PIC_OK);                            // ACK; NAK/timeout -> false
                                                  //   true only if the PIC acknowledged the write.
}

// ---- REQ_SET_SCHEDULE -> RSP_ACK/NAK ---------------------------------------
// Re-anchor the PIC's report-due countdown to fire after exactly
// 'remainingCaptures' more captures. Returns true only if ACKed.
bool PicLink::setSchedule(uint16_t remainingCaptures) {
  uint8_t tx[2];
  tx[0] = (uint8_t)(remainingCaptures >> 8); tx[1] = (uint8_t)(remainingCaptures);

  uint8_t  rx[2];
  uint16_t rl = 0;
  int r = transact(REQ_SET_SCHEDULE, tx, sizeof(tx), RSP_ACK,
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  return (r == PIC_OK);
}

// ---- REQ_GET_VALVE -> RSP_VALVE --------------------------------------------
// Read the valve status into 'out'. Returns true on success.
bool PicLink::getValve(PicValve &out) {
  uint8_t  rx[5];                                  // The valve reply is 5 bytes.
  uint16_t rl = 0;                                 // Will receive the reply length.
  int r = transact(REQ_GET_VALVE, nullptr, 0, RSP_VALVE,   // Send REQ_GET_VALVE, expect RSP_VALVE.
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  if (r != PIC_OK || rl != 5) return false;        // Fail if transaction failed or length wasn't 5.
  out.pwr_pin         = rx[0];                      // Byte 0: valve power pin level (0/1).
  out.ctrl_pin        = rx[1];                      // Byte 1: valve control/direction pin level (0/1).
  out.motion          = rx[2];                      // Byte 2: valve motion state (0..6).
  out.lock_flags      = rx[3];                      // Byte 3: which locks are active (bit0=temp, bit1=perm).
  out.leakSinceReport = rx[4];                      // Byte 4: LEAK1/LEAK2 tripped since last report (PIC clears after sending).
  return true;                                     // Successfully read the valve status.
}

// ---- REQ_VALVE_UNLOCK -> RSP_ACK/NAK ---------------------------------------
// Clear one or both valve locks. 'flags' picks which. Returns true only if ACKed.
bool PicLink::unlockValve(uint8_t flags) {
  uint8_t  tx[1] = { (uint8_t)(flags & VALVE_LOCK_BOTH) };   // One payload byte; mask off any stray bits (keep only temp/perm).
  uint8_t  rx[2];                                  // Small reply buffer for ACK/NAK.
  uint16_t rl = 0;                                 // Will receive reply length.
  int r = transact(REQ_VALVE_UNLOCK, tx, sizeof(tx), RSP_ACK,   // Send the unlock request, expect an ACK.
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  return (r == PIC_OK);                            // true only if the PIC acknowledged the unlock.
}

// ---- PKT_SYS_RESET (no reply) ----------------------------------------------
// Tell the PIC to reset itself. We do not expect (or wait for) any answer.
void PicLink::sysReset() {
  sendFrame(PKT_SYS_RESET, nullptr, 0);            // fire and forget; PIC resets
                                                  //   Send the reset frame with no payload and move on.
}

// ---- PKT_PHOTON_OFF_REQ (func 0x07, no reply) ------------------------------
// "I'm done -- cut my power." One-way, fire-and-forget (like sysReset). The PIC
// answers by driving RC4 HIGH (P-MOS off), removing the Photon's supply.
// Verified frames (CRC-16/MODBUS over func+len+data, sent big-endian):
//   reason 0 (DONE)       : AA 55 07 00 01 00 C0 00
//   reason 1 (CLOUD_FAIL) : AA 55 07 00 01 01 00 C1
bool PicLink::sendPhotonOff(uint8_t reason) {
  return sendFrame(PKT_PHOTON_OFF_REQ, &reason, 1);   // AA 55 07 00 01 <reason> crc_hi crc_lo
}

// ---- PKT_KEEPALIVE (func 0x0A, no reply) -----------------------------------
// "I'm alive, still connecting -- keep my power." One-way, fire-and-forget (like
// sysReset). Any CRC-valid frame resets the PIC's ACTIVE idle timer, so this
// zero-payload packet keeps a live-but-connecting Photon from being powered off
// before it can reach the cloud (or exhaust its own 80 s CLOUD_FAIL budget).
//   frame: AA 55 0A 00 00 <crc_hi> <crc_lo>
bool PicLink::sendKeepalive() {
  return sendFrame(PKT_KEEPALIVE, nullptr, 0);
}

// ---- REQ_POWER_STATE -> RSP_POWER_STATE ------------------------------------
// Ask whether the PIC is in its initial cold-boot power-hold. On success writes the
// single state byte to *out (POWER_STATE_INITIAL / POWER_STATE_NORMAL) and returns
// PIC_OK; otherwise returns a PIC_ERR_* code and leaves *out unchanged.
// Verified frames (CRC-16/MODBUS over func+len+data, sent big-endian):
//   REQ (no payload)   : AA 55 08 00 00 02 F0
//   RSP 0 (INITIAL)    : AA 55 88 00 01 00 14 2A
//   RSP 1 (NORMAL)     : AA 55 88 00 01 01 D4 EB
int PicLink::getPowerState(uint8_t *out) {
  uint8_t  rx[1];                                  // The reply carries a single state byte.
  uint16_t rl = 0;                                 // Will receive the reply length.
  int r = transact(REQ_POWER_STATE, nullptr, 0, RSP_POWER_STATE,   // Send REQ_POWER_STATE (no payload), expect RSP_POWER_STATE.
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  if (r != PIC_OK) return r;                       // Pass any transaction error up unchanged.
  if (rl != 1) return PIC_ERR_BAD_FRAME;           // The reply must be exactly one byte.
  if (out) *out = rx[0];                           // Hand back the state byte (0 INITIAL / 1 NORMAL).
  return PIC_OK;                                   // Success.
}

// Ask the PIC for our timing + debug config (RSP_PHOTON_CFG, 0x89, 14 bytes).
// transact() already retries on timeout/bad-CRC, so one call is robust; the
// caller adds a few more attempts for the very first exchange after power-up.
int PicLink::getPhotonConfig(PicPhotonCfg *out) {
  uint8_t  rx[14];                                 // 14-byte config block.
  uint16_t rl = 0;
  int r = transact(REQ_PHOTON_CFG, nullptr, 0, RSP_PHOTON_CFG,
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  if (r != PIC_OK) return r;                       // timeout/CRC/etc -> caller retries
  if (rl != 14) return PIC_ERR_BAD_FRAME;          // must be exactly 14 bytes
  if (out) {
    out->provided          = (rx[0] != 0);
    out->version           = rx[1];
    out->captureIntervalMs = ((uint32_t)rx[2] << 24) | ((uint32_t)rx[3] << 16) |
                             ((uint32_t)rx[4] <<  8) |  (uint32_t)rx[5];
    out->samplesPerReport  = (uint16_t)(((uint16_t)rx[6] << 8) | rx[7]);
    out->reportIntervalHr  = rx[8];
    out->fastBench         = (rx[9] != 0);
    out->debugDataseries   = (rx[10] != 0);
    out->missedFillMode    = rx[11];
    out->serialDelayMs     = (uint16_t)(((uint16_t)rx[12] << 8) | rx[13]);
  }
  return PIC_OK;
}