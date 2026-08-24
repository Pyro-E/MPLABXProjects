/*
 * pic_link.cpp  -  Framed packet link implementation (Photon V057 / PIC V055_DB_FINAL).
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
// it:  7 (frame overhead) + 23 (V057 header) + 1000*2 (16-bit samples) =
// 2030 bytes. The 4096 B buffer below leaves margin and matches the spec ask.
// The 1000-sample ceiling is the PIC's own cap: APP_FLOW_SLOTS (1024) minus
// APP_FLOW_RING_MARGIN (24), so PIC_MAX_SAMPLES = 1000 can never be exceeded.
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
// Human-readable name for a func code (for the [PKT] log).
static const char *picFuncName(uint8_t f) {
  switch (f) {
    case REQ_DATA:             return "REQ_DATA";
    case REQ_GET_PARAM:        return "REQ_GET_PARAM";
    case REQ_SET_LEAK:         return "REQ_SET_LEAK";
    case REQ_SET_GRID:         return "REQ_SET_GRID";
    case REQ_GET_VALVE:        return "REQ_GET_VALVE";
    case REQ_VALVE_UNLOCK:     return "REQ_VALVE_UNLOCK";
    case PKT_SYS_RESET:        return "SYS_RESET";
    case PKT_PHOTON_OFF_REQ:   return "PHOTON_OFF_REQ";
    case REQ_POWER_STATE:      return "REQ_POWER_STATE";
    case REQ_PHOTON_CFG:       return "REQ_PHOTON_CFG";
    case PKT_KEEPALIVE:        return "KEEPALIVE";
    case PKT_DATA_RECEIVED:    return "DATA_RECEIVED";
    case PKT_TIME_SYNC:        return "TIME_SYNC";
    case PKT_TIME_RECEIVED:    return "TIME_RECEIVED";
    case REQ_GET_LOCO:         return "REQ_GET_LOCO";
    case RSP_LOCO:             return "RSP_LOCO";
    case RSP_DATA:             return "RSP_DATA";
    case RSP_PARAM:            return "RSP_PARAM";
    case RSP_VALVE:            return "RSP_VALVE";
    case RSP_POWER_STATE:      return "RSP_POWER_STATE";
    case RSP_PHOTON_CFG:       return "RSP_PHOTON_CFG";
    case RSP_ACK:              return "RSP_ACK";
    case RSP_NAK:              return "RSP_NAK";
    default:                   return "UNKNOWN";
  }
}

// Log one framed packet as: [PKT] <dir> NAME(0x..) len=.. data=<hex>.
// Goes to BOTH the USB serial and the debug UART (Log handler broadcast).
// The hex dump is capped so a big RSP_DATA does not flood the console.
static void pktLog(const char *dir, uint8_t func, const uint8_t *data, uint16_t len) {
  char hex[3 * 24 + 12];
  size_t o = 0;
  uint16_t show = (len > 24) ? 24 : len;
  for (uint16_t k = 0; k < show && o + 4 < sizeof(hex); k++) {
    o += snprintf(hex + o, sizeof(hex) - o, "%02X ", data ? data[k] : 0);
  }
  if (len > show && o + 12 < sizeof(hex)) {
    snprintf(hex + o, sizeof(hex) - o, "...(+%u)", (unsigned)(len - show));
  } else if (o > 0) {
    hex[o - 1] = '\0';   // trim trailing space
  } else {
    hex[0] = '\0';
  }
  Log.info("[PKT] %s %s(0x%02X) len=%u data=[%s]",
           dir, picFuncName(func), func, (unsigned)len, hex);
}

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
  pktLog("TX", func, data, len);                   // [PKT] log the outgoing frame (USB + debug UART)
  Serial1.write(_txbuf, i);                        // Send all 'i' bytes of the finished frame at once.
  Serial1.flush();                                 // Wait until every byte has physically left the port.
  _lastTxMs = millis();                            // Feeds lastTxMillis() -> keepalive pump during publishes.
  return true;                                     // Frame sent successfully.
}

// Resynchronise on AA 55, then read func + len + data + crc and verify CRC.
// `cap` is the data[] capacity. On success *outLen = payload length.
int PicLink::readFrame(uint8_t *func, uint8_t *data, uint16_t cap,
                       uint16_t *outLen, uint32_t timeoutMs) {
  uint32_t deadline = millis() + timeoutMs;       // Absolute time by which the whole frame must arrive.
  _lastRxBytes = 0; _lastRxFunc = 0; _lastRxLen = 0;   // Appendix H.7.3: reset per-frame diagnostics.

  // Hunt for the AA 55 marker.
  uint8_t prev = 0, b = 0;                         // 'prev' = previous byte, 'b' = current byte.
  bool synced = false;                             // Becomes true once we find the AA 55 start marker.
  while ((int32_t)(deadline - millis()) > 0) {     // Keep scanning while there is still time left.
    if (!readByte(&b, 5)) continue;                // Try to read a byte (5 ms each); if none, loop again.
    if (prev == PIC_MARK0 && b == PIC_MARK1) { synced = true; break; }  // Found "AA 55" -> a frame begins.
    prev = b;                                      // Slide the window: this byte becomes "previous" for next time.
  }
  if (!synced) return PIC_ERR_TIMEOUT;             // Never found a start marker in time -> timeout error.

  // 'timeoutMs' is a budget for the WHOLE frame, so every remaining read must
  // measure its wait against the one shared deadline. Computing the remaining
  // time once and reusing it as a PER-BYTE timeout would let a peer that
  // trickles bytes stretch a 4 s transaction to (len + 5) * 4 s -- over two
  // hours on a full 2023-byte RSP_DATA -- while never technically timing out.
  // remaining() re-reads the clock on every call so that cannot happen.
  auto remaining = [deadline]() -> uint32_t {
    int32_t r = (int32_t)(deadline - millis());    // Signed: survives millis() wraparound.
    return (r > 0) ? (uint32_t)r : 1u;             // Always allow at least 1 ms so a ready byte is still taken.
  };

  uint8_t hdr[3];                                  // func, len_hi, len_lo
                                                  //   Small array to hold the 3 header bytes after the marker.
  for (uint8_t k = 0; k < 3; k++) {                // Read those 3 header bytes...
    if (!readByte(&hdr[k], remaining())) return PIC_ERR_TIMEOUT;   // ...failing with timeout if any is missing.
    _lastRxBytes++;                                // Appendix H.7.3: count header bytes actually received.
  }

  uint16_t len = ((uint16_t)hdr[1] << 8) | hdr[2]; // Rebuild the 16-bit length: high byte<<8 OR low byte.
  _lastRxFunc = hdr[0]; _lastRxLen = len;          // Appendix H.7.3: remember what we saw for the diagnostic.
  if (len > cap) return PIC_ERR_OVERFLOW;          // If the payload is bigger than our buffer, refuse it.

  for (uint16_t k = 0; k < len; k++) {             // Read exactly 'len' payload bytes...
    if (!readByte(&data[k], remaining())) return PIC_ERR_TIMEOUT; // ...timeout if any byte is missing.
    _lastRxBytes++;                                // Appendix H.7.3: count payload bytes actually received.
  }

  uint8_t crcb[2];                                 // Buffer for the 2 incoming CRC bytes.
  for (uint8_t k = 0; k < 2; k++) {                // Read both CRC bytes...
    if (!readByte(&crcb[k], remaining())) return PIC_ERR_TIMEOUT; // ...timeout if missing.
    _lastRxBytes++;                                // Appendix H.7.3: count CRC bytes actually received.
  }
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
  pktLog("RX", *func, data, len);                  // [PKT] log the verified incoming frame
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
    // V064 P-10: a RETRY must start from a KNOWN-EMPTY receive path.
    //
    // The previous attempt failed, which usually means its reply was late,
    // partial, or corrupt - and a partial reply is still arriving. sendFrame()
    // does flushRx() at the moment it transmits, but that only discards what has
    // already landed; the tail still on the wire arrives afterwards and becomes
    // the leading bytes of the next reply. Requests are 7 bytes and RSP_DATA is
    // up to 2023, so the asymmetry is large: the leftovers of one bad reply can
    // outlast several requests. readFrame() does resynchronise on AA 55, but
    // sample data can contain a false AA 55, and once the parser locks onto one
    // it reads a bogus length and loses that attempt too - which is how a single
    // slip turns into a run of failures that only ends when the bytes happen to
    // line up again.
    //
    // So: wait for the line to fall quiet, then drop whatever arrived. Bounded,
    // and only on retries - the first attempt of a transaction pays nothing.
    if (attempt > 0u) {
      uint32_t startMs = millis();
      uint32_t lastByteMs = startMs;
      while ((millis() - startMs) < PHOTON_RX_QUIESCE_MAX_MS) {
        if (Serial1.available()) {
          Serial1.read();                          // consume the straggler
          lastByteMs = millis();
        } else if ((millis() - lastByteMs) >= PHOTON_RX_IDLE_MS) {
          break;                                   // line has been quiet long enough
        }
      }
      flushRx();                                   // belt and braces: start from empty
    }

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

// ---- REQ_DATA -> RSP_DATA (23-byte header) ---------------------------------
// Ask the PIC for its stored flow samples, then unpack them into out[].
// Payload = 23-byte header + sample_count * 2 sample bytes, all big-endian:
//   u32 start_time | u8 start_time_valid | u32 sample_count | u32 total_captures
//   | u32 total_impulses | u16 overflow_ffff | u32 sample_interval_ms
//   | then sample_count * u16 pulses
// There is no per-sample index on the wire; a sample's position in the batch IS
// its position in time (oldest first).
int PicLink::requestData(PicSample *out, uint16_t maxSamples, PicReportInfo *info) {
  static const uint16_t XHDR = PIC_RSP_DATA_HDR_BYTES;   // 23
  static uint8_t rx[PIC_RSP_DATA_HDR_BYTES + PIC_MAX_SAMPLES * PIC_BYTES_PER_SAMPLE];
  uint16_t rl = 0;                                 // Will receive the actual reply length.
  int r = transact(REQ_DATA, nullptr, 0, RSP_DATA, // Send REQ_DATA (no payload) and expect RSP_DATA back.
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_DATA_MS);
  if (r != PIC_OK) return r;                       // If the transaction failed, pass the error code up.
  if (rl < XHDR) return PIC_ERR_BAD_FRAME;         // Must contain at least the 23-byte header.

  // --- decode the 23-byte header (all big-endian) ---
  uint32_t f_start = ((uint32_t)rx[0]  << 24) | ((uint32_t)rx[1]  << 16) | ((uint32_t)rx[2]  << 8) | rx[3];
  uint8_t  f_svld  = rx[4];
  uint32_t f_count = ((uint32_t)rx[5]  << 24) | ((uint32_t)rx[6]  << 16) | ((uint32_t)rx[7]  << 8) | rx[8];
  uint32_t f_caps  = ((uint32_t)rx[9]  << 24) | ((uint32_t)rx[10] << 16) | ((uint32_t)rx[11] << 8) | rx[12];
  uint32_t f_imp   = ((uint32_t)rx[13] << 24) | ((uint32_t)rx[14] << 16) | ((uint32_t)rx[15] << 8) | rx[16];
  uint16_t f_ovf   = ((uint16_t)rx[17] << 8)  | rx[18];
  uint32_t f_intms = ((uint32_t)rx[19] << 24) | ((uint32_t)rx[20] << 16) | ((uint32_t)rx[21] << 8) | rx[22];
#if PHOTON_BATCH_SEQ_ENABLE
  // V064 P-5: field 8, one byte, present only on the 24-byte header.
  uint8_t  f_bseq  = rx[PIC_RSP_DATA_BATCHSEQ_OFF];
#endif

  if (f_count > PIC_MAX_SAMPLES) return PIC_ERR_BAD_FRAME;   // Sanity: never more than 1000 samples.
  if ((uint32_t)XHDR + f_count * PIC_BYTES_PER_SAMPLE != rl) return PIC_ERR_BAD_FRAME;
                                                  //   Length must be exactly XHDR + count*2, or the frame is wrong.
                                                  //   V064 P-5: XHDR is 23 or 24 depending on
                                                  //   PHOTON_BATCH_SEQ_ENABLE. A PIC built with the
                                                  //   opposite setting fails HERE, deliberately - a
                                                  //   version mismatch on a metering link must stop,
                                                  //   not be papered over by guessing the layout.

  if (f_count > maxSamples) return PIC_ERR_OVERFLOW; // The caller's out[] array is too small to hold them all.

  if (info) {                                      // Hand the header fields back to the caller.
    info->startTime        = f_start;
    info->startTimeValid   = f_svld;
    info->sampleCount      = f_count;
    info->totalCaptures    = f_caps;
    info->totalImpulses    = f_imp;
    info->overflowFfff     = f_ovf;
    info->sampleIntervalMs = f_intms;
#if PHOTON_BATCH_SEQ_ENABLE
    info->batchSeq         = f_bseq;
    info->batchSeqValid    = 1u;
#else
    info->batchSeq         = 0u;
    info->batchSeqValid    = 0u;   // no identifier on the wire -> caller must not dedupe
#endif
  }

  const uint8_t *p = &rx[XHDR];                    // 'p' points at the first sample byte (past the header).
  for (uint32_t i = 0; i < f_count; i++, p += 2) { // Each sample = 2 bytes, big-endian, pulses only.
    out[i].pulses = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);  // 16-bit pulse count (MSB first).
    out[i].index  = 0;                             // no per-sample index anymore; position in batch = order.
  }
  return (int)f_count;                             // Return how many samples we decoded (0 or more).
}

// ---- REQ_GET_PARAM -> RSP_PARAM (16 B, or 8 B on an older PIC) -------------
// Read the PIC's four leak parameters, and - on the current firmware - the
// report grid (anchor + interval) that follows them.
bool PicLink::getParams(PicParams &out, PicGridParams *grid, bool *gridValid) {
  uint8_t  rx[PIC_RSP_PARAM_BYTES_V2];             // 16 bytes on the current PIC.
  uint16_t rl = 0;                                 // Will receive the reply length.
  if (gridValid) *gridValid = false;               // Assume "no grid" until we see 16 bytes.
  int r = transact(REQ_GET_PARAM, nullptr, 0, RSP_PARAM,   // Send REQ_GET_PARAM, expect RSP_PARAM.
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  if (r != PIC_OK) return false;                   // Transaction failed.
  if (rl != PIC_RSP_PARAM_BYTES_V2 && rl != PIC_RSP_PARAM_BYTES_V1) return false;

  out.leak1_counts   = ((uint16_t)rx[0] << 8) | rx[1];   // Bytes 0-1 -> leak1 counts (big-endian).
  out.leak1_window_s = ((uint16_t)rx[2] << 8) | rx[3];   // Bytes 2-3 -> leak1 window seconds.
  out.leak2_counts   = ((uint16_t)rx[4] << 8) | rx[5];   // Bytes 4-5 -> leak2 counts.
  out.leak2_window_s = ((uint16_t)rx[6] << 8) | rx[7];   // Bytes 6-7 -> leak2 window seconds.

  if (rl == PIC_RSP_PARAM_BYTES_V2) {              // Current PIC: the grid follows the leak values.
    uint32_t anchor = ((uint32_t)rx[8]  << 24) | ((uint32_t)rx[9]  << 16) |
                      ((uint32_t)rx[10] <<  8) |  (uint32_t)rx[11];
    uint32_t intvl  = ((uint32_t)rx[12] << 24) | ((uint32_t)rx[13] << 16) |
                      ((uint32_t)rx[14] <<  8) |  (uint32_t)rx[15];
    if (grid) { grid->anchorSec = anchor; grid->intervalSec = intvl; }
    if (gridValid) *gridValid = true;
  }
  return true;                                     // Successfully read the parameters.
}

// ---- REQ_SET_LEAK -> RSP_ACK/NAK (len 8 FIXED) -----------------------------
// Write ONLY the 4 leak parameters. Byte-for-byte the old 8-byte SET_PARAM.
// Returns true only if the PIC ACKed. A wrong length is NAKed, not dropped.
bool PicLink::setLeak(const PicParams &in) {
  uint8_t  tx[8];                                  // exactly 4x u16, big-endian
  uint16_t n = 0;
  tx[n++] = (uint8_t)(in.leak1_counts   >> 8); tx[n++] = (uint8_t)(in.leak1_counts);
  tx[n++] = (uint8_t)(in.leak1_window_s >> 8); tx[n++] = (uint8_t)(in.leak1_window_s);
  tx[n++] = (uint8_t)(in.leak2_counts   >> 8); tx[n++] = (uint8_t)(in.leak2_counts);
  tx[n++] = (uint8_t)(in.leak2_window_s >> 8); tx[n++] = (uint8_t)(in.leak2_window_s);

  uint8_t  rx[2];                                  // ACK echoes the func; NAK carries a reason.
  uint16_t rl = 0;
  int r = transact(REQ_SET_LEAK, tx, n, RSP_ACK,
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  return (r == PIC_OK);                            // true only if the PIC acknowledged the write.
}

// ---- REQ_SET_GRID -> RSP_ACK/NAK (len 8 FIXED) -----------------------------
// Write ONLY the report grid: anchor (u32 BE) then interval (u32 BE).
// Returns true only if the PIC ACKed. An ACK proves the frame PARSED, not that
// the grid was APPLIED (a PIC_USE_OWN_TIMING build ACKs but keeps its own grid);
// the caller confirms application by reading the grid back with getParams().
bool PicLink::setGrid(const PicGridParams &grid) {
  uint8_t  tx[8];                                  // anchor u32 + interval u32, big-endian
  uint16_t n = 0;
  tx[n++] = (uint8_t)((grid.anchorSec   >> 24) & 0xFFu);
  tx[n++] = (uint8_t)((grid.anchorSec   >> 16) & 0xFFu);
  tx[n++] = (uint8_t)((grid.anchorSec   >>  8) & 0xFFu);
  tx[n++] = (uint8_t)( grid.anchorSec          & 0xFFu);
  tx[n++] = (uint8_t)((grid.intervalSec >> 24) & 0xFFu);
  tx[n++] = (uint8_t)((grid.intervalSec >> 16) & 0xFFu);
  tx[n++] = (uint8_t)((grid.intervalSec >>  8) & 0xFFu);
  tx[n++] = (uint8_t)( grid.intervalSec        & 0xFFu);

  uint8_t  rx[2];
  uint16_t rl = 0;
  int r = transact(REQ_SET_GRID, tx, n, RSP_ACK,
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  return (r == PIC_OK);                            // true only if the PIC acknowledged the write.
}

// ---- REQ_GET_LOCO -> RSP_LOCO (10 B) ---------------------------------------
// Read the PIC's oscillator self-calibration state. Purely observational: it
// lets the bench watch applied_k converge without any log path on the PIC.
bool PicLink::getLocoStatus(PicLocoStatus &out) {
  uint8_t  rx[PIC_RSP_LOCO_BYTES];
  uint16_t rl = 0;
  int r = transact(REQ_GET_LOCO, nullptr, 0, RSP_LOCO,
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  if (r != PIC_OK || rl != PIC_RSP_LOCO_BYTES) return false;
  out.hfPrecision     = ((uint16_t)rx[0] << 8) | rx[1];
  out.hfCalibrated    = rx[2];
  out.hfInRange       = rx[3];
  out.cloudCalibrated = rx[4];
  out.cloudPrecision  = ((uint16_t)rx[5] << 8) | rx[6];
  out.cloudInRange    = rx[7];
  out.appliedK        = ((uint16_t)rx[8] << 8) | rx[9];
  return true;
}

// ---- REQ_GET_VALVE -> RSP_VALVE --------------------------------------------
// Read the valve status into 'out'. Returns true on success.
bool PicLink::getValve(PicValve &out) {
  uint8_t  rx[8];                                  // The valve reply is 8 bytes.
  uint16_t rl = 0;                                 // Will receive the reply length.
  int r = transact(REQ_GET_VALVE, nullptr, 0, RSP_VALVE,   // Send REQ_GET_VALVE, expect RSP_VALVE.
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  if (r != PIC_OK || rl != 8) return false;        // Fail if transaction failed or length wasn't 8.
  out.pwr_pin         = rx[0];                      // Byte 0: valve power pin level (0/1).
  out.ctrl_pin        = rx[1];                      // Byte 1: valve control/direction pin level (0/1).
  out.motion          = rx[2];                      // Byte 2: valve motion state (0..6).
  out.lock_flags      = rx[3];                      // Byte 3: which locks are active (bit0=temp, bit1=perm).
  out.temp_lock_count = ((uint32_t)rx[4] << 24) | ((uint32_t)rx[5] << 16) |   // Bytes 4-7: rebuild the 32-bit
                        ((uint32_t)rx[6] << 8)  |  (uint32_t)rx[7];           // cumulative temp-lock counter.
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

// ---- PKT_DATA_RECEIVED (func 0x0B, no reply) -------------------------------
// "The last RSP_DATA batch is safe (CRC ok + stored)." The PIC commits: it
// advances its FIFO read pointer + report marks (consume-on-ACK). One-way.
//   AA 55 0B 00 00 <crc_hi> <crc_lo>
bool PicLink::sendDataReceived(uint8_t batchSeq, bool seqValid) {
#if PHOTON_BATCH_SEQ_ENABLE
  // V064 P-5: echo the batch_seq we are acknowledging, unmodified. The PIC uses
  // it to release exactly that batch. seqValid should always be true here (the
  // 24-byte header always carries one); the len-0 fallback exists only so a
  // caller that genuinely has no seq cannot silently send a garbage byte.
  if (seqValid) {
    uint8_t d[1] = { batchSeq };
    return sendFrame(PKT_DATA_RECEIVED, d, 1);
  }
  return sendFrame(PKT_DATA_RECEIVED, nullptr, 0);
#else
  (void)batchSeq; (void)seqValid;                 // 23-byte header: no identifier exists.
  return sendFrame(PKT_DATA_RECEIVED, nullptr, 0);
#endif
}

// ---- PKT_TIME_SYNC (func 0x0C) -> PKT_TIME_RECEIVED (0x0D) ------------------
// Reliable: sends the cloud time and waits for the PIC's dedicated 0x0D ack,
// retransmitting on timeout/CRC error (via transact) so the PIC gets the time
// even over a lossy link. Returns true only when the PIC acknowledged.
//   AA 55 0C 00 05 <valid> <t3 t2 t1 t0> <crc_hi> <crc_lo>
bool PicLink::sendTimeSync(bool timeValid, uint32_t epochNow) {
  uint8_t d[5];
  d[0] = timeValid ? 1u : 0u;
  d[1] = (uint8_t)((epochNow >> 24) & 0xFFu);
  d[2] = (uint8_t)((epochNow >> 16) & 0xFFu);
  d[3] = (uint8_t)((epochNow >>  8) & 0xFFu);
  d[4] = (uint8_t)( epochNow        & 0xFFu);
  uint8_t  rsp[4];                     // small buffer (TIME_RECEIVED has len 0; guards NAK path)
  uint16_t rl = 0;
  int r = transact(PKT_TIME_SYNC, d, 5, PKT_TIME_RECEIVED,
                   rsp, sizeof(rsp), &rl, PHOTON_TIMEOUT_READ_MS);
  return (r == PIC_OK);
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

// Ask the PIC for our timing + debug config (RSP_PHOTON_CFG, 0x89, 13 bytes).
// transact() already retries on timeout/bad-CRC, so one call is robust; the
// caller adds a few more attempts for the very first exchange after power-up.
int PicLink::getPhotonConfig(PicPhotonCfg *out) {
  uint8_t  rx[13];                                 // 13-byte config block.
  uint16_t rl = 0;
  int r = transact(REQ_PHOTON_CFG, nullptr, 0, RSP_PHOTON_CFG,
                   rx, sizeof(rx), &rl, PHOTON_TIMEOUT_READ_MS);
  if (r != PIC_OK) return r;                       // timeout/CRC/etc -> caller retries
  if (rl != 13) return PIC_ERR_BAD_FRAME;          // must be exactly 13 bytes
  if (out) {
    out->provided          = (rx[0] != 0);
    out->version           = rx[1];
    out->captureIntervalMs = ((uint32_t)rx[2] << 24) | ((uint32_t)rx[3] << 16) |
                             ((uint32_t)rx[4] <<  8) |  (uint32_t)rx[5];
    out->samplesPerReport  = (uint16_t)(((uint16_t)rx[6] << 8) | rx[7]);
    out->fastBench         = (rx[8] != 0);
    out->debugDataseries   = (rx[9] != 0);
    out->missedFillMode    = rx[10];
    out->serialDelayMs     = (uint16_t)(((uint16_t)rx[11] << 8) | rx[12]);
  }
  return PIC_OK;
}