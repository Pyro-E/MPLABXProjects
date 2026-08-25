/*
 * piclink_test.cpp - host-side byte-level verification of pic_link.cpp.
 *
 * Compiles the REAL pic_link.cpp against a loopback Stream stub and checks
 * that what goes on the wire, and what comes off it, matches the PIC V055
 * packet spec byte for byte. This exercises the four items that could not be
 * checked by inspection alone:
 *   1. RSP_DATA 23-byte header field order and endianness
 *   2. CRC-16/MODBUS framing on both TX and RX paths
 *   3. REQ_GET_LOCO (0x0E) request bytes and RSP_LOCO decode
 *   4. REQ_SET_LEAK (0x03, len 8) and REQ_SET_GRID (0x0F, len 8) byte layout
 *      (Appendix E: the old length-discriminated SET_PARAM is split in two)
 */

#include "pic_link.h"
#include <vector>
#include <cstdio>
#include <cstring>

static int g_fail = 0;

static void check(const char *what, bool ok) {
  printf("  %-58s %s\n", what, ok ? "PASS" : "FAIL");
  if (!ok) g_fail++;
}

static uint16_t crcModbus(const uint8_t *p, size_t n) {
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= *p++;
    for (int i = 0; i < 8; i++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
  }
  return c;
}

// Build a full frame the way the PIC would, and queue it as inbound bytes.
static void queueFrame(uint8_t func, const std::vector<uint8_t> &data) {
  std::vector<uint8_t> body;
  body.push_back(func);
  body.push_back((uint8_t)(data.size() >> 8));
  body.push_back((uint8_t)(data.size() & 0xFF));
  for (uint8_t b : data) body.push_back(b);
  uint16_t crc = crcModbus(body.data(), body.size());

  Serial1.pending.push_back(0xAA);
  Serial1.pending.push_back(0x55);
  for (uint8_t b : body) Serial1.pending.push_back(b);
  Serial1.pending.push_back((uint8_t)(crc >> 8));
  Serial1.pending.push_back((uint8_t)(crc & 0xFF));
  // One frame per flush, so a test can stage several transactions back to back.
  Serial1.releaseChunks.push_back(body.size() + 4);
}

static void be32(std::vector<uint8_t> &v, uint32_t x) {
  v.push_back((uint8_t)(x >> 24)); v.push_back((uint8_t)(x >> 16));
  v.push_back((uint8_t)(x >> 8));  v.push_back((uint8_t)x);
}
static void be16(std::vector<uint8_t> &v, uint16_t x) {
  v.push_back((uint8_t)(x >> 8)); v.push_back((uint8_t)x);
}

// Verify the frame the code just transmitted: markers, func, len, CRC.
static bool checkTx(uint8_t wantFunc, const std::vector<uint8_t> &wantData) {
  const std::vector<uint8_t> &t = Serial1.tx;
  size_t need = 7 + wantData.size();
  if (t.size() != need) { printf("    (tx size %zu, want %zu)\n", t.size(), need); return false; }
  if (t[0] != 0xAA || t[1] != 0x55) return false;
  if (t[2] != wantFunc) return false;
  if ((size_t)((uint16_t)t[3] << 8 | t[4]) != wantData.size()) return false;
  for (size_t i = 0; i < wantData.size(); i++)
    if (t[5 + i] != wantData[i]) { printf("    (tx data[%zu]=%02X want %02X)\n", i, t[5+i], wantData[i]); return false; }
  uint16_t crc = crcModbus(&t[2], 3 + wantData.size());
  return t[need - 2] == (uint8_t)(crc >> 8) && t[need - 1] == (uint8_t)(crc & 0xFF);
}

static void reset() { Serial1.reset(); }

int main() {
  PicLink pic;
  printf("pic_link byte-level self-test\n");

  // ---- 1. RSP_DATA: 23-byte header, field order + endianness --------------
  {
    reset();
    std::vector<uint8_t> d;
    be32(d, 0x6812A0F0);   // 1 start_time
    d.push_back(1);        // 2 start_time_valid
    be32(d, 4);            // 3 sample_count
    be32(d, 9);            // 4 total_captures
    be32(d, 1234);         // 5 total_impulses
    be16(d, 2);            // 6 overflow_ffff
    be32(d, 300000);       // 7 sample_interval_ms
#if PHOTON_BATCH_SEQ_ENABLE
    d.push_back(0x00);     // 8 batch_seq (V064 P-5; header is 24 bytes here)
#endif
    if (d.size() != PIC_RSP_DATA_HDR_BYTES) { printf("header build wrong\n"); return 1; }
    be16(d, 0x0001); be16(d, 0xFFFF); be16(d, 0x0102); be16(d, 0x0000);

    queueFrame(RSP_DATA, d);
    PicSample s[16];
    PicReportInfo info;
    memset(&info, 0, sizeof(info));
    int n = pic.requestData(s, 16, &info);

    check("REQ_DATA request bytes are AA 55 01 00 00 00 20",
          checkTx(REQ_DATA, {}) && Serial1.tx[5] == 0x00 && Serial1.tx[6] == 0x20);
    check("RSP_DATA decoded 4 samples", n == 4);
    check("hdr field 1 start_time      (u32 BE)", info.startTime == 0x6812A0F0);
    check("hdr field 2 start_time_valid(u8)",     info.startTimeValid == 1);
    check("hdr field 3 sample_count    (u32 BE)", info.sampleCount == 4);
    check("hdr field 4 total_captures  (u32 BE)", info.totalCaptures == 9);
    check("hdr field 5 total_impulses  (u32 BE)", info.totalImpulses == 1234);
    check("hdr field 6 overflow_ffff   (u16 BE)", info.overflowFfff == 2);
    check("hdr field 7 sample_interval (u32 BE)", info.sampleIntervalMs == 300000);
    check("samples are u16 BE, oldest first",
          s[0].pulses == 1 && s[1].pulses == 0xFFFF && s[2].pulses == 0x0102 && s[3].pulses == 0);
  }

  // ---- 2. CRC rejection on the RX path ------------------------------------
  {
    reset();
    std::vector<uint8_t> d(PIC_RSP_DATA_HDR_BYTES, 0);
    queueFrame(RSP_DATA, d);
    Serial1.pending[Serial1.pending.size() - 1] ^= 0xFF;   // corrupt the CRC low byte
    PicSample s[16];
    int n = pic.requestData(s, 16, nullptr);
    check("corrupted CRC is rejected, not silently accepted", n < 0);
  }

  // ---- 3. REQ_GET_LOCO (0x0E) -> RSP_LOCO (0x8A) --------------------------
  {
    reset();
    std::vector<uint8_t> d;
    be16(d, 130);       // hf_precision
    d.push_back(1);     // hf_calibrated
    d.push_back(1);     // hf_in_range
    d.push_back(1);     // cloud_calibrated
    be16(d, 127);       // cloud_precision
    d.push_back(1);     // cloud_in_range
    be16(d, 129);       // applied_k
    queueFrame(RSP_LOCO, d);

    PicLocoStatus loco;
    memset(&loco, 0, sizeof(loco));
    bool ok = pic.getLocoStatus(loco);
    check("REQ_GET_LOCO frame is AA 55 0E 00 00 03 10",
          checkTx(REQ_GET_LOCO, {}) && Serial1.tx[5] == 0x03 && Serial1.tx[6] == 0x10);
    check("RSP_LOCO 10-byte decode", ok && loco.hfPrecision == 130 &&
          loco.hfCalibrated == 1 && loco.hfInRange == 1 && loco.cloudCalibrated == 1 &&
          loco.cloudPrecision == 127 && loco.cloudInRange == 1 && loco.appliedK == 129);
    check("applied_k is Q7 (129/128 = 1.0078)",
          ok && loco.appliedK / PIC_LOCO_Q7_ONE > 1.007f && loco.appliedK / PIC_LOCO_Q7_ONE < 1.009f);
  }

  // ---- 4. REQ_SET_LEAK (0x03, len 8) --------------------------------------
  // Appendix E: leak-only write. Byte-for-byte the old 8-byte SET_PARAM, just
  // renamed. 4x u16 big-endian, no grid appended, ever.
  {
    reset();
    PicParams p; p.leak1_counts = 10; p.leak1_window_s = 60;
                 p.leak2_counts = 20; p.leak2_window_s = 120;
    queueFrame(RSP_ACK, {});
    bool ok = pic.setLeak(p);
    check("SET_LEAK(0x03) = 4xu16 leak, len 8, no grid",
          ok && checkTx(REQ_SET_LEAK, {0x00,0x0A,0x00,0x3C,0x00,0x14,0x00,0x78}));
  }

  // ---- 5. REQ_SET_GRID (0x0F, len 8) --------------------------------------
  // Appendix E: grid-only write. anchor(u32 BE) then interval(u32 BE).
  // 3600 = 0x00000E10, 172800 = 0x0002A300.
  {
    reset();
    PicGridParams g; g.anchorSec = 3600; g.intervalSec = 172800;
    queueFrame(RSP_ACK, {});
    bool ok = pic.setGrid(g);
    check("SET_GRID(0x0F) = anchor u32 + interval u32, len 8, BE",
          ok && checkTx(REQ_SET_GRID, {0x00,0x00,0x0E,0x10, 0x00,0x02,0xA3,0x00}));
  }

  // ---- 6. RSP_PARAM 16-byte decode ---------------------------------------
  {
    reset();
    std::vector<uint8_t> d = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
    be32(d, 3600); be32(d, 172800);
    queueFrame(RSP_PARAM, d);
    PicParams p; PicGridParams g; bool gv = false;
    memset(&p, 0, sizeof(p)); memset(&g, 0, sizeof(g));
    bool ok = pic.getParams(p, &g, &gv);
    check("RSP_PARAM(16) decode incl. grid",
          ok && gv && p.leak1_counts == 0x0102 && p.leak2_window_s == 0x0708 &&
          g.anchorSec == 3600 && g.intervalSec == 172800);
  }

  // ---- 7. RSP_PARAM 8-byte legacy decode ---------------------------------
  {
    reset();
    queueFrame(RSP_PARAM, {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08});
    PicParams p; PicGridParams g; bool gv = true;
    memset(&p, 0, sizeof(p)); memset(&g, 0xEE, sizeof(g));
    bool ok = pic.getParams(p, &g, &gv);
    check("RSP_PARAM(8) accepted, gridValid reported false", ok && !gv);
  }

  // ---- 8. PKT_TIME_SYNC (0x0C) -> PKT_TIME_RECEIVED (0x0D) ---------------
  {
    reset();
    queueFrame(PKT_TIME_RECEIVED, {});
    bool ok = pic.sendTimeSync(true, 0x6812A0F0);
    check("TIME_SYNC payload = valid + epoch u32 BE",
          ok && checkTx(PKT_TIME_SYNC, {0x01, 0x68, 0x12, 0xA0, 0xF0}));
  }

  // ---- 9. NAK is surfaced, not retried into a timeout ---------------------
  {
    reset();
    queueFrame(RSP_NAK, {NAK_BUSY});
    PicParams p; p.leak1_counts = 1; p.leak1_window_s = 1;
                 p.leak2_counts = 1; p.leak2_window_s = 1;
    bool ok = pic.setLeak(p);
    check("RSP_NAK makes setLeak fail", !ok);
    check("NAK reason byte is retained", pic.lastNak() == NAK_BUSY);
    check("NAK is not retried (exactly one frame sent)", Serial1.tx.size() == 7 + 8);
  }

  // ---- 10. sample_count / length consistency guard ------------------------
  {
    reset();
    std::vector<uint8_t> d;
    be32(d, 0); d.push_back(0); be32(d, 5);   // claims 5 samples...
    be32(d, 5); be32(d, 5); be16(d, 0); be32(d, 1000);
    be16(d, 1); be16(d, 2);                   // ...but only supplies 2
    queueFrame(RSP_DATA, d);
    PicSample s[16];
    int n = pic.requestData(s, 16, nullptr);
    check("count/length mismatch is rejected", n == PIC_ERR_BAD_FRAME);
  }

  // ---- 11. the frame timeout is a budget for the WHOLE frame --------------
  // A peer that stalls mid-frame and then trickles one byte just inside the
  // timeout must not be able to stretch the transaction. Before the shared
  // deadline was enforced this ran for over two hours instead of 16 s.
  {
    reset();
    Serial1.pending.push_back(0xAA); Serial1.pending.push_back(0x55);
    Serial1.pending.push_back(RSP_DATA);
    Serial1.pending.push_back(0x07); Serial1.pending.push_back(0xD0);   // len = 2000
    for (int i = 0; i < 2002; i++) Serial1.pending.push_back(0x00);
    Serial1.freeBytes = 5;                       // marker + header at line rate...
    Serial1.trickleMs = PHOTON_TIMEOUT_DATA_MS - 100;   // ...then a stall per byte

    uint32_t t0 = hostClock();
    static PicSample big[PIC_MAX_SAMPLES];
    int n = pic.requestData(big, PIC_MAX_SAMPLES, nullptr);
    uint32_t elapsed = hostClock() - t0;
    uint32_t budget  = (uint32_t)(PHOTON_RETRY_COUNT + 1) * PHOTON_TIMEOUT_DATA_MS;
    check("a stalling peer cannot stretch the frame timeout",
          n == PIC_ERR_TIMEOUT && elapsed <= budget + budget / 10u);
    if (elapsed > budget + budget / 10u)
      printf("    (elapsed %lu ms vs budget %lu ms)\n",
             (unsigned long)elapsed, (unsigned long)budget);
  }

  // ---- 12. SET_GRID length is NAKed, not dropped --------------------------
  // Appendix E: the whole point of the split is that the PIC answers a wrong
  // length instead of silently discarding the frame. A NAK_BAD_LEN must make
  // setGrid() return false and expose the reason - never a silent success.
  {
    reset();
    PicGridParams g; g.anchorSec = 3600; g.intervalSec = 172800;
    queueFrame(RSP_NAK, {NAK_BAD_LEN});
    bool ok = pic.setGrid(g);
    check("SET_GRID NAK_BAD_LEN -> setGrid fails", !ok);
    check("SET_GRID NAK reason is retained", pic.lastNak() == NAK_BAD_LEN);
  }

  // ---- 13. the two writes are independent ---------------------------------
  // A failed SET_LEAK must not prevent SET_GRID from being sent and ACKed, and
  // each carries its own func code on the wire. This is the "if one fails the
  // other is still valid" property from Appendix E.3.
  {
    reset();
    PicParams p; p.leak1_counts = 10; p.leak1_window_s = 60;
                 p.leak2_counts = 20; p.leak2_window_s = 120;
    PicGridParams g; g.anchorSec = 0; g.intervalSec = 1800;

    queueFrame(RSP_NAK, {NAK_BUSY});     // SET_LEAK is rejected...
    queueFrame(RSP_ACK, {});             // ...SET_GRID still goes and is ACKed
    bool leakOk = pic.setLeak(p);
    bool gridOk = pic.setGrid(g);
    check("SET_LEAK can fail while SET_GRID still succeeds", !leakOk && gridOk);
    check("two distinct frames on the wire: 0x03 then 0x0F",
          Serial1.tx.size() == (7u + 8u) * 2u &&
          Serial1.tx[2] == REQ_SET_LEAK && Serial1.tx[4] == 8 &&
          Serial1.tx[7 + 8 + 2] == REQ_SET_GRID && Serial1.tx[7 + 8 + 4] == 8);
  }

  // ---- 14. GET_PARAM (0x02 -> RSP_PARAM 16) is unchanged by the split ------
  // Reads still return leak + grid in one 16-byte reply; only the WRITES split.
  {
    reset();
    std::vector<uint8_t> d = {0x00,0x0A,0x00,0x3C,0x00,0x14,0x00,0x78};
    be32(d, 3600); be32(d, 172800);
    queueFrame(RSP_PARAM, d);
    PicParams p; PicGridParams g; bool gv = false;
    memset(&p, 0, sizeof(p)); memset(&g, 0, sizeof(g));
    bool ok = pic.getParams(p, &g, &gv);
    check("GET_PARAM still reads leak + grid in one 16-byte RSP_PARAM",
          ok && gv && p.leak1_counts == 10 && p.leak2_window_s == 120 &&
          g.anchorSec == 3600 && g.intervalSec == 172800 &&
          Serial1.tx.size() == 7u &&           // exactly one REQ_GET_PARAM frame
          Serial1.tx[2] == REQ_GET_PARAM);
  }

  // ---- 15. zero-sample RSP_DATA needs no PKT_DATA_RECEIVED ----------------
  // Confirmed PIC behaviour: the PIC empties its buffer when it receives
  // REQ_DATA during the initial hold (s_read = s_end), so a zero-sample reply
  // has nothing left to commit and 0x0B would be meaningless. The link layer
  // must therefore never ack on its own - whether to send 0x0B is the caller's
  // decision, taken only after the batch is durably stored.
  {
    reset();
    std::vector<uint8_t> d;
    be32(d, 0x6812A0F0);   // 1 start_time
    d.push_back(1);        // 2 start_time_valid
    be32(d, 0);            // 3 sample_count = 0
    be32(d, 0);            // 4 total_captures
    be32(d, 0);            // 5 total_impulses
    be16(d, 0);            // 6 overflow_ffff
    be32(d, 300000);       // 7 sample_interval_ms
#if PHOTON_BATCH_SEQ_ENABLE
    d.push_back(0x07);     // 8 batch_seq (V064 P-5)
#endif
    queueFrame(RSP_DATA, d);

    PicSample s[4];
    PicReportInfo info;
    memset(&info, 0, sizeof(info));
    int n = pic.requestData(s, 4, &info);
    check("zero-sample RSP_DATA is a valid frame decoded as n=0",
          n == 0 && info.sampleCount == 0 && info.startTimeValid == 1 &&
          info.sampleIntervalMs == 300000);
    check("no PKT_DATA_RECEIVED is emitted by the link layer itself",
          Serial1.tx.size() == 7u && Serial1.tx[2] == REQ_DATA);
  }

  // ---- 16. V064 P-5: batch_seq on the 24-byte header ----------------------
  // Only meaningful when the paired switch is on. With it off, the 23-byte
  // header is byte-for-byte the V063 frame and batchSeqValid must be 0 so the
  // caller knows not to attempt de-duplication.
  {
    reset();
    std::vector<uint8_t> d;
    be32(d, 0x6812A0F0);   // 1 start_time
    d.push_back(1);        // 2 start_time_valid
    be32(d, 2);            // 3 sample_count
    be32(d, 2);            // 4 total_captures
    be32(d, 40);           // 5 total_impulses
    be16(d, 0);            // 6 overflow_ffff
    be32(d, 5290);         // 7 sample_interval_ms
#if PHOTON_BATCH_SEQ_ENABLE
    d.push_back(0x2A);     // 8 batch_seq
    if (d.size() != 24) { printf("header build wrong\n"); return 1; }
#else
    if (d.size() != 23) { printf("header build wrong\n"); return 1; }
#endif
    be16(d, 11); be16(d, 22);

    queueFrame(RSP_DATA, d);
    PicSample s[8];
    PicReportInfo info;
    memset(&info, 0, sizeof(info));
    int n = pic.requestData(s, 8, &info);

#if PHOTON_BATCH_SEQ_ENABLE
    check("P-5: 24-byte header decodes batch_seq from byte 23",
          n == 2 && info.batchSeqValid == 1u && info.batchSeq == 0x2A &&
          info.sampleIntervalMs == 5290u && s[0].pulses == 11 && s[1].pulses == 22);
#else
    check("P-5: 23-byte header decodes with batchSeqValid=0",
          n == 2 && info.batchSeqValid == 0u &&
          info.sampleIntervalMs == 5290u && s[0].pulses == 11 && s[1].pulses == 22);
#endif
  }

  // ---- 17. V064 P-5: a header built for the OTHER setting is REJECTED -----
  // The pairing is deliberately a hard failure. On a metering link a silent
  // disagreement about the layout would misread the sample count and corrupt the
  // volume accounting; refusing the frame stops the run instead.
  {
    reset();
    std::vector<uint8_t> d;
    be32(d, 0); d.push_back(0); be32(d, 1);
    be32(d, 1); be32(d, 10); be16(d, 0); be32(d, 5290);
#if PHOTON_BATCH_SEQ_ENABLE
    // 23-byte header (switch-off PIC) offered to a switch-on Photon.
#else
    d.push_back(0x2A);   // 24-byte header (switch-on PIC) offered to a switch-off Photon
#endif
    be16(d, 11);
    queueFrame(RSP_DATA, d);
    PicSample s[8];
    int n = pic.requestData(s, 8, nullptr);
    check("P-5: a mismatched header version fails hard, never misreads",
          n == PIC_ERR_BAD_FRAME);
  }

  // ---- 18. V064 P-5: the 0x0B ack echoes the batch_seq --------------------
  {
    reset();
    pic.sendDataReceived(0x2A, true);
#if PHOTON_BATCH_SEQ_ENABLE
    check("P-5: DATA_RECEIVED carries len 1 = the batch_seq being acked",
          Serial1.tx.size() == 8u && Serial1.tx[2] == PKT_DATA_RECEIVED &&
          Serial1.tx[3] == 0x00 && Serial1.tx[4] == 0x01 && Serial1.tx[5] == 0x2A);
#else
    check("P-5: with the switch off DATA_RECEIVED is the unchanged len-0 frame",
          Serial1.tx.size() == 7u && Serial1.tx[2] == PKT_DATA_RECEIVED &&
          Serial1.tx[3] == 0x00 && Serial1.tx[4] == 0x00);
#endif
  }

  printf("%s (%d failures)\n", g_fail ? "FAILURES PRESENT" : "all checks passed", g_fail);
  return g_fail ? 1 : 0;
}
