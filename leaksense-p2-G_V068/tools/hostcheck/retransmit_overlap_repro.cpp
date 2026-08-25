/*
 * repro.cpp - Reproduces the ACK-loss retransmission double-count on the
 * Photon side, using the real src/hourly.cpp (V058) unmodified.
 *
 * Models exactly what leaksense.cpp does per report:
 *   startLocal = hourlyResolveSpanStart(info, endLocal, prevReportEnd, 120)
 *   hourlyProcess(...)
 *   legacyRollingApply(): hourlyData[idx] += gal ; dailyGallons += gal
 *   prevReportEnd = endLocal
 */
#include "hourly.h"
#include <stdio.h>

static float testGallons(uint16_t pulses, float dtSec) { (void)dtSec; return (float)pulses; }

static const uint32_t H = 3600u;
static const uint32_t BUCKET = 3600u;
static const uint32_t TOL = 120u;              // SPAN_CONTINUITY_TOL_SEC

static float    hourlyData[24] = {0.0f};       // retained in firmware
static float    dailyGallons   = 0.0f;         // retained in firmware
static uint32_t prevReportEnd  = 0u;           // retained in firmware
static HourlyCarry carry;

// Mirror of legacyRollingApply() in leaksense.cpp (lines 403-427).
static void legacyRollingApply(const HourlyResult &h) {
  for (uint16_t i = 0; i < h.count; i++) {
    uint32_t binStart = h.baseLocal + (uint32_t)i * h.bucketSec;
    int idx = (int)((binStart / h.bucketSec) % 24u);
    hourlyData[idx] += h.gal[i];
    dailyGallons    += h.gal[i];
  }
}

static PicReportInfo mkInfo(uint32_t start, uint32_t n, uint32_t caps, uint32_t imp) {
  PicReportInfo i; memset(&i, 0, sizeof(i));
  i.startTime = start; i.startTimeValid = 1; i.sampleCount = n;
  i.totalCaptures = caps; i.totalImpulses = imp; i.sampleIntervalMs = H * 1000u;
  return i;
}

// Mirror of ingestReport() placement + accounting.
static void ingest(const PicSample *s, uint32_t n, const PicReportInfo &info,
                   uint32_t endLocal, const char *tag) {
  const char *src = "";
  uint32_t startLocal = hourlyResolveSpanStart(info, endLocal, prevReportEnd, TOL, &src);
  HourlyResult r;
  bool ok = hourlyProcess(s, n, info, startLocal, endLocal, BUCKET,
                          (float)H, testGallons, PIC_MISSED_FILL_AVERAGE, carry, r);
  printf("  %-10s start=%lu (%s) end=%lu n=%lu -> %u buckets, base=%lu\n",
         tag, (unsigned long)startLocal, src, (unsigned long)endLocal,
         (unsigned long)n, (unsigned)r.count, (unsigned long)r.baseLocal);
  if (!ok) { printf("  %-10s NOT PLACED\n", tag); return; }
  legacyRollingApply(r);
  prevReportEnd = endLocal;
}

int main() {
  hourlyCarryClear(carry);

  const uint32_t T0 = 100u * 24u * H;            // some local epoch, on the hour
  const uint32_t T1 = T0 + 6u * H;
  const uint32_t T2 = T0 + 12u * H;

  // Session 1: 6 hourly samples, 1 pulse (= 1 gal) each. Ingested and stored.
  PicSample a[6];
  for (int i = 0; i < 6; i++) a[i].pulses = 1;
  printf("session 1 (batch is ingested, then the 0x0B ACK is LOST on the wire)\n");
  ingest(a, 6, mkInfo(T0, 6, 6, 6), T1, "RPT");
  printf("  daily after session 1 = %.1f gal (true water so far = 6.0)\n\n", dailyGallons);

  // Session 2: the PIC never saw the ACK, so it resends the same 6 samples plus
  // 6 new ones. total_impulses is now "everything since the last ACK" = 12.
  PicSample b[12];
  for (int i = 0; i < 12; i++) b[i].pulses = 1;
  printf("session 2 (PIC retransmits the same span T0..T1 plus 6 new samples)\n");
  ingest(b, 12, mkInfo(T0, 12, 12, 12), T2, "RPT");
  printf("  daily after session 2 = %.1f gal (true water so far = 12.0)\n\n", dailyGallons);

  printf("published hourlyData[] hours 0..11 (each should be 1.0):\n   ");
  for (int i = 0; i < 12; i++) printf(" %.1f", hourlyData[i]);
  printf("\n\nverdict: %s\n",
         (dailyGallons > 12.5f) ? "DOUBLE-COUNTED (no idempotency guard)" : "ok");
  return 0;
}

/*
 * Build and run (from the extracted V058 tree):
 *
 *   g++ -std=gnu++14 -DPLATFORM_ID=32 \
 *       -Itools/hostcheck/stubs -Isrc \
 *       retransmit_overlap_repro.cpp src/hourly.cpp tools/hostcheck/stubs/stubs.cpp \
 *       -o repro && ./repro
 *
 * Variant 2 (the ring already dropped the old samples, only the totals carry
 * them): change session 2 to 6 samples but keep mkInfo(T0, 6, 12, 12) and the
 * end at T2. missed-fill then restores the already-placed volume a second time.
 */
