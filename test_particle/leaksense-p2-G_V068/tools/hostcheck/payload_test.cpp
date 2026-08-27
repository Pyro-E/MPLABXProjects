/*
 * payload_test.cpp  -  Host-only size check for the V068 published payloads.
 *
 * Not part of the firmware build. Compiled and run by tools/hostcheck/check.sh
 * against the REAL vendored JsonParserGeneratorRK, not the stub, because the
 * question being asked here is precisely how many bytes that library emits.
 *
 * WHY THIS EXISTS
 * ---------------
 * JsonWriter truncates silently. If a payload does not fit its buffer, the tail
 * of the object - including the closing brace - is simply dropped, isTruncated()
 * goes true, and nothing else happens. Up to V067 nobody read that flag, and two
 * payloads were over their buffers:
 *
 *   - no setFloatPlaces() call existed anywhere, so every float was formatted
 *     with "%f", i.e. SIX decimal places. A 12.4 gal bucket published as
 *     "12.400000", ten bytes instead of four.
 *   - sensorData was built in a JsonWriterStatic<512> while carrying ~35 fields
 *     and a 24-element float array.
 *
 * The contract's 48-slot array is published as ONE message by design (a sliding
 * window that has been split in half is not a sliding window), so "does it fit"
 * is a property that must be checked rather than assumed. The firmware checks it
 * at runtime too - see the isTruncated() guards in publishHourlyRolling48() and
 * imuPublish() - but a bench run is a slow way to discover a size problem.
 *
 * MAINTENANCE NOTE
 * ----------------
 * The builders below MIRROR the publish path; they are not the publish path.
 * Adding a field to publishHourlyRolling48() or imuPublish() without adding it
 * here makes this test optimistic. The runtime guards are what actually protect
 * the device; this suite is the early warning that keeps the bench from being
 * the place you find out.
 *
 * Build/run:  tools/hostcheck/run_tests.sh
 */

#include "Particle.h"
#include "app_config.h"
#include "JsonParserGeneratorRK.h"
#include <stdio.h>
#include <math.h>

static int  g_fail = 0;
static void check(bool ok, const char *what) {
  printf("  %-62s %s\n", what, ok ? "PASS" : "FAIL");
  if (!ok) g_fail++;
}

static float roundDecimals(float v, uint8_t places) {
  float m = 1.0f;
  for (uint8_t i = 0; i < places; i++) m *= 10.0f;
  return floorf(v * m + 0.5f) / m;
}

// ---- The contract's fixed 48-slot event ------------------------------------
// 'perSlot' is the gallons value used in every slot, so a worst case can be
// driven in: a production house at 12.4 gal/h is the ordinary case, 128.5 gal/h
// is a large-property upper bound.
static size_t buildRolling48(float perSlot, uint8_t decimals, bool *truncated) {
  JsonWriterStatic<1024> jw;
  jw.setFloatPlaces(decimals);
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("pf",               "P2");
    jw.insertKeyValue("bucketSec",        (int)3600);
    jw.insertKeyValue("hourlyBaseUtc",    (int)1787068800);
    jw.insertKeyValue("hourlyFinalUtc",   (int)1787241600);
    jw.insertKeyValue("hourlyBaseLocal",  (int)1787040000);
    jw.insertKeyValue("hourlyFinalLocal", (int)1787212800);
    jw.insertKeyValue("tzOffsetSec",      (int)-28800);
    jw.insertKeyValue("HourlyrReset",     (double)3.7);
    jw.insertKeyValue("hourlyDayUtc",     (int)8);
    jw.insertKeyValue("reportIntervalHr", (int)48);
    jw.insertKeyValue("reportIntervalSec",(int)172800);
    jw.insertKeyValue("slotsFilled",      (int)48);
    jw.insertKeyValue("slotsGapFilled",   (int)0);
    jw.insertKeyArray("hourlyGallons");
    for (uint8_t i = 0; i < ROLL48_COUNT; i++)
      jw.insertArrayValue(roundDecimals(perSlot, decimals));
    jw.finishObjectOrArray();
  }
  if (truncated) *truncated = jw.isTruncated();
  return jw.getOffset();
}

// ---- One chunk of the variable-length audit series --------------------------
static size_t buildBuckets(uint16_t perChunk, float perBucket, uint8_t decimals,
                           bool *truncated) {
  JsonWriterStatic<1024> jw;
  jw.setFloatPlaces(decimals);
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("platform",        "P2");
    jw.insertKeyValue("bucketSec",       (int)3600);
    jw.insertKeyValue("hourlyBaseLocal", (int)1787040000);
    jw.insertKeyValue("hourlyFinalLocal",(int)1787212800);
    jw.insertKeyValue("hourlyBaseUtc",   (int)1787068800);
    jw.insertKeyValue("hourlyFinalUtc",  (int)1787241600);
    jw.insertKeyValue("tzOffsetSec",     (int)-28800);
    jw.insertKeyValue("spanStart",       (int)1787040000);
    jw.insertKeyValue("spanEnd",         (int)1787212800);
    jw.insertKeyValue("samplesUsed",     (int)1234);
    jw.insertKeyValue("bucketsMakeable", (int)192);
    jw.insertKeyValue("bucketsSent",     (int)192);
    jw.insertKeyValue("chunk",           (int)1);
    jw.insertKeyValue("totalChunks",     (int)3);
    jw.insertKeyValue("indexStart",      (int)0);
    jw.insertKeyValue("count",           (int)perChunk);
    jw.insertKeyValue("equalized",       (int)1);
    jw.insertKeyArray("hourlyBuckets");
    for (uint16_t i = 0; i < perChunk; i++)
      jw.insertArrayValue(roundDecimals(perBucket, decimals));
    jw.finishObjectOrArray();
  }
  if (truncated) *truncated = jw.isTruncated();
  return jw.getOffset();
}

// ---- sensorData, including the V068 contract fields -------------------------
template <size_t BUF>
static size_t buildSensorData(int decimals, bool *truncated) {
  JsonWriterStatic<BUF> jw;
  if (decimals >= 0) jw.setFloatPlaces(decimals);
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("platform", "P2");
    jw.insertKeyValue("sensor", (int)0);
    jw.insertKeyValue("leaking", (int)0);
    jw.insertKeyValue("shutoff", (int)0);
    jw.insertKeyValue("overflow", (int)0);
    jw.insertKeyValue("temp", (float)23.5f);
    jw.insertKeyValue("flowCal", (float)1.255f);
    jw.insertKeyValue("cfgLeakGpm", (float)0.5f);
    jw.insertKeyValue("cfgShutoffVol", (float)50.0f);
    jw.insertKeyValue("cfgAutoShutoff", (int)1);
    jw.insertKeyValue("cfgAlertMode", (int)1);
    jw.insertKeyValue("picLeak1Counts", (int)100);
    jw.insertKeyValue("picLeak1WinS", (int)480);
    jw.insertKeyValue("picLeak2Counts", (int)400);
    jw.insertKeyValue("picLeak2WinS", (int)180);
    jw.insertKeyValue("picParamsDirty", (int)0);
    jw.insertKeyValue("valveMotion", (int)2);
    jw.insertKeyValue("valveLockFlags", (int)3);
    jw.insertKeyValue("valvePwr", (int)0);
    jw.insertKeyValue("valveCtrl", (int)0);
    jw.insertKeyValue("valveTempLocks", (int)182);
    jw.insertKeyValue("bucketSec", (int)3600);
    jw.insertKeyValue("bucketsMakeable", (int)48);
    jw.insertKeyValue("bucketsSent", (int)48);
    jw.insertKeyValue("tzOffsetSec", (int)-28800);
    jw.insertKeyValue("gridAnchorSec", (int)0);
    jw.insertKeyValue("gridIntervalSec", (int)172800);
    jw.insertKeyValue("gridFromPic", (int)1);
    jw.insertKeyValue("carryPulses", (int)426);
    // ---- V068 contract fields ----
    jw.insertKeyValue("pf", "P2");
    jw.insertKeyValue("Cal", (float)1.255f);
    jw.insertKeyValue("a1Count", (int)100);
    jw.insertKeyValue("a1Win", (int)480);
    jw.insertKeyValue("a2Count", (int)400);
    jw.insertKeyValue("a2Win", (int)180);
    jw.insertKeyValue("reportIntervalHr", (int)48);
    jw.insertKeyValue("reportIntervalSec", (int)172800);
    jw.insertKeyValue("hourlyDayUtc", (int)8);
    jw.insertKeyValue("nextPublishEpoch", (int)1787241600);
    jw.insertKeyValue("a1Events", (int)8);
    jw.insertKeyValue("a1WindowSec", (int)172800);
    jw.insertKeyValue("a2Events", (int)1);
    jw.insertKeyValue("Shutoffs", (int)1);
    jw.insertKeyValue("lifetimeGal", (double)123456.789);
    // Req 5: no [0-23] array on the wire. The contract window is hourlyGallons[48].
    jw.insertKeyValue("rssi", (int)-62);
    jw.insertKeyValue("battery", (float)3.912f);
    jw.insertKeyValue("freeMem", (int)123456);
    jw.insertKeyValue("uptime", (int)104);
  }
  if (truncated) *truncated = jw.isTruncated();
  return jw.getOffset();
}

int main() {
  printf("publish payload size self-test (V068)\n");

  bool tr = false;

  // ---- 1. The contract array at the shipping settings ---------------------
  {
    size_t prod = buildRolling48(12.4f, ROLL48_DECIMALS, &tr);
    printf("       hourlyGallons  production 12.4 gal/h, %u dp : %4u bytes\n",
           (unsigned)ROLL48_DECIMALS, (unsigned)prod);
    check(!tr, "hourlyGallons: production payload is not truncated");
    check(prod < CONTRACT_EVENT_WARN_BYTES,
          "hourlyGallons: production payload is under the 600-byte warning line");

    size_t big = buildRolling48(128.5f, ROLL48_DECIMALS, &tr);
    printf("       hourlyGallons  large 128.5 gal/h,     %u dp : %4u bytes\n",
           (unsigned)ROLL48_DECIMALS, (unsigned)big);
    check(!tr, "hourlyGallons: large-property payload is not truncated");
    check(big < PARTICLE_EVENT_MAX_BYTES,
          "hourlyGallons: large-property payload is inside the Particle event limit");
  }

  // ---- 2. Why ROLL48_DECIMALS is 1 and not 2 ------------------------------
  // Two decimal places on a large property is the case the runtime warning is
  // sized for: still deliverable, but close enough that it must be visible.
  {
    size_t two = buildRolling48(128.5f, 2, &tr);
    printf("       hourlyGallons  large 128.5 gal/h,     2 dp : %4u bytes\n",
           (unsigned)two);
    check(!tr, "hourlyGallons: 2 dp still fits the buffer");
    check(two > CONTRACT_EVENT_WARN_BYTES,
          "hourlyGallons: 2 dp crosses the warning line, so the WARN is not decorative");
  }

  // ---- 3. The audit series, at the chunk size this build ships ------------
  {
    size_t chunk = buildBuckets(HOURLY_PUBLISH_PER_CHUNK, 1234.567f,
                                HOURLY_BUCKET_DECIMALS, &tr);
    printf("       hourlyBuckets  %u buckets, worst case, %u dp : %4u bytes\n",
           (unsigned)HOURLY_PUBLISH_PER_CHUNK, (unsigned)HOURLY_BUCKET_DECIMALS,
           (unsigned)chunk);
    check(!tr, "hourlyBuckets: a full worst-case chunk is not truncated");
    check(chunk < PARTICLE_EVENT_MAX_BYTES,
          "hourlyBuckets: a full worst-case chunk is inside the Particle event limit");

    // A production 48-bucket report must still go out as ONE event, or the
    // session event count in the request's section 11 no longer holds.
    check(HOURLY_PUBLISH_PER_CHUNK >= 48,
          "hourlyBuckets: a 48-bucket production report still fits one chunk");
  }

  // ---- 4. The defect this suite was written to pin ------------------------
  // At 96 buckets and three decimals - the combination V067's chunk size would
  // have produced once the audit array gained precision - the payload does not
  // fit and the library says nothing. If this check ever reports "fits", the
  // silent-truncation hazard has been removed by something else and this case
  // should be re-examined rather than deleted.
  {
    size_t over = buildBuckets(96, 1234.567f, HOURLY_BUCKET_DECIMALS, &tr);
    printf("       hourlyBuckets  96 buckets (the V067 chunk size)  : %4u bytes, truncated=%d\n",
           (unsigned)over, (int)tr);
    check(tr, "hourlyBuckets: 96 x 3 dp DOES overrun - which is why the chunk is now 64");
  }

  // ---- 5. sensorData ------------------------------------------------------
  {
    size_t v068 = buildSensorData<1024>(HOURLY_BUCKET_DECIMALS, &tr);
    printf("       sensorData     V068, typical               : %4u bytes\n",
           (unsigned)v068);
    check(!tr, "sensorData: the V068 payload fits its 1024-byte buffer");
    check(v068 < PARTICLE_EVENT_MAX_BYTES,
          "sensorData: the V068 payload is inside the Particle event limit");

    size_t worst = buildSensorData<1024>(HOURLY_BUCKET_DECIMALS, &tr);
    printf("       sensorData     V068, no 24-slot array      : %4u bytes\n",
           (unsigned)worst);
    check(!tr, "sensorData: payload still fits without hourlyRolling24");
    check(worst < PARTICLE_EVENT_MAX_BYTES,
          "sensorData: payload is inside the Particle event limit");

    // V067 truncated because it stuffed a 24-slot array into a 512-byte buffer
    // with unbounded "%f" floats. That array is gone (req 5); the 1024-byte
    // buffer is the shipping size. Keep a 512-byte probe as a size print only.
    bool trOld = false;
    size_t v067 = buildSensorData<512>(-1, &trOld);
    printf("       sensorData     512 buf, %%f (size probe)    : %4u bytes, truncated=%d\n",
           (unsigned)v067, (int)trOld);
  }

  printf("%s (%d failure%s)\n", g_fail ? "FAILED" : "all checks passed",
         g_fail, g_fail == 1 ? "" : "s");
  return g_fail ? 1 : 0;
}
