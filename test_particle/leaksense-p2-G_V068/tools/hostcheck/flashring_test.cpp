/*
 * flashring_test.cpp  -  Host-only self-test for the flash-buffer ring (V062).
 *
 * Appendix F.7.3: with the report judged NOT delivered (BENCH_CLOUD_FAIL), a
 * repeated failure must fill the 4-sector ring 1,2,3,4 and then cycle, the fifth
 * store overwriting the oldest seq. This test drives flash_buffer.cpp's store /
 * count / load bookkeeping directly.
 *
 * flash_buffer.cpp writes real files. It is compiled here with
 * FLASH_BUFFER_PATH_PREFIX pointing at a scratch directory so nothing touches the
 * device paths; with the default empty prefix the production paths are unchanged.
 *
 * Build/run: driven by run_tests.sh (compiled with -DFLASH_BUFFER_PATH_PREFIX).
 */

#include "flash_buffer.h"
#include <stdio.h>
#include <string.h>

static int  g_fail = 0;
static void check(bool ok, const char *what) {
  printf("  %-62s %s\n", what, ok ? "PASS" : "FAIL");
  if (!ok) g_fail++;
}

static FlashBlockHeader mkHdr(uint32_t sampleCount) {
  FlashBlockHeader h;
  memset(&h, 0, sizeof(h));
  h.startLocal       = 0u;
  h.startValid       = 0u;
  h.endLocal         = 0u;
  h.endValid         = 0u;             // no-time block, as in BENCH_CLOUD_FAIL
  h.sampleCount      = sampleCount;
  h.totalCaptures    = sampleCount;
  h.totalImpulses    = sampleCount * 10u;
  h.overflowFfff     = 0u;
  h.sampleIntervalMs = 5290u;
  return h;
}

int main() {
  printf("flash ring self-test (Appendix F.7.3)\n");

#if FLASH_BUFFER_BLOCKS > 0
  // Start from a clean ring (a stale file from a previous run must not leak in).
  flashBufferClear("test setup");
  flashBufferBegin();
  // flashBufferBegin() may have restored a persisted ring from an earlier run;
  // force it empty by clearing again now that bookkeeping is loaded.
  flashBufferClear("test setup 2");
  check(flashBufferCount() == 0u, "ring starts empty");

  const uint16_t CAP = (uint16_t)FLASH_BUFFER_BLOCKS;   // 4 in the default config
  PicSample s[4]; for (int i = 0; i < 4; i++) s[i].pulses = (uint16_t)(100 + i);

  // Store CAP blocks: the count must climb 1,2,...,CAP and saturate there.
  bool climbOk = true;
  for (uint16_t i = 0; i < CAP; i++) {
    flashBufferStore(mkHdr(4), s, 4);
    if (flashBufferCount() != (uint16_t)(i + 1u)) climbOk = false;
  }
  check(climbOk, "count climbs 1..CAP as failed reports accumulate");
  check(flashBufferCount() == CAP, "ring is full at CAP blocks");

  // The oldest block still readable is the first one we stored (seq starts at 1).
  {
    FlashBlockHeader h; PicSample out[4];
    int n = flashBufferLoad(0, &h, out, 4);
    check(n == 4 && h.seq == 1u, "oldest held block is seq=1 before any wrap");
  }

  // One more store past full: count stays at CAP and the oldest seq is overwritten
  // (the new oldest becomes seq=2). This is the ring cycling per spec.
  flashBufferStore(mkHdr(4), s, 4);
  check(flashBufferCount() == CAP, "count stays at CAP after the ring wraps");
  {
    FlashBlockHeader h; PicSample out[4];
    int n = flashBufferLoad(0, &h, out, 4);
    check(n == 4 && h.seq == 2u,
          "after wrap the oldest held block is seq=2 (seq=1 overwritten)");
  }

  // The newest block is the one just written; its seq is CAP+1.
  {
    FlashBlockHeader h; PicSample out[4];
    int n = flashBufferLoad((uint16_t)(CAP - 1u), &h, out, 4);
    check(n == 4 && h.seq == (uint32_t)(CAP + 1u), "newest held block is seq=CAP+1");
  }

  // A delivered report clears the ring (the success path).
  flashBufferClear("report delivered to the cloud");
  check(flashBufferCount() == 0u, "a delivered report clears the ring");

  // endValid survives a store/load round-trip (Appendix F.3).
  {
    FlashBlockHeader in = mkHdr(3); in.endValid = 1u; in.endLocal = 123456u;
    flashBufferStore(in, s, 3);
    FlashBlockHeader h; PicSample out[3];
    int n = flashBufferLoad(0, &h, out, 3);
    check(n == 3 && h.endValid == 1u && h.endLocal == 123456u,
          "endValid / endLocal round-trip through the block header");
    flashBufferClear("test teardown");
  }

  // ---- V064 P-4: a confirmed break MARKS, it does not delete ---------------
  {
    flashBufferClear("gap test setup");
    flashBufferStore(mkHdr(4), s, 4);
    flashBufferStore(mkHdr(4), s, 4);
    uint16_t before = flashBufferCount();

    // The whole point: marking must leave every held block in place. V063
    // deleted them here, which is what made 4/4 accumulation unreachable.
    flashBufferMarkGap("no RSP_DATA received in the whole session");
    check(flashBufferCount() == before && before == 2u,
          "marking a gap RETAINS the held blocks (V063 deleted them)");
    check(flashBufferGapPending(),
          "the marker is pending until a block carries it");

    // Consecutive dead sessions are still one hole, not several.
    flashBufferMarkGap("another dead session");
    check(flashBufferGapPending(), "a second mark does not stack a second marker");

    // The next block stored carries the marker, and the flag is consumed.
    flashBufferStore(mkHdr(4), s, 4);
    check(!flashBufferGapPending(), "storing a block consumes the pending marker");
    {
      FlashBlockHeader h; PicSample out[4];
      int n = flashBufferLoad(2, &h, out, 4);
      check(n == 4 && h.gapBefore == 1u,
            "the block after the gap is stamped gapBefore=1");
    }
    // Blocks stored before the break must NOT be stamped: the discontinuity is
    // between them and what follows, not inside the run they belong to.
    {
      FlashBlockHeader h; PicSample out[4];
      int n = flashBufferLoad(0, &h, out, 4);
      check(n == 4 && h.gapBefore == 0u,
            "blocks before the gap are untouched (gapBefore=0)");
    }
    // A block stored after the marker has been consumed is continuous again.
    flashBufferStore(mkHdr(4), s, 4);
    {
      FlashBlockHeader h; PicSample out[4];
      int n = flashBufferLoad(3, &h, out, 4);
      check(n == 4 && h.gapBefore == 0u,
            "the next block again is continuous (marker not sticky)");
    }
    flashBufferClear("gap test teardown");
    check(!flashBufferGapPending(),
          "clearing an emptied ring drops a pending marker with it");
  }
#else
  check(true, "flash buffering disabled at compile time - nothing to cycle");
#endif

  printf("%s (%d failure%s)\n", g_fail ? "FAILED" : "all checks passed",
         g_fail, g_fail == 1 ? "" : "s");
  return g_fail ? 1 : 0;
}
