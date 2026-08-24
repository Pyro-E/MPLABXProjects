#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# check.sh - host-side syntax check of the LeakSense P2 sources.
#
# Runs `g++ -fsyntax-only` over every src/*.cpp against the Particle API stubs
# in tools/hostcheck/stubs/. This catches syntax errors, undeclared symbols,
# type mismatches, printf format bugs and the dbg_uart static_asserts WITHOUT
# needing the Particle toolchain. It is NOT a substitute for
# `particle compile p2 . --target 6.4.1`, which is still the authoritative build.
#
# Usage:  tools/hostcheck/check.sh [PLATFORM_ID]
#         PLATFORM_ID defaults to 32 (P2). Use 13 for a Boron parse check.
# ---------------------------------------------------------------------------
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
PLATFORM="${1:-32}"

FLAGS=(
  -std=gnu++14
  -fsyntax-only
  -Wall
  -Wextra
  -Wno-unused-parameter
  -Wformat=2
  -DPLATFORM_ID="$PLATFORM"
  -I"$HERE/stubs"
  -I"$ROOT/src"
)

fail=0
for f in "$ROOT"/src/*.cpp; do
  printf '%-24s ' "$(basename "$f")"
  if g++ "${FLAGS[@]}" "$f" 2> "$HERE/.err"; then
    echo "OK"
  else
    echo "FAILED"
    cat "$HERE/.err"
    fail=1
  fi
done
rm -f "$HERE/.err"

# ---------------------------------------------------------------------------
# Structural guard: the zero-sample RSP_DATA path must return BEFORE 0x0B.
#
# The PIC empties its buffer when it receives REQ_DATA during the initial hold
# (s_read = s_end), so a zero-sample reply has nothing left to commit and
# PKT_DATA_RECEIVED would be meaningless. This is a source-ordering property of
# serviceMeterFromPic() in leaksense.cpp, which the byte-level link test cannot
# reach (it links pic_link.cpp only). Checked here instead so an edit that moves
# the ack above the early return is caught rather than shipped.
# ---------------------------------------------------------------------------
printf '%-24s ' "zero-sample no-ack"
if awk '
  /int n = picLink\.requestData/      { req  = NR }
  req && !zero && /if \(n == 0\)/     { zero = NR }
  req && /picLink\.sendDataReceived/  { if (!ack) ack = NR }
  END { exit (req && zero && ack && zero < ack) ? 0 : 1 }
' "$ROOT/src/leaksense.cpp"; then
  echo "OK (n == 0 returns before sendDataReceived)"
else
  echo "FAILED - the zero-sample early return no longer precedes PKT_DATA_RECEIVED"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V059): ingestReport() must trim the already-placed prefix
# BEFORE it bins anything.
#
# A lost 0x0B ack or a replayed flash block delivers the same samples twice, and
# nothing downstream is idempotent - hourlyProcess() bins every sample it is
# handed and legacyRollingApply() accumulates with "+=". The correction is a
# source-ordering property of ingestReport(), so the behaviour tests (which link
# hourly.cpp alone) cannot reach it. Checked here instead.
# ---------------------------------------------------------------------------
printf '%-24s ' "overlap trim ordering"
if awk '
  /hourlyOverlapSkip\(/                     { if (!trim) trim = NR }
  /hourlyProcess\(s, n, eff/                { if (!bin)  bin  = NR }
  /prevCreditedImpulses = info\.totalImpulses/ { credit = NR }
  END { exit (trim && bin && credit && trim < bin && bin < credit) ? 0 : 1 }
' "$ROOT/src/leaksense.cpp"; then
  echo "OK (trim -> bin -> record credited totals)"
else
  echo "FAILED - ingestReport() no longer trims the overlap before binning"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V062, Appendix F.1-A): the no-absolute-time branch must be
# decided BEFORE any time-axis reasoning - i.e. before hourlyResolveSpanStart()
# and hourlyOverlapSkip(). The original defect was exactly the opposite order:
# the overlap guard ran first, compared a stale prevReportEnd against a fabricated
# local uptime, and returned the whole batch as "already placed", so the no-time
# path never executed. ingestReport() links the whole application, so the
# behaviour tests cannot reach it; this ordering property is checked here.
# ---------------------------------------------------------------------------
printf '%-24s ' "no-time branch first"
if awk '
  # Match the CALL sites (not comments): the resolve/overlap calls each have an
  # argument list, and the no-time branch opens with "if (!haveAbsTime) {".
  /^  if \(!haveAbsTime\) \{/                        { if (!nt)  nt  = NR }
  /startLocal = hourlyResolveSpanStart\(info,/       { if (!res) res = NR }
  /skip = hourlyOverlapSkip\(n, startLocal/          { if (!ovl) ovl = NR }
  END { exit (nt && res && ovl && nt < res && nt < ovl) ? 0 : 1 }
' "$ROOT/src/leaksense.cpp"; then
  echo "OK (!haveAbsTime handled before resolveSpanStart / overlapSkip)"
else
  echo "FAILED - the no-time branch no longer precedes the time-axis functions"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V062, Appendix F.1-C): the overlap/continuity seam must be
# consulted through prevReportEndValid, not the raw retained prevReportEnd. A
# device carrying a stale prevReportEnd from an older no-cloud firmware must not
# have it treated as a real epoch. The absolute-time path builds `prevEnd` from
# the flag and passes THAT (never prevReportEnd directly) into the two seam
# functions; and a real placement sets the flag true.
# ---------------------------------------------------------------------------
printf '%-24s ' "seam validity gate"
if awk '
  /prevEnd = prevReportEndValid \? prevReportEnd : 0u;/  { gate = NR }
  /prevReportEndValid[ \t]*= true;/                      { set  = NR }
  /hourlyOverlapSkip\(n, startLocal, endLocal, prevEnd\)/ { useo = NR }
  END { exit (gate && set && useo) ? 0 : 1 }
' "$ROOT/src/leaksense.cpp"; then
  echo "OK (prevReportEndValid gates the seam; set on real placement)"
else
  echo "FAILED - the seam is no longer gated by prevReportEndValid"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V063, Appendix F.2 / H.13): a session that had no cloud time
# must be judged NOT delivered, so the flash ring is retained and its 4-sector
# cycle can be observed. V062 pinned this with a compile-time #if BENCH_CLOUD_FAIL.
# V063 makes the verdict RUNTIME (one binary serves all four modes + injected
# faults): g_cloudPublishOk is seeded from hadTimeThisSession = Time.isValid()
# BEFORE imuPublish(), so a no-time session (CLOUD_FAIL, or an injected connect
# failure) starts NOT delivered and the ring survives. Check that ordering.
# ---------------------------------------------------------------------------
printf '%-24s ' "no-time retains ring"
if awk '
  /hadTimeThisSession = Time\.isValid\(\);/            { had = NR }
  /g_cloudPublishOk = hadTimeThisSession;/             { seed = NR }
  seed && /imuPublish\(\);/ && !pub                     { pub = NR }
  END { exit (had && seed && pub && had <= seed && seed < pub) ? 0 : 1 }
' "$ROOT/src/leaksense.cpp"; then
  echo "OK (no-time session seeds g_cloudPublishOk=false before imuPublish)"
else
  echo "FAILED - the runtime no-time NOT-delivered verdict is gone or reordered"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V063, Appendix H.3.2 / H.3.3): the SINGLE clock gate.
#
# An unsynced Particle clock reads its 2000-01-01 default (UTC 946684800), a
# value that looks like a plausible epoch. H.3 removes the whole class by making
# clockNowUtc() the ONLY place Time.now() is read in the application data path -
# it returns false / 0 when the clock is invalid, so 946684800 can never enter a
# bucket, prevReportEnd, flash span, payload or gMeter window. This guard pins
# that: Time.now() must appear in leaksense.cpp exactly once, and only inside
# clockNowUtc(). A new direct call is exactly the H.2/H.3 regression, so it fails
# the build here rather than leaking a fake epoch on the device. (dbg_uart.cpp is
# a pure diagnostic logger with its own guarded read; it is out of scope.)
# ---------------------------------------------------------------------------
printf '%-24s ' "single clock gate"
now_calls=$(grep -c 'Time\.now()' "$ROOT/src/leaksense.cpp")
if awk '
  /static inline bool clockNowUtc/         { ing = 1 }
  ing && /Time\.now\(\)/                    { inside = NR }
  ing && /^}/                               { ing = 0 }
  /Time\.now\(\)/ && !/clockNowUtc/          { seen[NR] = 1 }
  END {
    # exactly one Time.now() textual call, and it is the one inside clockNowUtc
    exit (inside != 0) ? 0 : 1
  }
' "$ROOT/src/leaksense.cpp" && [ "$now_calls" -eq 1 ]; then
  echo "OK (Time.now() only inside clockNowUtc, count=$now_calls)"
else
  echo "FAILED - a Time.now() call escapes the single clock gate (count=$now_calls)"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V064, P-1/P-3): a failed REQ_DATA must NOT destroy data.
#
# This is the regression that cost the V063 campaign its CLOUD_FAST evidence:
# the n < 0 branch called flashBufferClear() and hourlyCarryClear() on the spot,
# destroying three accumulated blocks 121 ms before a good RSP_DATA arrived. The
# fix is that the branch only RECORDS the failure (g_picAttemptFailed) and the
# verdict is taken once, later, by finishSessionContinuityVerdict().
#
# Pin both halves: the failure branch must reach its `return` without calling
# either destructive function, and the verdict function must exist and be called
# from the session path.
# ---------------------------------------------------------------------------
printf '%-24s ' "failed REQ_DATA keeps"
if awk '
  /^  if \(n < 0\) \{/                  { inbranch = 1; next }
  inbranch && /^    return;/            { inbranch = 0 }
  # Comment lines describe the OLD behaviour on purpose; only real calls count.
  inbranch && /^[[:space:]]*\/\//       { next }
  inbranch && /^[[:space:]]*\*/         { next }
  inbranch && /flashBufferClear\(/      { bad = 1 }
  inbranch && /hourlyCarryClear\(/      { bad = 1 }
  inbranch && /g_picAttemptFailed = true/ { rec = 1 }
  END { exit (rec && !bad) ? 0 : 1 }
' "$ROOT/src/leaksense.cpp"; then
  echo "OK (n < 0 records only; no clear in the branch)"
else
  echo "FAILED - the failed-REQ_DATA branch destroys data again, or stopped recording"
  fail=1
fi

printf '%-24s ' "deferred verdict wired"
if awk '
  /^static void finishSessionContinuityVerdict\(\)/ { def = NR }
  /^  finishSessionContinuityVerdict\(\);/          { if (!call) call = NR }
  END { exit (def && call && def < call) ? 0 : 1 }
' "$ROOT/src/leaksense.cpp"; then
  echo "OK (verdict defined and called after the drain)"
else
  echo "FAILED - the session-end continuity verdict is missing or never called"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V064, P-4): a confirmed break marks, it does not delete.
#
# The verdict function must call flashBufferMarkGap() and must NOT call
# flashBufferClear(). Deletion is now reserved for a DELIVERED report and the
# timebase-change path; reintroducing it here would restore the data loss that
# P-4 replaced with a marker.
# ---------------------------------------------------------------------------
printf '%-24s ' "broken series marks"
if awk '
  /^static void finishSessionContinuityVerdict\(\)/ { inf = 1; next }
  inf && /^}/                          { inf = 0 }
  inf && /flashBufferMarkGap\(/        { mark = 1 }
  inf && /flashBufferClear\(/          { bad  = 1 }
  END { exit (mark && !bad) ? 0 : 1 }
' "$ROOT/src/leaksense.cpp"; then
  echo "OK (marks a gap, never clears)"
else
  echo "FAILED - the continuity verdict deletes buffered data instead of marking it"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V065, req 6): the bucket engine must stay independent of the
# REPORT GRID.
#
# Two different periods are easy to confuse: the report grid (when the PIC powers
# us up - it holds 180 s under PIC_USE_OWN_TIMING while we ask for 1800 s) and
# the bucket width (60 s in fast cadence, 3600 s in production). Only the second
# may reach hourly.cpp. If the bucket engine ever started reading the grid
# interval, a PIC that keeps its own grid would silently misplace every bucket,
# and the PIC-side request asks specifically for confirmation that it does not.
#
# hourly.cpp takes bucketSec as a parameter and knows nothing about the grid.
# This pins that: no grid symbol may appear in the bucket engine's code.
# ---------------------------------------------------------------------------
printf '%-24s ' "bucket grid independence"
if awk '
  # Strip comments before matching, so the explanatory prose ("bucket grid
  # phase", "re-aligned grid") is not mistaken for a dependency.
  { line = $0; sub(/\/\/.*$/, "", line); sub(/^[ \t]*\*.*$/, "", line) }
  line ~ /intervalSec|g_grid|GRID_INTERVAL|gridIntervalSec/ { bad = NR; print "  leaked at line " NR ": " $0 }
  END { exit bad ? 1 : 0 }
' "$ROOT/src/hourly.cpp" "$ROOT/src/hourly.h"; then
  echo "OK (hourly engine reads bucketSec only, never the report grid)"
else
  echo "FAILED - the bucket engine now depends on the report grid interval"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V065, req 4.b): the PIC link baud must come from the config
# constant, so the static_asserts that size the RX quiesce cap and the frame
# timeout against it cannot be bypassed by a literal at the call site. That
# bypass is exactly how the 700 ms cap survived the 38400 -> 9600 change.
# ---------------------------------------------------------------------------
printf '%-24s ' "link baud from config"
if grep -qE 'picLink\.begin\(PIC_UART_BAUD\)' "$ROOT/src/leaksense.cpp" &&
   ! grep -qE 'picLink\.begin\([0-9]' "$ROOT/src/leaksense.cpp"; then
  echo "OK (picLink.begin uses PIC_UART_BAUD, not a literal)"
else
  echo "FAILED - picLink.begin() takes a literal baud; the baud-derived static_asserts no longer bind"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V068): the two hourly events must keep their separate names
# and their separate shapes.
#
# The contract requires "hourlyGallons" to be a FIXED 48-slot array. Our own
# series is variable-length by nature and that is correct - 47 and 49 are
# routine, and a flash-ring recovery delivers 96 or more. The two therefore ship
# as two events: "hourlyGallons" (fixed 48, display) and "hourlyBuckets"
# (variable, audit). The hazard being pinned is a later edit that "simplifies"
# them back into one name: same name plus different shape is precisely what
# breaks a dashboard, and it breaks it silently.
# ---------------------------------------------------------------------------
printf '%-24s ' "two hourly events"
if grep -qE 'cloudEmit\("hourlyBuckets"' "$ROOT/src/leaksense.cpp" &&
   grep -qE 'cloudEmit\("hourlyGallons"' "$ROOT/src/leaksense.cpp" &&
   [ "$(grep -cE 'cloudEmit\("hourlyGallons"' "$ROOT/src/leaksense.cpp")" = "1" ] &&
   awk '
     /static void publishHourlyBuckets/  { inBuckets = 1 }
     /static void publishHourlyRolling48/ { inBuckets = 0 }
     inBuckets && /cloudEmit\("hourlyGallons"/ { bad = 1 }
     END { exit bad ? 1 : 0 }
   ' "$ROOT/src/leaksense.cpp"; then
  echo "OK (fixed-48 hourlyGallons and variable hourlyBuckets are distinct events)"
else
  echo "FAILED - the variable-length series is publishing under the contract's fixed-48 name"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V068, request section 4): a UTC field must be a CONVERSION
# of our local axis, never a rename of it.
#
# Every epoch in this firmware is LOCAL - the offset is applied once, in the
# TIME_SYNC we hand the PIC, and everything downstream is already local. A
# contract field named ...Utc that simply carries the local value is out by the
# offset (8 h in the current deployment), and nothing in the payload reveals it.
# That is the quiet kind of wrong, so it is pinned here: every hourly*Utc
# insertKeyValue must subtract g_tzOffsetSec on the same line.
# ---------------------------------------------------------------------------
printf '%-24s ' "utc fields converted"
if awk '
  /insertKeyValue\("hourly(Base|Final)Utc"/ {
    total++
    if ($0 ~ /g_tzOffsetSec/) converted++
    else print "  not converted at line " NR ": " $0
  }
  END { exit (total > 0 && total == converted) ? 0 : 1 }
' "$ROOT/src/leaksense.cpp"; then
  echo "OK (every hourly*Utc field subtracts the offset, never renames)"
else
  echo "FAILED - a *Utc contract field carries a local epoch; the series will be published offset-hours out of place"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V068): the contract's grid-derived fields must come from the
# grid the PIC is RUNNING, not the one we asked for.
#
# Under PIC_USE_OWN_TIMING the PIC ACKs SET_GRID and then keeps its own value.
# V064 published the request (1800 s) while reports actually arrived every 180 s
# - a 10x error that stood because nothing cross-checked it. reportIntervalHr,
# reportIntervalSec, hourlyDayUtc and nextPublishEpoch are all derived from that
# pair, so they all go through contractGrid(); this pins that they cannot be
# wired to g_grid directly, one field at a time.
# ---------------------------------------------------------------------------
printf '%-24s ' "contract grid observed"
if grep -qE 'static inline void contractGrid' "$ROOT/src/leaksense.cpp" &&
   awk '
     /insertKeyValue\("(reportIntervalHr|reportIntervalSec|hourlyDayUtc)"/ {
       if ($0 ~ /g_grid\./) { print "  requested grid used at line " NR ": " $0; bad = 1 }
     }
     END { exit bad ? 1 : 0 }
   ' "$ROOT/src/leaksense.cpp"; then
  echo "OK (grid-derived contract fields read the observed grid)"
else
  echo "FAILED - a contract field publishes the grid we requested rather than the one the PIC runs"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V068): every published payload must set an explicit float
# precision.
#
# JsonWriter formats floats with "%f" when none is set - six decimal places, so
# a 12.4 gal bucket costs ten bytes instead of four - and it truncates SILENTLY
# when the result does not fit. That combination is what put the V067 sensorData
# payload over its buffer without a word in the log. Every JsonWriterStatic in
# the publish path must therefore call setFloatPlaces().
# ---------------------------------------------------------------------------
printf '%-24s ' "explicit float places"
if awk '
  /JsonWriterStatic<[0-9]+> jw;/ { writers++; pending = 1; next }
  pending && /setFloatPlaces\(/  { placed++; pending = 0 }
  pending && /JsonWriterAutoObject/ { print "  no setFloatPlaces before line " NR; pending = 0 }
  END { exit (writers > 0 && writers == placed) ? 0 : 1 }
' "$ROOT/src/leaksense.cpp"; then
  echo "OK (every publish writer sets its float precision)"
else
  echo "FAILED - a payload is built with default float formatting (six decimals, silent truncation)"
  fail=1
fi

# ---------------------------------------------------------------------------
# Structural guard (V068): the leak-event window must close on the same values
# that were just published.
#
# a1Events is a count since the last publish, so the reset has to happen AFTER
# the payload is emitted. Reset it earlier and the published count is always
# zero; forget the reset entirely and the count accumulates forever, which reads
# as a worsening leak that is not there.
# ---------------------------------------------------------------------------
printf '%-24s ' "leak window closes late"
if awk '
  /cloudEmit\("sensorData"/       { pub = NR }
  /leakEventsAfterPublish\(/      { if (!call || call < pub) call = NR }
  END { exit (pub && call && call > pub) ? 0 : 1 }
' "$ROOT/src/leaksense.cpp"; then
  echo "OK (window reset follows the sensorData publish)"
else
  echo "FAILED - the leak-event window is reset before or without the publish"
  fail=1
fi

if [ "$fail" -eq 0 ]; then
  echo "--- syntax check passed (PLATFORM_ID=$PLATFORM) ---"
  # The syntax check only proves the code parses. Run the behaviour tests too,
  # so the time-axis arithmetic is actually exercised.
  if [ "$PLATFORM" = "32" ]; then
    echo
    bash "$HERE/run_tests.sh" || fail=1
  fi
else
  echo "--- syntax check FAILED (PLATFORM_ID=$PLATFORM) ---"
fi
exit "$fail"
