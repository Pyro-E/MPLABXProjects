#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# run_tests.sh - build and run the host-side self-tests.
#
# Compiles hourly_test.cpp and piclink_test.cpp together with the real
# src/hourly.cpp and src/pic_link.cpp against the Particle stubs, so both the
# time-axis arithmetic and the wire format are exercised on a normal Linux box.
# These are behaviour tests, unlike check.sh which only parses.
# ---------------------------------------------------------------------------
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

g++ -std=gnu++14 -Wall -Wextra -Wno-unused-parameter \
    -DPLATFORM_ID=32 -I"$HERE/stubs" -I"$ROOT/src" \
    "$HERE/hourly_test.cpp" "$ROOT/src/hourly.cpp" \
    "$HERE/stubs/stubs.cpp" -o "$HERE/.hourly_test" || exit 1

"$HERE/.hourly_test"
rc=$?
rm -f "$HERE/.hourly_test"
[ "$rc" -ne 0 ] && exit "$rc"

echo

# V068: the contract's 48-slot sliding window. Linked against the real
# src/roll48.cpp. The gap accounting is the part of this release most likely to
# be wrong and it is not reachable from any test that links leaksense.cpp, which
# is the reason the window lives in its own translation unit at all.
g++ -std=gnu++14 -Wall -Wextra -Wno-unused-parameter \
    -DPLATFORM_ID=32 -I"$HERE/stubs" -I"$ROOT/src" \
    "$HERE/roll48_test.cpp" "$ROOT/src/roll48.cpp" \
    "$HERE/stubs/stubs.cpp" -o "$HERE/.roll48_test" || exit 1

"$HERE/.roll48_test"
rc=$?
rm -f "$HERE/.roll48_test"
[ "$rc" -ne 0 ] && exit "$rc"

echo

# V068: payload sizes, measured against the REAL JsonParserGeneratorRK rather
# than estimated. The contract array is published in one message by design, so
# "does it fit" is a property that has to be checked, not assumed - and the
# library truncates silently when it does not.
# NOTE the include order: the REAL library header must come before stubs/, or
# JsonWriterStatic<N> resolves to the empty stub and the test measures nothing
# (it writes past a 1-byte object and segfaults). Particle.h still comes from
# stubs/, which is what the real library needs to compile on a host.
g++ -std=gnu++14 -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare \
    -DPLATFORM_ID=32 -I"$ROOT/lib/JsonParserGeneratorRK/src" \
    -I"$HERE/stubs" -I"$ROOT/src" \
    "$HERE/payload_test.cpp" "$ROOT/lib/JsonParserGeneratorRK/src/JsonParserGeneratorRK.cpp" \
    "$HERE/stubs/stubs.cpp" -o "$HERE/.payload_test" || exit 1

"$HERE/.payload_test"
rc=$?
rm -f "$HERE/.payload_test"
[ "$rc" -ne 0 ] && exit "$rc"

echo

# Byte-level link test: drives the real pic_link.cpp through a loopback UART and
# compares what goes on the wire against the PIC V055 packet spec, byte for byte.
g++ -std=gnu++14 -Wall -Wextra -Wno-unused-parameter \
    -DPLATFORM_ID=32 -I"$HERE/stubs" -I"$ROOT/src" \
    "$HERE/piclink_test.cpp" "$ROOT/src/pic_link.cpp" \
    "$HERE/stubs/stubs.cpp" -o "$HERE/.piclink_test" || exit 1

"$HERE/.piclink_test"
rc=$?
rm -f "$HERE/.piclink_test"
[ "$rc" -ne 0 ] && exit "$rc"

echo

# V064 P-5: the batch_seq pairing is a build-time switch, so the DEFAULT build
# above only proves the switch-OFF wire format. Build and run the same suite with
# PHOTON_BATCH_SEQ_ENABLE=1 as well - that is the configuration that ships beside
# PIC V023 with PCFG_BATCH_SEQ_ENABLE=1, and an untested half of a paired change
# is exactly the thing that turns up on the bench instead.
g++ -std=gnu++14 -Wall -Wextra -Wno-unused-parameter \
    -DPLATFORM_ID=32 -DPHOTON_BATCH_SEQ_ENABLE=1 \
    -I"$HERE/stubs" -I"$ROOT/src" \
    "$HERE/piclink_test.cpp" "$ROOT/src/pic_link.cpp" \
    "$HERE/stubs/stubs.cpp" -o "$HERE/.piclink_bseq_test" || exit 1

echo "(PHOTON_BATCH_SEQ_ENABLE=1)"
"$HERE/.piclink_bseq_test"
rc=$?
rm -f "$HERE/.piclink_bseq_test"
[ "$rc" -ne 0 ] && exit "$rc"

echo

# Flash-ring behaviour test (Appendix F.7.3). flash_buffer.cpp writes real files,
# so it is compiled with FLASH_BUFFER_PATH_PREFIX pointing at a scratch directory
# - the default empty prefix leaves the production device paths untouched.
SCRATCH="$(mktemp -d "${TMPDIR:-/tmp}/lsbring.XXXXXX")"
rm -f "$SCRATCH"/lsblk*.bin "$SCRATCH"/lsblkring.bin 2>/dev/null
g++ -std=gnu++14 -Wall -Wextra -Wno-unused-parameter \
    -DPLATFORM_ID=32 -DFLASH_BUFFER_PATH_PREFIX="\"$SCRATCH\"" \
    -I"$HERE/stubs" -I"$ROOT/src" \
    "$HERE/flashring_test.cpp" "$ROOT/src/flash_buffer.cpp" \
    "$HERE/stubs/stubs.cpp" -o "$HERE/.flashring_test" || { rm -rf "$SCRATCH"; exit 1; }

"$HERE/.flashring_test"
rc=$?
rm -f "$HERE/.flashring_test"
rm -rf "$SCRATCH"
exit $rc
