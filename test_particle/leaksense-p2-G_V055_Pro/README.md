# LeakSense P2 — Integrated Firmware

Merges doc 1 (LeakSense flow/leak/publish) and doc 2 (PIC18F06Q40 UART meter
reader) into a single Particle **Photon 2 (P2)** project. Covers Kevin's four
requirements.

| # | Requirement | Where it lives |
|---|-------------|----------------|
| 1 | Ingest PIC data + align to hourly intervals | `pic_link.*`, `ingestPicBatch()`, `accumulateHourly()` |
| 2 | Pass 4 variables cloud → Photon → PIC | `setLeakParams()` → `PicLink::setParams()` (`REQ_SET_PARAM` packet) |
| 3 | Keep Boron compatible | `PLATFORM_ID` guards in `app_config.h` (Kevin verifies HW) |
| 4 | Cloud connection only every 24 h | `runSleep()` 24h + epoch gate in `handleMonitoring()` |

> **V040 protocol update:** the PIC link is now the **framed packet protocol**
> from the *PIC ↔ Photon2 Interface Specification* (`AA 55` marker + CRC-16/MODBUS
> in both directions). The old raw `0xAA → COUNT+samples` upload and the
> home-grown `0xC0` config frame are **gone**. The four cloud→PIC variables are
> now the PIC's own leak parameters (`leak1/leak2 counts + window`), and a new
> motorized-valve subsystem (status read + lock clearing + reset) is wired in.
> See §4 and §10.

---

## 1. File layout

```
leaksense-p2/
├─ project.properties        # build config (no registry deps needed — see below)
├─ lib/
│  └─ JsonParserGeneratorRK/ # VENDORED (local toolchain doesn't auto-fetch deps)
│     ├─ library.properties
│     └─ src/JsonParserGeneratorRK.{h,cpp}
└─ src/
   ├─ app_config.h           # pin map · calibration · intervals · host cfg + PIC leak params
   ├─ pic_link.h / .cpp      # PIC UART framed packet protocol (AA55+CRC, V040)
   └─ leaksense.cpp          # main: ingest · leak · publish · sleep · persistence · cloud fns
```

The project is **self-contained**: with the default config (`USE_IMU 0`) it builds
with no registry downloads, because the only library it needs
(`JsonParserGeneratorRK`) is vendored under `lib/`.

## 2. Runtime flow (24-hour cycle)

- **PIC wakes the P2 (D10 ↑)** → the P2 reads the batch with `REQ_DATA` (framed,
  CRC-checked; re-requested on bad CRC), folds it into the hourly buckets /
  interval log / leak model, then sleeps again **without connecting to the
  cloud** (core of requirement #4).
- **24h timer wake** → drain any remaining PIC data via the WAKE handshake +
  `REQ_DATA`, connect to the cloud, publish `sensorData` + `meterIntervals`
  chunks, push any pending leak parameters to the PIC, refresh valve status,
  then sleep.
- **Button (MODE/A1) wake** → publish immediately.
- The device does not sleep while a leak/shutoff is active or while flow is in progress.

## 3. Cloud config: host analytics vs. PIC leak parameters (requirement #2)

There are now **two** independent four-value config groups, because in V040 the
PIC runs its **own** leak detection and valve, while the P2 keeps its own
host-side analytics.

### 3a. Host analytics — `setConfig` (unchanged API, host-only)

```
particle call <device> setConfig "5.0,30.0,1,1"
                                   │   │   │ └ alertMode    0=off 1=publish 2=publish+extra
                                   │   │   └── autoShutoff  0/1 (auto-close the LOCAL SSR valve)
                                   │   └────── shutoffVolGal  30-min cumulative shutoff threshold (gallons)
                                   └────────── leakThreshGpm  instantaneous host leak-alert threshold (GPM)
```

- Drives the P2's own leak model, dashboard echo, and the local SSR valve (D3/D4).
- Stored in **retained + EEPROM**. Echoed back in `sensorData` as
  `cfgLeakGpm/cfgShutoffVol/cfgAutoShutoff/cfgAlertMode`.
- **No longer sent to the PIC** — the old `0xC0` frame has no V040 equivalent.

### 3b. PIC leak parameters — `setLeakParams` (cloud → P2 → PIC)

These are the literal `REQ_SET_PARAM` payload (4×`uint16`, big-endian):

```
particle call <device> setLeakParams "100,480,400,180"
                                       │   │   │  └ leak2_window_s  alert-2 window (s)  → permanent lock
                                       │   │   └─── leak2_counts    alert-2 threshold (counts)
                                       │   └─────── leak1_window_s  alert-1 window (s)  → temporary lock
                                       └─────────── leak1_counts    alert-1 threshold (counts)
```

- Defaults `100, 480, 400, 180` (match the spec's verified `REQ_SET_PARAM` frame).
- **Read-modify-write**: the P2 first does `REQ_GET_PARAM` to pull the live four,
  applies your new values, then `REQ_SET_PARAM` writes all four (there is no
  per-field write). Cached in **retained + EEPROM** so they can be re-pushed
  after a reset.
- Echoed in `sensorData` as `picLeak1Counts/picLeak1WinS/picLeak2Counts/picLeak2WinS`
  plus `picParamsDirty` (1 = a write is still pending delivery).
- `getLeakParams` refreshes the cache straight from the PIC; `syncPic` force-writes
  the cached values now.

## 4. PIC link = framed packet protocol (V040)

`pic_link.*` implements the *PIC ↔ Photon2 Interface Specification* exactly. Every
message, both directions, is:

```
AA 55 | func | len_hi len_lo | data[len] | crc_hi crc_lo
```

- **CRC-16/MODBUS** — poly `0xA001`, init `0xFFFF`, no final XOR, over
  `func+len+data` (the `AA 55` marker is excluded), transmitted **big-endian**
  (`crc_hi` then `crc_lo`). Verified against all six example frames in spec §8.
- **All multi-byte fields are big-endian.**
- **WAKE handshake (spec 5.3):** WAKE HIGH → send immediately; WAKE LOW → send
  `0xF0`, wait for WAKE HIGH (`PHOTON_WAKE_WAIT_MS`, resend `0xF0` up to
  `PHOTON_WAKE_RETRIES`), then send. WAKE stays HIGH while bytes flow either way,
  so back-to-back packets keep the PIC awake; it drops 500 ms after the link
  goes quiet.
- **One request at a time:** after a `REQ_*` the P2 waits for the matching
  `RSP_*` before sending anything else, and **resends the same request** on
  timeout or bad CRC (up to `PHOTON_RETRY_COUNT`). `RSP_NAK` is reported, not
  retried.

Functions implemented (`PicLink`):

| Call | Frame(s) | Purpose |
|------|----------|---------|
| `requestData()` | `REQ_DATA` → `RSP_DATA` | Pull `COUNT` + 10-14 packed samples |
| `getParams()` / `setParams()` | `REQ_GET_PARAM`/`REQ_SET_PARAM` → `RSP_PARAM`/`ACK` | Read / write the 4 leak params |
| `getValve()` | `REQ_GET_VALVE` → `RSP_VALVE` | Pins, motion (0..6), lock flags, temp-lock count |
| `unlockValve(flags)` | `REQ_VALVE_UNLOCK` → `ACK` | Clear temp / perm / both locks |
| `sysReset()` | `PKT_SYS_RESET` | Reset PIC (no reply; clears the permanent lock) |

> The PIC is the authority on leak/valve behaviour now — it closes the valve on
> alert (temporary lock auto-reopens after 10 min; permanent lock holds until
> `unlockValve` or reset). The P2 only **reads** valve status and **clears**
> locks; there is no "write valve position" packet.

## 5. ⚠️ PIC sample → time alignment assumption

The guide's samples carry no absolute time, only `index (0..1023)`. So the following
is **assumed** (`ingestPicBatch()`):

> The batch is a contiguous run of samples; each sample spans
> `PIC_SAMPLE_INTERVAL_SEC` (default 60 s), and **the last sample ends at the most
> recent interval boundary before now.**

If the PIC can send a real timestamp (or the epoch of the first sample), just replace
the `tsEnd` computation in `ingestPicBatch()` with that value. Also set
`PIC_SAMPLE_INTERVAL_SEC` to match the PIC's actual sampling period; by default it is
aligned 1:1 with the logger interval (60 s).

## 6. P2 retained-memory handling

The P2 retained-memory limits flagged earlier are worked around as follows.

- **Large buffers** (`gMeter` ~2.9 KB, leak model `model` ~18 KB) live in normal RAM.
  ULTRA_LOW_POWER sleep preserves RAM, so sleep/wake is fine; for reset/power-loss
  resilience they are **persisted to LittleFS** (`/usr/gmeter.dat`,
  `/usr/leakmodel.dat`) via `persistAll()` (on publish and just before sleep).
- Only **small critical state** (hourly gallons, counters, config, etc.) is kept in
  `retained`, and on the P2 it is flushed to flash with `System.backupRamSync()`. This
  keeps usage well under the 3068-byte limit.
- Includes `STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));` plus dual storage
  of config/calibration in EEPROM.

> Effect: leak learning (one week) and the current-day interval log are restored even
> after a hard reset — both were lost on reset in doc 1.

## 7. Pin map

| Function | Pin |
|----------|-----|
| MODE button | A1 |
| LED / status | D7 |
| Valve direction | D2 |
| Valve SSR power | D6 |
| PIC WAKE (input, PIC drives HIGH) | D10 |
| PIC UART | Serial1 (TX/RX) |
| Battery ADC | A6 (P2) / A5 (Boron) |
| Local hall sensor (optional) | A2 (enable with `USE_LOCAL_METER`) |

## 8. Build

```
# Pick the command for your board:
particle compile p2     leaksense-p2 --target 5.6.0    # Photon 2
particle compile argon  leaksense-p2 --target 6.2.1    # Argon  (Gen3 WiFi)
particle compile boron  leaksense-p2 --target 6.2.1    # Boron  (Gen3 cellular)
particle flash <device> leaksense-p2
```

By default the build needs **no registry downloads**: the only required library
(`JsonParserGeneratorRK`) is vendored in `lib/`, and the IMU is disabled
(`USE_IMU 0`). This is why local Workbench/CLI compiles work even though the local
toolchain does not auto-fetch `project.properties` dependencies.

### Fast bench-test mode (optional, no cloud)

For PIC↔Photon protocol/UART bring-up there is an optional build switch,
`FAST_BENCH_TEST` in `src/app_config.h` (commented out by default). When enabled it
**skips all cloud connect/publish** and simply runs the PIC UART exchange
(`REQ_DATA → RSP_DATA`, valve status), logs it over USB via the existing
`Log.info()` lines, and still ends each session with the one-way
`PKT_PHOTON_OFF_REQ` (func `0x07`, `OFF_REASON_DONE`) so the PIC cuts power and the
cycle repeats — same UART behaviour and same shutdown as production, minus the
~20–40 s cloud connect. Uncomment the `#define FAST_BENCH_TEST 1` line to build it;
leave it commented for the normal cloud build (which is unaffected). The
power-gating / D10-removal changes still apply in this mode. For the cleanest logs
during first bring-up, power the Photon continuously (bypass the P-MOS) so USB does
not re-enumerate each cycle, then switch to real power-gating once verified.

### Enabling the IMU (LSM6DS) — requires vendoring

`Adafruit_LSM6DS` is **not** in the Particle registry, so it cannot be added as a
cloud dependency (that is what produced `Adafruit_LSM6DS33.h: No such file`). To use
the IMU you must vendor the Arduino libraries into `lib/`:

```
lib/
├─ Adafruit_LSM6DS/src/   (Adafruit_LSM6DS33.h, .cpp, Adafruit_LSM6DS.*, ...)
├─ Adafruit_BusIO/src/    (Adafruit_I2CDevice.*, Adafruit_SPIDevice.*, ...)
└─ Adafruit_Sensor/src/   (Adafruit_Sensor.h)
```

Get them from GitHub:
- github.com/adafruit/Adafruit_LSM6DS
- github.com/adafruit/Adafruit_BusIO
- github.com/adafruit/Adafruit_Sensor

Then set `USE_IMU 1` in `src/app_config.h` and rebuild. Note these are Arduino
libraries; `Adafruit_BusIO` is the part most likely to need small tweaks on P2
(RTL872x). The IMU is supplemental telemetry only — the leak / PIC / cloud-config
features do not depend on it.

## 9. Known limitations / items needing hardware verification

- **Not yet HW-tested**: the code now compiles for Argon/P2/Boron with the IMU off,
  but on-device behavior still needs verification. A possible first-build tweak is the
  `Serial1` buffer hook (`acquireSerial1Buffer`) on your Device OS version.
- **IMU is OFF by default** (`USE_IMU 0`) because its library is not in the registry
  (see section 8). Vendor + enable it if you need temp/accel/gyro telemetry.
- **PIC firmware must speak V040**: the P2 now sends framed `REQ_*` packets and
  expects framed `RSP_*` replies with CRC-16. A PIC still on the old raw
  `0xAA → COUNT+samples` firmware will not interoperate — flash the V040 PIC build.
- **Time alignment is assumption-based** (section 5) — sending a timestamp from the PIC
  makes it exact.
- **Local hall-sensor path** is disabled by default (`USE_LOCAL_METER 0`), assuming the
  PIC is the flow source. Set it to 1 to run both, but watch for double counting.
- **Platforms**: P2 / Argon / Boron are all supported via `PLATFORM_ID` guards.
  `backupRamSync()` runs on P2 only (Gen3 persists retained memory automatically).
  Battery reads from the fuel gauge on Argon/Boron and from the ADC divider on P2.
  Kevin verifies the cellular (Boron) build on real hardware.

## 10. Cloud function summary

| Function | Argument | Action |
|----------|----------|--------|
| `setConfig` | `"leakGpm,vol,auto,alert"` | Set 4 **host** analytics vars (not sent to PIC) |
| `getConfig` | (ignored) | Publish current config |
| `setLeakParams` | `"l1cnt,l1win,l2cnt,l2win"` | Read-modify-write the PIC's 4 leak params (`REQ_SET_PARAM`) |
| `getLeakParams` | (ignored) | Refresh leak params from the PIC + publish (`REQ_GET_PARAM`) |
| `getValve` | (ignored) | Read PIC valve status + publish (`REQ_GET_VALVE`) |
| `unlockValve` | `temp`/`perm`/`both` | Clear PIC valve lock (`REQ_VALVE_UNLOCK`) |
| `picReset` | (ignored) | `PKT_SYS_RESET` the PIC (clears the permanent lock) |
| `syncPic` | (ignored) | Force-push the cached leak params now (`REQ_SET_PARAM`) |
| `shutoffSwitch` | `close`/`open`/`off` | Local SSR valve control (10 s timer) |
| `leakingSwitch` | `on`/`off` | Manual host leak-state toggle/reset |
| `setFlowCal` / `getFlowCal` | scale / — | Flow calibration |
| `setWiFi` / `clearWiFi` | `ssid,pass` / — | (P2 only) WiFi credentials |
