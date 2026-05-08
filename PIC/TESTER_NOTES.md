# PIC16LF Firmware – Notes for the Tester

Firmware under test: [main.c](main.c)

The firmware meets the functional spec (1 s sampling, 60 s rollup, 180-min ring buffer, gpm × 10 storage, low-power sleep, bit-bang link with CRC-8, local fail-safe valve closure, valve fault watchdog). The items below are **not bugs** — they are unimplemented optional features, calibration placeholders, and configuration assumptions that need to be verified on the bench before sign-off.

## 1. Calibration values are placeholders

Set these to match the actual hardware before any flow testing:

- `PULSES_PER_GALLON` — currently **450**. Replace with the K-factor from the installed flow sensor's datasheet.
- `LEAK_GPM_X10` — currently **1** (0.1 gpm sustained for 10 min).
- `BIG_FLOW_GPM_X10` — currently **80** (8.0 gpm).
- `LEAK_SUSTAIN_MINUTES` — currently **10**.
- `VALVE_CLOSE_SETTLE_MIN` / `VALVE_FAULT_FLOW_MIN` — currently **1 / 3** minutes.

Please confirm thresholds against the product spec (0.1–10 gpm range) and adjust.

## 2. Timer1 clock source — verify on your exact part

`timer1_init_1s()` uses `T1CLK = 0x04`, which selects **LFINTOSC** on common PIC16(L)F18xxx parts. This is required so the 1 s tick keeps running through `SLEEP()`. Please verify against the datasheet for the exact PIC16LF part being used; the `T1CLK` encoding differs across the family. Also confirm the actual LFINTOSC frequency (nominal 31 kHz, but ±15 % typical) and re-tune `TMR1H/TMR1L` preload if 1 s accuracy matters.

**Test:** with no flow input, confirm `latched_minute_count` updates every ~60 s and the ring buffer fills at the expected rate.

## 3. Brownout flag is not auto-set

`FLAG_BROWNOUT` is defined but never asserted in firmware. If brownout reporting is required:
- Enable BOR via config (`#pragma config BOREN = ON`).
- Add a one-time check of `PCON0bits.nBOR` at startup and set `FLAG_BROWNOUT`.

Currently the test for "did the device experience a brownout" will always read 0.

## 4. No flow-pulse debounce

The IOC ISR counts every rising edge on the flow input. This is fine if the sensor front end is Schmitt-triggered or otherwise clean. If you see spurious counts on the bench (e.g. with long cables or a noisy hall sensor), let the firmware author know — a software debounce is straightforward to add but was deliberately omitted.

## 5. Optional features not implemented

- **`total_lifetime_count`** (mentioned as optional in the spec) is not implemented.
- **Particle rail enable / PMOS power gate** is not wired to a GPIO and not implemented. The PIC currently only signals `PIC_WAKE`; it does not control the Photon's power rail.

## 6. Photon-side protocol must match

The bit-bang link uses a custom protocol. The Photon firmware must implement:

- **PIC → Photon frames:** `START(0xAA), type, len, payload[len], CRC8, END(0x55)`
  CRC-8 polynomial **0x07**, init **0x00**, computed over `type, len, payload[]`.
- **Photon → PIC commands:** fixed 3 bytes: `cmd, arg, CRC8(cmd, arg)`.
  For commands without an argument, send `arg = 0` but still include it in the CRC.
- **Wake handshake:** PIC raises `PIC_WAKE` (RA4). Photon then drives `PIC_CLK` (RA1) to start the bit-bang transfer. PIC will wait up to ~30 s for `PIC_CLK` to go high before timing out (`FLAG_UART_TIMEOUT`).
- **ACK semantics:** the PIC clears the latched ring buffer and `FLAG_BUFFER_FULL` only on receipt of `CMD_ACK`. If the Photon never ACKs, the next minute will trigger another wake attempt.

## 7. ICSP / shared-pin behaviour

Pins **RA0 (DATA / ICSPDAT)** and **RA1 (CLK / ICSPCLK)** are shared between the bit-bang link and the programmer. Firmware leaves them as inputs at boot and observes a 100 ms quiet window before any transmit. Please verify:

- ICSP programming works with the Photon connected (series isolation resistors recommended on the shared nets).
- No drive contention is observed on a scope during the first ~100 ms after reset.

## 8. Bit-bang timeout is loop-count based

`wait_clk_state()` uses a loop counter, not a hardware timer, so its real wall-clock timeout depends on compiler optimisation level. It is only used as a guard during an active session (where the chip is awake anyway), so this is acceptable but worth noting if you change build flags.

## 9. Recommended bench tests

1. **Idle current** with no flow — confirm low-power sleep is being entered (current should drop to ~µA between ticks).
2. **Pulse counting** — inject known frequencies and verify `latched_minute_count` matches `pulses × 10 / PULSES_PER_GALLON`.
3. **Big-flow event** — sustained flow ≥ `BIG_FLOW_GPM_X10` should immediately raise `PIC_WAKE` and send a `MSG_EVENT`.
4. **Leak event** — sustained low flow for ≥ `LEAK_SUSTAIN_MINUTES` minutes should raise `FLAG_LEAK` and (if armed) close the valve.
5. **Buffer full** — after `report_interval_min` minutes, `MSG_MINUTE_REPORT` should be sent.
6. **No-Photon survival** — power up the PIC with the Photon disconnected. Verify the ring buffer keeps filling, leak/big-flow auto-shutoff still operates, and the device recovers cleanly when the Photon reappears.
7. **Valve fault** — physically (or simulated) leave flow on after a CLOSE command; after `VALVE_FAULT_FLOW_MIN` minutes, `valve_state` should advance to `VALVE_FAULT` and `FLAG_VALVE_FAULT` should be set.
8. **Bad command** — send a command with a wrong CRC; PIC should set `FLAG_BAD_CMD` and not actuate the valve.
9. **ICSP re-flash** with Photon powered and connected.
