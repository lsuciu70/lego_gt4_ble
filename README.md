# LWP3-GT4-SDK

High-performance Hardware Abstraction Layer (HAL) for the LEGO Technic Porsche GT4 e-Performance (42176).

The SDK provides deterministic BLE control, steering calibration, telemetry acquisition, and latency profiling for robotics and autonomous driving experiments.

The goal of this project is to expose a stable and minimal hardware interface that higher-level software (ADAS, autonomy, perception, planning, racing logic) can build upon.

---

## Disclaimer

Porsche®, GT4®, and e-Performance™ are trademarks of Porsche AG.

LEGO® is a trademark of the LEGO Group.

This project is not affiliated with, sponsored by, authorized by, or endorsed by Porsche AG or the LEGO Group.

---

# Design Philosophy

This library is intentionally a HAL.

It is responsible only for:

- BLE communication
- LWP3 protocol handling
- Steering calibration
- Steering control
- Drive control
- Telemetry acquisition
- Latency measurement

It is NOT responsible for:

- Path planning
- Lane keeping
- Obstacle avoidance
- PID control
- State estimation
- Sensor fusion
- Autonomous driving logic

Those belong in higher software layers.

---

# Architecture

The SDK uses a dedicated transmission thread and atomic state latches (lock-free where the platform's `std::atomic` support allows it).

```text
Application
    ↓
sendCommand()
    ↓
Latest-Wins Command Latch
    ↓
TX Thread
    ↓
BLE / LWP3
    ↓
LEGO Move Hub
    ↓
Telemetry Callback
    ↓
Telemetry Latch
    ↓
getLatestTelemetry()
```

The application owns the control loop.

Typical control frequency:

- 50 Hz (20 ms)

---

# Features

## Steering Calibration

Automatic discovery of steering limits.

Calibration sequence:

1. Capture current steering position
2. Sweep left until physical stop
3. Return toward initial position
4. Sweep right until physical stop
5. Compute true center
6. Move steering to center

Soft steering limits are automatically derived from the measured hardware limits.

Each sweep uses stall detection (position telemetry stops changing) rather
than a fixed timer, so both directions reach the true mechanical limit
regardless of friction differences between them — a fixed-duration sweep
was found to under-travel on the lower-friction side, biasing the computed
center. A hard time cap (1.5s) still bounds how long the motor pushes
against the stop.

Calibration is mandatory before vehicle operation.

---

## Deterministic Control

Commands are sent through a dedicated TX thread.

The HAL implements:

- Latest-wins command semantics
- Non-blocking command submission
- Thread-safe operation

If multiple commands are issued rapidly, only the most recent command is transmitted.

---

## Telemetry

The HAL continuously tracks steering position.

Telemetry contains:

```cpp
struct Telemetry {
    int32_t steer_pos;
    TimestampNs timestamp_ns;
};
```

Values are available through an atomic latch (lock-free on platforms/builds where `std::atomic<Telemetry>` is natively lock-free; otherwise backed by libatomic).

---

## IMU (Accelerometer / Gyroscope) — opt-in, not automatic

The Move Hub has an internal 3-axis accelerometer and 3-axis gyroscope. The
HAL can expose both as raw samples, but **`connect()` does not subscribe to
them** — call `enableImu()` explicitly if you want this data.

```cpp
struct ImuSample {
    int16_t x;
    int16_t y;
    int16_t z;
    TimestampNs timestamp_ns;
};
```

Values are the hub's native raw units — the HAL does not scale, filter, or
map axes to the chassis. That interpretation belongs in the application
layer, consistent with the HAL's zero-control-policy design.

**Why opt-in:** kept on-request for interface consistency with the other
telemetry streams, not because it's inherently risky on its own — see
below.

**Confirmed finding: IMU and drive encoders are mutually exclusive.**
`enableImu()` at delta=1 (report on any change — the accelerometer/
gyroscope report at up to ~95 Hz / ~56 Hz from mechanical vibration while
actually driving) is **confirmed safe**, reproduced clean over 32s of
active driving (alternating virtual-port/direct-port commands). The
earlier belief that IMU's own notification rate was the problem (and the
resulting delta=10 "mitigation") turned out to be a wrong diagnosis: that
version still stalled during driving, which was confusing until isolated
properly. **Enabling IMU together with `enableDriveEncoders()` at the same
time reliably breaks the BLE connection within seconds of driving** —
reproduced twice, either as a thrown `"Peripheral is not connected"` or a
silent telemetry freeze. Either one alone (even IMU at delta=1, or
encoders at their ~264 Hz combined measured rate) is safe; the two
together are not. RSSI (`enableLinkStatus()`) does not appear to be a
factor either way — present in both safe combinations, absent in the
unsafe one. **Do not call `enableImu()` and `enableDriveEncoders()` on the
same connection.**

---

## Link Status (RSSI) — opt-in, not automatic

The hub reports its own BLE signal strength, but **`connect()` does not
subscribe to it** — call `enableLinkStatus()` explicitly if you want this
data. (Opt-in for interface consistency with `enableImu()`/
`enableDriveEncoders()`, not because it's risky: this signal has run
alongside every driving test done on this hardware with no observed
instability.)

```cpp
struct LinkStatus {
    int8_t rssi_dbm;
    TimestampNs timestamp_ns;
};
```

Intended use: let the application layer detect a degrading link (falling
RSSI, or a stale `timestamp_ns`) and stop the vehicle proactively, before
the BLE connection actually drops.

---

## Drive Encoders — opt-in, not automatic

Both drive motors have their own built-in rotation encoder, but
**`connect()` does not subscribe to them** — call `enableDriveEncoders()`
explicitly if you want this data. (Opt-in for interface consistency with
`enableImu()`/`enableLinkStatus()`, not because it's risky on its own:
measured at ~264 Hz combined, send-on-change TX, wheels suspended —
running cleanly alongside driving with zero exceptions.)

**Do not combine with `enableImu()`** — the two together reliably break
the BLE connection even though either is safe alone; see "IMU" above.

```cpp
struct DriveEncoders {
    int32_t left_ticks;
    int32_t right_ticks;
    TimestampNs timestamp_ns;
};
```

Cumulative raw tick counts, sign-corrected so a positive-going count means
"forward" on that wheel (matching `Command::throttle_left/right`). No
ticks-per-revolution or wheel-circumference scaling is applied — that
belongs in the application layer. Verified on hardware, including a rough
field calibration: ~15 ticks/cm (~1515 ticks/m) at throttle 30, on carpet.
Treat that figure as a starting point, not a precise constant — it will
vary with surface, battery level, and throttle.

Note: the two drive motors measured a real, mechanical ~10-18% speed
difference from each other at identical commanded throttle in testing
(friction/traction, not a software bug) — expect the vehicle to drift
slightly off a straight line under open-loop symmetric throttle. An
application layer can use these per-wheel encoder deltas to correct for it
in closed loop.

---

## Latency Profiling

The SDK measures physical steering response.

Latency is not measured when a packet is transmitted.

Latency is measured when the steering rack physically reaches the requested target.

This produces a realistic control-system latency measurement that includes:

- BLE transport
- Hub processing
- Motor response
- Gear train backlash
- Mechanical settling

Returned statistics:

```cpp
struct LatencyStats {
    float mean_ms;
    float p50_ms;
    float p99_ms;
};
```

---

# Configuration

Tuning knobs (delta intervals, TX rate/keepalive timing, epsilon-matching
tolerance, stall-detection thresholds) and the hub's MAC address have
compiled-in defaults, matching the values validated on hardware throughout
this SDK's development. They can optionally be overridden from a plain-text
file at a fixed location in the project, `config/gt4.conf`:

```text
# key=value, one per line. '#' starts a comment.
mac_address=28:3C:90:9C:82:14
imu_delta=1
steer_rate_limit_ms=15
throttle_rate_limit_ms=200
keepalive_interval_ms=1000
epsilon_deg=3.0
stall_max_sweep_ms=1500
stall_poll_ms=50
stall_window_ms=200
stall_epsilon_raw=2
```

The file is entirely optional. If it's missing, or a key inside it is
missing or fails to parse, that field silently falls back to its
compiled-in default — never a hard error. Unknown keys are ignored, so
adding new tunable fields later won't break an existing config file.

This deliberately does NOT include protocol constants (BLE UUIDs, LWP3
port IDs, message opcodes, in `Lwp3Constants.hpp`) — those are hardware
facts, not configuration, and stay compile-time.

`PorscheGt4`'s constructor loads this file once (via `LWP3::loadConfig()`
from `Lwp3Config.hpp`) and applies the tuning values internally.
`examples/` and `tests/` also call `LWP3::loadConfig().mac_address`
directly for `connect()`, so the hub's MAC address only needs to be
edited in one place.

---

# Public API

The full interface specification — every method's signature, behavior,
guarantees, units, and the common integration mistakes — lives in
`client_handout_ble.md`, not here, to avoid keeping two references in
sync. This section covers only the drive architecture's design rationale,
since that history explains *why* the API looks the way it does.

For symmetric commands (`throttle_left == throttle_right`), the HAL sends
one atomic packet through the LWP3 virtual/combined port. For differential
commands, it sends two direct per-motor writes, always together even if
only one side changed. Verified on hardware for both cases, including an
in-place skid turn confirmed via drive-encoder feedback.

---

## Drive Architecture: Send-On-Change, Not Fixed-Rate

**Commands are transmitted on change (plus a low-rate keepalive), not
retransmitted at a fixed high frequency.** This is the actual fix behind a
long day of hardware debugging — read on before changing `txLoop()`.

### What actually caused the connection to drop

Two separate, unrelated problems were found and fixed during development,
and it's worth recording both since they looked identical from the
outside ("BLE connection drops during driving"):

**1. Outgoing write rate.** Retransmitting a drive command at a fixed
~50 Hz — the SDK's originally documented control-loop rate — reliably
dropped the BLE connection within 1-2 seconds, regardless of whether the
command value was actually changing. A command sent once and never
repeated, or repeated at ~1 Hz, was reliable over many repeated hardware
tests (15-32s each). Fix: `txLoop()` now sends a command immediately when
it changes, and otherwise repeats the last command at most once per
`_keepaliveIntervalNs` (1s).

**The continuous-change ceiling has since been characterized** by a
dedicated hardware sweep (isolating value-change rate from port-mode
switching), with a clear and initially counter-intuitive result:
steering and throttle have very different safe ceilings, so `txLoop()`
now gates them with **independent floors** instead of one shared one.

- A genuinely changing **steer** value was safe up to at least 100 Hz —
  the top of what was swept; the true ceiling may be higher. Default
  floor: `_steerRateLimitNs`, 15 ms (~67 Hz), comfortably inside that.
- A genuinely changing **throttle** value was far more fragile: safe at
  10 Hz / broke by 20 Hz via the virtual port (symmetric), and safe at
  5 Hz / broke by 10 Hz via direct dual-writes (differential) — roughly
  matching the 2-vs-3-packets-per-write difference between the two paths.
  Default floor: `_throttleRateLimitNs`, 200 ms (5 Hz), at the safe
  boundary for the worse (differential) case.
- **Port-mode switching frequency is NOT the cause** — a control test
  that fixed the virtual/direct switch rate at a known-safe 10 Hz while
  sweeping steer-rate independently broke at the same threshold as
  switching freely, ruling that out. It's specifically how often the
  throttle *value* changes.
- Validated end-to-end: a caller driving both steer and throttle as fast
  as it likes (tested up to 100 Hz of attempted changes, alternating
  symmetric/differential) completed cleanly — the independent floors
  protect the connection even from an aggressive/naive caller.

**2. IMU combined with drive encoders.** Independently, enabling
accelerometer/gyroscope notifications (see "IMU" above) *together with*
drive-encoder notifications — both auto-subscribed at the time this was
found, before either became opt-in — destabilized the connection even
with the write-rate fix from (1) already in place. At the time this
looked like "IMU notification volume" being the culprit on its own, since
disabling IMU alone fixed it; a coarser delta interval was tried as a
mitigation and still stalled, which was confusing until properly isolated
later: **it's specifically IMU + drive encoders together that breaks the
connection — either one alone, even at its most aggressive notification
rate, is safe** (see "IMU" above for the isolation tests). RSSI does not
appear to be a factor. Both are opt-in (`enableImu()` /
`enableDriveEncoders()`), not subscribed by `connect()`, specifically so
an application can choose one or the other but must not enable both.

Earlier in development, (1) alone was misdiagnosed as "the LWP3 virtual
port is unsafe" — differential commands sent through it produced no
movement, and once (2) started causing drops during testing, switching
between the virtual port and direct per-motor writes looked like the
trigger. Neither conclusion held up once (1) and (2) were both fixed and
retested: the virtual port has been reliable, including differential
commands and switching to/from direct writes mid-session, once problems
(1) and (2) above stopped confounding the results.

### Why direct writes are still used for differential commands

The virtual port's combined "Speed1/Speed2" packet reliably produced no
wheel movement for opposite-sign values on this hub (a real, reproducible
finding, unrelated to (1) or (2) above and not yet explained). Symmetric
values through the virtual port work fine. So: symmetric → virtual port
(atomic, no timing gap); differential → two direct per-motor writes.

One more hardware finding for the direct-write path: writing to only ONE
physical motor port while the pair is still grouped under the virtual port
(created at connect() regardless — see `setupHandshake()`) stops the
*other*, untouched motor too, as if the hub treats a lone write as
breaking the group's synchronization. So even though only one wheel's
value may have changed, `txLoop()` always writes both physical ports
together on the differential path.

---

# Quick Start

```cpp
#include "Lwp3Gt4.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>

using namespace std::chrono_literals;

int main() {
    LWP3::PorscheGt4 car;

    if (!car.connect("28:3C:90:9C:82:14")) {
        return 1;
    }

    if (!car.autoCalibrate()) {
        return 1;
    }

    if (!car.isReady()) {
        return 1;
    }

    for (int i = 0; i < 250; ++i) {
        car.sendCommand({0, 30, 30});
        std::this_thread::sleep_for(20ms);
    }

    car.sendCommand({0, 0, 0});

    car.disconnect();
    std::_Exit(0);  // not `return 0` — see "Known Issue: Slow Process Exit" below
}
```

---

# Recommended Application Structure

The SDK is intended to be used inside a fixed-frequency control loop.

Example:

```text
50 Hz loop

read telemetry
      ↓
estimate state
      ↓
compute command
      ↓
sendCommand()
```

A typical autonomy stack built on top of the HAL:

```text
Camera
   ↓
Perception
   ↓
State Estimator
   ↓
Planner
   ↓
Controller
   ↓
LWP3-GT4-SDK
   ↓
Vehicle
```

---

# Performance

Measured on the development platform.

Observed steering response:

- Mean latency: ~94 ms
- Median latency (P50): ~71 ms
- Worst-case latency (P99): ~258 ms

These values represent physical steering response and not merely BLE packet transmission time.

---

# Limitations

## No wheel/vehicle-speed telemetry

The HAL exposes steering position, raw accelerometer/gyroscope samples from
the hub's internal IMU, and BLE link RSSI.

It does not expose:

- wheel speed
- vehicle velocity
- battery data

unless future versions add those capabilities.

---

## Calibration required

Steering commands are not meaningful until calibration has completed.

Always run:

```cpp
car.autoCalibrate();
```

after connecting.

---

## BLE transport

The system depends on:

- BlueZ
- SimpleBLE
- Linux BLE stack behavior

Latency and reliability may vary between adapters and operating systems.

---

## Known Issue: Slow Process Exit

`disconnect()` itself returns promptly (a few seconds at most — measured
`porsche.disconnect()` alone taking ~2-3s on this hardware, which is
BlueZ/D-Bus overhead, not something this HAL adds). But letting the
process exit normally afterward (falling off the end of `main()`, so
`PorscheGt4`'s destructor and the C++ runtime's static teardown both run)
was measured on hardware to add **25+ seconds** on top of that, with no
further output or activity — almost certainly a SimpleBLE-internal
background thread not being torn down cleanly at process exit. This is
unrelated to anything in this HAL's own code (confirmed by instrumenting
every step of `disconnect()` itself: all of it completes in a few seconds,
well before the hang starts).

**Workaround, used in every example/test in this repo:** call
`std::_Exit(0)` (from `<cstdlib>`) immediately after `disconnect()`
returns, instead of `return 0` / falling off `main()`. This skips local
destructors and the C++ runtime's static teardown entirely and terminates
the process immediately — safe here because `disconnect()` has already
released everything that matters (the TX thread and the BLE connection);
nothing meaningful is left to clean up.

Note this is a different mechanism from the TX-thread detach fallback in
`disconnect()` (see "disconnect()" in the API reference) — that one
handles a stuck `write_command()` mid-drive; this one handles a slow
exit path after a completely successful, clean disconnect.

---

## Not a safety system

This project is experimental robotics software.

Do not use it in any application where malfunction could cause injury or property damage.

Concretely, this has already happened during development: when the BLE
connection dropped mid-drive (see "Drive Architecture" above for the two
causes that were found and fixed), the hub kept executing the last command
it had received — indefinitely, with the wheels still spinning — because
**the hub has no command-timeout/watchdog of its own**. There is no
software-side way to stop the vehicle once the connection is actually
gone; a `disconnect()` call or a `sendCommand({0,0,0})` cannot reach a hub
it's no longer connected to. This is a hardware/firmware characteristic of
the hub, not something this HAL can fix. Always be prepared to cut power
by hand; do not rely on software-only stop.

---

# Intended Use

This HAL is designed as the foundation layer for:

- ADAS experiments
- Autonomous LEGO vehicles
- Robotics research
- Control-system development
- State-estimation research
- Latency-compensation research
- Path-planning experiments

The SDK provides hardware access.

Autonomy lives above it.
