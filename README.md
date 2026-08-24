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

**Why opt-in:** while the vehicle is actually moving, mechanical vibration
makes the accelerometer/gyroscope report at very high rate (measured up to
~95 Hz / ~56 Hz). That notification volume — traffic *from* the hub, not
commands sent *to* it — was found on hardware to destabilize the BLE
connection during real driving. Subscribing to steering position and both
drive encoders (always on) did not cause this; only accel/gyro did. Call
`enableImu()` only if your application genuinely needs IMU data and can
accept that risk; see "Drive Architecture" below for the full story.

---

## Link Status (RSSI)

The hub reports its own BLE signal strength:

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

## Drive Encoders

Both drive motors have their own built-in rotation encoder:

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

# Public API

## Command

```cpp
struct Command {
    int32_t steer;
    int32_t throttle_left;
    int32_t throttle_right;
};
```

Ranges:

- steer: approximately -100 to +100
- throttle_left / throttle_right: -100 to +100

Positive throttle always means "that wheel spins forward" — the HAL
corrects for the mirrored motor mounting internally. Set
`throttle_left == throttle_right` to drive straight.

For symmetric commands (`throttle_left == throttle_right`), the HAL sends
one atomic packet through the LWP3 virtual/combined port. For differential
commands, it sends two direct per-motor writes, always together even if
only one side changed — see "Drive Architecture" below. Verified on
hardware for both cases, including an in-place skid turn confirmed via
drive-encoder feedback.

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
`_keepaliveIntervalNs` (1s), with a small floor (`_txRateLimitNs`, 50ms)
even if the command is changing every tick. **The true safe ceiling for a
command that changes faster than ~1 Hz has not been characterized** —
only "changes at human/decision-loop pace, ~1x/s or slower" is validated.

**2. Incoming IMU notification volume.** Independently, enabling
accelerometer/gyroscope notifications (see "IMU" above) — which stream at
up to ~95 Hz / ~56 Hz while the vehicle is actually moving, from
mechanical vibration — destabilized the connection even with the write-rate
fix from (1) already in place and even with every other subscription
(steering, both drive encoders) left on. This is traffic *from* the hub,
unrelated to anything the application sends. Fix: IMU is opt-in
(`enableImu()`), not subscribed by `connect()`.

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

## connect()

```cpp
bool connect(std::string_view address);
```

Connects to the LEGO Move Hub and performs protocol initialization.

Returns:

- true on success
- false on failure

---

## disconnect()

```cpp
void disconnect();
```

Stops background processing and disconnects from the hub.

Also clears calibration state (hardware center, virtual drive port,
telemetry). A subsequent `connect()` always requires a fresh
`autoCalibrate()` before `isReady()` becomes true again, even when
reconnecting to the same vehicle.

---

## autoCalibrate()

```cpp
bool autoCalibrate();
```

Performs steering calibration.

This must be called before vehicle operation.

Returns:

- true on success
- false on failure

---

## sendCommand()

```cpp
void sendCommand(const Command& cmd);
```

Thread-safe and non-blocking.

Example:

```cpp
car.sendCommand({25, 40, 40});
```

Meaning:

- steering = 25
- throttle_left = 40
- throttle_right = 40

---

## getLatestTelemetry()

```cpp
Telemetry getLatestTelemetry() const;
```

Returns the latest steering telemetry.

Example:

```cpp
auto t = car.getLatestTelemetry();

std::cout << t.steer_pos << std::endl;
```

---

## enableImu()

```cpp
bool enableImu();
```

Subscribes to the hub's accelerometer/gyroscope. Not called automatically
by `connect()` — see "IMU" above for why. Call after `connect()`, before
relying on `getAccel()`/`getGyro()`.

Returns:

- true if the subscription requests were sent successfully
- false on failure

---

## getAccel() / getGyro()

```cpp
ImuSample getAccel() const;
ImuSample getGyro() const;
```

Return the latest raw sample from the hub's internal accelerometer /
gyroscope. Only updates after a successful `enableImu()` call.

Example:

```cpp
auto a = car.getAccel();

std::cout << a.x << " " << a.y << " " << a.z << std::endl;
```

---

## getLinkStatus()

```cpp
LinkStatus getLinkStatus() const;
```

Returns the latest BLE RSSI reported by the hub.

Example:

```cpp
auto link = car.getLinkStatus();

std::cout << (int)link.rssi_dbm << " dBm" << std::endl;
```

---

## getDriveEncoders()

```cpp
DriveEncoders getDriveEncoders() const;
```

Returns the latest cumulative encoder ticks from both drive motors.

Example:

```cpp
auto enc = car.getDriveEncoders();

std::cout << enc.left_ticks << " " << enc.right_ticks << std::endl;
```

---

## getLatencyStats()

```cpp
LatencyStats getLatencyStats();
```

Returns accumulated latency statistics.

Example:

```cpp
auto stats = car.getLatencyStats();

std::cout << stats.mean_ms << std::endl;
```

---

## isReady()

```cpp
bool isReady() const;
```

Returns true when the vehicle is ready for operation.

Requirements:

- successful BLE connection
- successful initialization
- successful calibration

Becomes false again after `disconnect()`, and stays false after a
reconnect until `autoCalibrate()` succeeds on the new connection.

---

# Quick Start

```cpp
#include "Lwp3Gt4.hpp"

#include <chrono>
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

    return 0;
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
