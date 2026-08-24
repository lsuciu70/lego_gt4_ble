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

## IMU (Accelerometer / Gyroscope)

The Move Hub has an internal 3-axis accelerometer and 3-axis gyroscope. The
HAL exposes both as raw samples:

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
belongs in the application layer. Verified on hardware for symmetric
(straight-line) driving; see the Command section above for the differential
caveat.

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

**Known limitation:** symmetric driving (`throttle_left == throttle_right`)
is verified working end-to-end, including drive-encoder feedback.
Genuinely differential values (opposite signs, e.g. for an in-place skid
turn) are accepted by the API and protocol, but on-hardware testing showed
**no wheel movement** in that case — the drive motors' virtual port may
have a firmware-level constraint limiting it to symmetric/mirrored motion.
Not yet root-caused; treat differential throttle as unverified until this
is investigated further (see client_handout_ble.md, section 9a).

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

## getAccel() / getGyro()

```cpp
ImuSample getAccel() const;
ImuSample getGyro() const;
```

Return the latest raw sample from the hub's internal accelerometer /
gyroscope.

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
