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

The SDK uses a dedicated transmission thread and lock-free state latches.

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

Values are available through a lock-free latch.

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
    int32_t throttle;
};
```

Ranges:

- steer: approximately -100 to +100
- throttle: -100 to +100

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
car.sendCommand({25, 40});
```

Meaning:

- steering = 25
- throttle = 40

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
        car.sendCommand({0, 30});
        std::this_thread::sleep_for(20ms);
    }

    car.sendCommand({0, 0});

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

## Steering telemetry only

The HAL currently exposes steering position telemetry.

It does not expose:

- wheel speed
- vehicle velocity
- IMU data
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
