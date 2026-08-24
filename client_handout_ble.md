# LWP3-GT4-SDK Client Handout

Version: 7.x

Audience:
Software engineers integrating the Porsche GT4 BLE SDK into robotics,
ADAS, autonomy, or control applications.

This document is the operational contract between the SDK and the client.

---

# 1. What This SDK Is

LWP3-GT4-SDK is a Hardware Abstraction Layer (HAL) for the LEGO Technic
Porsche GT4 e-Performance vehicle.

The SDK provides:

- BLE connection management
- LEGO Wireless Protocol v3 (LWP3) handling
- Steering calibration
- Steering commands
- Drive commands
- Steering telemetry
- Raw IMU telemetry (accelerometer + gyroscope, from the hub's internal sensors)
- BLE link status (RSSI)
- Physical latency measurements

The SDK does NOT provide:

- Autonomous driving
- PID controllers
- Path planning
- Obstacle avoidance
- Vision processing
- Sensor fusion
- Vehicle state estimation

These belong in higher software layers.

---

# 2. Supported Platform

Current target platform:

- Ubuntu 24.04 LTS
- BlueZ
- SimpleBLE

The SDK has been developed and validated on Linux.

Operation on Windows or macOS is not guaranteed.

---

# 3. Vehicle Model

The SDK currently supports:

LEGO Technic Porsche GT4 e-Performance (42176)

using

LEGO Move Hub 88019

communicating through

LEGO Wireless Protocol v3 (LWP3)

over Bluetooth Low Energy (BLE).

---

# 4. Lifecycle

A client application must follow this sequence.

```text
Create object
      ↓
connect()
      ↓
autoCalibrate()
      ↓
isReady()
      ↓
sendCommand()
      ↓
getLatestTelemetry()
      ↓
disconnect()
```

Failure to follow this sequence may result in undefined behavior.

---

# 5. Typical Usage

```cpp
LWP3::PorscheGt4 car;

if (!car.connect(address))
{
    return false;
}

if (!car.autoCalibrate())
{
    return false;
}

if (!car.isReady())
{
    return false;
}

car.sendCommand({0, 30});

auto telemetry = car.getLatestTelemetry();

car.disconnect();
```

---

# 6. Connection Model

## connect()

```cpp
bool connect(std::string_view address);
```

Purpose:

- Connect to Move Hub
- Initialize BLE notifications
- Perform protocol handshake
- Create virtual drive port

Success means:

- BLE connection established
- Protocol initialization completed

Failure means:

- Vehicle unavailable
- BLE connection failed
- Handshake failed

---

# 7. Calibration

## autoCalibrate()

```cpp
bool autoCalibrate();
```

Purpose:

Discover actual steering limits.

Process:

1. Find left mechanical stop
2. Find right mechanical stop
3. Compute center position
4. Move steering to center
5. Store center internally

Result:

Steering commands become relative to the true hardware center.

Calibration is required before normal operation.

---

# 8. Ready State

## isReady()

```cpp
bool isReady() const;
```

Returns true when:

- connection established
- virtual drive port available
- calibration completed

Clients should refuse vehicle operation until ready.

---

# 8a. Reconnecting

`disconnect()` clears all session state:

- calibration status
- hardware center
- virtual drive port
- telemetry

This means calibration does NOT carry over across a disconnect/connect
cycle, even if it is the same physical vehicle.

```text
connect()
autoCalibrate()
isReady()          -> true
disconnect()
connect()
isReady()          -> false   (calibration state was reset)
autoCalibrate()    -> required again
isReady()          -> true
```

Clients must call `autoCalibrate()` again after every `connect()`,
including reconnects, before relying on `isReady()`.

This is intentional: the physical rig or the hub may have changed between
sessions, so a stale calibration must never be reused silently.

---

# 9. Command Interface

## Command Structure

```cpp
struct Command
{
    int32_t steer;
    int32_t throttle;
};
```

---

## Steering

Range:

```text
-100 ... +100
```

Interpretation:

```text
-100 = full left
   0 = center
+100 = full right
```

Values outside this range should be considered invalid.

---

## Throttle

Range:

```text
-100 ... +100
```

Interpretation:

```text
-100 = maximum reverse
   0 = stop
+100 = maximum forward
```

---

# 10. Command Semantics

## Latest Wins

The SDK does not queue commands.

Example:

```cpp
sendCommand({10,0});
sendCommand({20,0});
sendCommand({30,0});
```

Only the latest command is guaranteed to be transmitted.

Result:

```text
Vehicle receives:
{30,0}
```

This behavior is intentional.

The SDK is optimized for closed-loop control systems.

---

# 11. Thread Safety

## sendCommand()

Thread-safe.

Can be called from:

- control thread
- autonomy thread
- UI thread

without additional locking.

---

## getLatestTelemetry()

Thread-safe.

Can be called concurrently with sendCommand().

---

# 12. Internal Transmission Rate

The SDK applies an internal transmission gate.

Current limit:

```text
15 ms
```

Equivalent maximum update frequency:

```text
66 Hz
```

Recommended client control loop:

```text
50 Hz
```

Equivalent:

```text
20 ms period
```

This matches BLE behavior and steering dynamics.

---

# 13. Telemetry Interface

## Telemetry Structure

```cpp
struct Telemetry
{
    int32_t steer_pos;
    TimestampNs timestamp_ns;
};
```

---

## steer_pos

Current measured steering position.

Characteristics:

- derived from Move Hub telemetry
- relative to calibrated center
- updated asynchronously

---

## timestamp_ns

Monotonic timestamp.

Characteristics:

- nanoseconds
- monotonic clock
- suitable for latency calculations

Not:

- wall clock time
- UTC time

---

# 14. Telemetry Semantics

The SDK stores only the latest telemetry sample.

The SDK does not maintain history.

Example:

```text
sample1
sample2
sample3
sample4
```

Client reads once.

Result:

```text
sample4
```

The newest sample always replaces older samples.

---

# 14a. IMU and Link Status Interfaces

## getAccel() / getGyro()

```cpp
ImuSample getAccel() const;
ImuSample getGyro() const;
```

```cpp
struct ImuSample
{
    int16_t x;
    int16_t y;
    int16_t z;
    TimestampNs timestamp_ns;
};
```

Source:

The Move Hub has an internal 3-axis accelerometer and 3-axis gyroscope.
The SDK subscribes to both and exposes the raw samples.

Units:

Raw, hub-native units. The SDK does NOT scale to g / deg-per-second, and
does NOT remap axes to the vehicle chassis. Calibration and interpretation
are the client's responsibility, same as with steering telemetry.

Semantics:

Same latest-only semantics as `Telemetry` (see sections 13-14): no
history, no queue, newest sample always wins.

---

## getLinkStatus()

```cpp
LinkStatus getLinkStatus() const;
```

```cpp
struct LinkStatus
{
    int8_t rssi_dbm;
    TimestampNs timestamp_ns;
};
```

Source:

The hub's own RSSI property (LWP3 Hub Properties, continuous updates).

Intended use:

Detect a degrading BLE link (falling RSSI, or `timestamp_ns` not advancing)
and stop the vehicle proactively, before the connection actually drops.
The SDK does not do this automatically — no hidden control policy, same
design principle as everywhere else in this document.

---

# 15. Latency Measurement

## getLatencyStats()

```cpp
LatencyStats getLatencyStats();
```

Returns:

```cpp
struct LatencyStats
{
    float mean_ms;
    float p50_ms;
    float p99_ms;
};
```

---

# 16. What Latency Means

Latency is NOT:

```text
packet sent → packet acknowledged
```

Latency IS:

```text
command transmitted
        ↓
steering physically moves
        ↓
steering reaches target
```

This includes:

- BLE transport
- Hub processing
- Motor response
- Gear train dynamics
- Mechanical slack

The measurement represents actual vehicle response.

---

# 17. Epsilon Matching

The SDK uses a tolerance window.

Conceptually:

```text
abs(actual_position - target_position) < epsilon
```

When this condition becomes true:

- command considered completed
- latency sample recorded

This avoids false latency measurements caused by mechanical backlash.

---

# 18. Expected Performance

Typical measured behavior:

```text
Mean latency: ~94 ms
P50 latency : ~71 ms
P99 latency : ~258 ms
```

These numbers are physical response times.

Actual values depend on:

- battery level
- steering load
- BLE environment
- operating system scheduling

---

# 19. What the SDK Guarantees

The SDK guarantees:

- thread-safe command submission
- latest-wins semantics
- steering calibration
- steering telemetry access
- deterministic command dispatch model
- latency statistics collection

---

# 20. What the SDK Does NOT Guarantee

The SDK does not guarantee:

- command delivery timing
- BLE transport timing
- real-time Linux scheduling
- fixed steering response
- obstacle avoidance
- vehicle safety

The vehicle remains a physical system subject to mechanical and wireless limitations.

---

# 21. Recommended Control Loop

```cpp
while (running)
{
    auto telemetry = car.getLatestTelemetry();

    auto cmd = controller.compute(telemetry);

    car.sendCommand(cmd);

    sleep(20ms);
}
```

Recommended frequency:

```text
50 Hz
```

---

# 22. Common Integration Mistakes

## Mistake 1

Sending commands before calibration.

Wrong:

```cpp
connect();
sendCommand(...);
```

Correct:

```cpp
connect();
autoCalibrate();
sendCommand(...);
```

---

## Mistake 2

Expecting command queuing.

Wrong assumption:

```cpp
Every command will execute.
```

Actual behavior:

```cpp
Only newest command is retained.
```

---

## Mistake 3

Using telemetry as history.

Wrong assumption:

```cpp
getLatestTelemetry() returns all samples.
```

Actual behavior:

```cpp
Returns latest sample only.
```

---

## Mistake 4

Treating latency as BLE latency.

Wrong assumption:

```text
Latency == packet transport time
```

Actual meaning:

```text
Latency == physical steering response time
```

---

## Mistake 5

Assuming calibration persists across a disconnect/connect cycle.

Wrong:

```cpp
car.connect(address);
car.autoCalibrate();
car.disconnect();
...
car.connect(address);
car.sendCommand(...);   // isReady() is false here
```

Correct:

```cpp
car.connect(address);
car.autoCalibrate();
car.disconnect();
...
car.connect(address);
car.autoCalibrate();    // required again, see section 8a
car.sendCommand(...);
```

---

# 23. Intended Use Cases

The SDK is designed for:

- autonomous LEGO vehicles
- robotics experiments
- ADAS research
- latency compensation studies
- steering control research
- vehicle control prototyping

The SDK is the hardware layer.

Higher-level intelligence should be implemented above it.
