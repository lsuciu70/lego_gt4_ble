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
- Raw IMU telemetry (accelerometer + gyroscope, from the hub's internal sensors) — opt-in via `enableImu()`, not subscribed by default; see section 14a
- BLE link status (RSSI) — opt-in via `enableLinkStatus()`, not subscribed by default
- Drive-encoder telemetry (per-wheel rotation ticks) — opt-in via `enableDriveEncoders()`, not subscribed by default
- Independent left/right throttle, including differential/skid-turn motion (verified on hardware; see section 9a for the drive architecture behind it)
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

car.sendCommand({0, 30, 30});

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

1. Find left mechanical stop (stall detection, not a fixed timer)
2. Find right mechanical stop (stall detection, not a fixed timer)
3. Compute center position
4. Move steering to center
5. Store center internally

Each sweep stops as soon as position telemetry stops changing (stalled
against the physical limit), capped at 1.5s as a hard safety ceiling. A
fixed-duration sweep was tried first and found to be unreliable: friction
differs between the two sweep directions (~18% more travel in the same
time on one side than the other in testing), so a fixed timer reaches the
limit asymmetrically and biases the computed center — this was confirmed
visually as a steering rack that stayed noticeably off-center after
calibration.

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
    int32_t throttle_left;
    int32_t throttle_right;
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

## Throttle (throttle_left / throttle_right)

Range:

```text
-100 ... +100
```

Interpretation, per wheel:

```text
-100 = maximum reverse
   0 = stop
+100 = maximum forward
```

The SDK corrects for the mirrored motor mounting internally, so positive
always means "forward" on both sides.

For straight-line driving, set `throttle_left == throttle_right`.

---

# 9a. Drive Architecture: Send-On-Change, Hybrid Virtual/Direct (READ BEFORE MODIFYING)

For symmetric commands (`throttle_left == throttle_right`), the HAL sends
one atomic packet through the LWP3 virtual/combined port. For differential
commands, it sends two direct per-motor writes (`PORT_OUTPUT_COMMAND`, one
per physical motor port), always both together even if only one side
changed. Commands are transmitted on change plus a low-rate keepalive —
see section 12 — not retransmitted at a fixed high frequency.

## History: two real problems that looked like one

Development went through several iterations misdiagnosing what turned out
to be two independent problems, both producing the same external symptom
("BLE connection drops during driving"). Recorded here so the same ground
isn't re-covered:

**Problem 1 — outgoing write rate.** Retransmitting a drive command at a
fixed ~50 Hz (the SDK's originally documented control-loop rate)
reliably dropped the BLE connection within 1-2 seconds on this hardware,
regardless of whether the command value was actually changing. A command
sent once and never repeated, or repeated at ~1 Hz, was reliable across
many repeated hardware tests (15-32s each, including differential
commands and alternating between the virtual port and direct writes
mid-session). Fix: send-on-change + ~1Hz keepalive (section 12). The safe
ceiling for a command that changes faster than ~1 Hz has NOT been
characterized.

**Problem 2 — incoming IMU notification volume**, found independently,
after problem 1 was already fixed and driving still occasionally
destabilized. Accelerometer/gyroscope notifications stream at up to
~95 Hz / ~56 Hz while the vehicle is actually moving (mechanical
vibration) — traffic *from* the hub, unrelated to anything the client
sends. This alone was enough to drop the connection during real driving,
even with problem 1 fixed and every other subscription (steering, both
drive encoders) left enabled. Mitigation: IMU is opt-in (`enableImu()`,
section 14a), not subscribed by `connect()`, and requests a coarser
delta interval than the default. **This is not a confirmed fix** — see
section 14a: a combined drive+IMU test still stalled the connection even
with the mitigation in place. Treat IMU as unsafe during active driving.

**What this means for two earlier conclusions in this document's history,
now corrected:** With only problem 1 fixed (not yet problem 2), testing
looked exactly like "the virtual port is unsafe to mix with direct
writes" — switching between them appeared to leave the hub unresponsive.
That conclusion did not hold up once problem 2 was also fixed and
retested: alternating between the virtual port and direct writes
repeatedly, mid-session, has been reliable. If you see connection drops
during driving again, suspect problem 1 or 2 recurring (or a third,
undiscovered cause) before suspecting the virtual/direct split itself.

## Why direct writes are still used for differential commands

Independent of problems 1 and 2 above: the virtual port's combined
"Speed1/Speed2" packet reliably produces NO wheel movement for
opposite-sign values on this hub. This is a real, reproducible finding,
confirmed with full exception visibility (not a swallowed connection
error) — not yet root-caused. Symmetric values through the virtual port
work correctly. Hence: symmetric → virtual port; differential → direct.

One more hardware finding for the direct-write path: writing to only ONE
physical motor port while the pair is still grouped under the virtual
port (created at connect() regardless — see `setupHandshake()`) stops the
*other*, untouched motor too, as if the hub treats a lone write as
breaking the group's synchronization. So `txLoop()` always writes both
physical ports together on the differential path, even when only one
wheel's value changed.

## Trade-off of the direct-write path

The two writes (left motor, right motor) are sequential BLE packets, not
one atomic packet, so there is a small timing gap between when the left
and right motor receive their target speed. This was audible as a brief
slip/chirp at the start of a hard acceleration in testing. Ramp throttle
rather than commanding it instantaneously if this matters for your
application.

---

# 10. Command Semantics

## Latest Wins

The SDK does not queue commands.

Example:

```cpp
sendCommand({10,0,0});
sendCommand({20,0,0});
sendCommand({30,0,0});
```

Only the latest command is guaranteed to be transmitted.

Result:

```text
Vehicle receives:
{30,0,0}
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

The SDK does NOT retransmit at a fixed high frequency. It uses send-on-
change plus a low-rate keepalive — see section 9a for why (a fixed ~50 Hz
retransmit reliably dropped the BLE connection).

Behavior:

```text
Command changed since last transmit?
  -> sent immediately (bounded by a small floor, currently 50ms,
     even if the value is changing every tick)

Command unchanged since last transmit?
  -> resent at most once per keepalive interval (currently 1s)
```

Clients can still call `sendCommand()` as often as they like (e.g. every
control tick at 50 Hz) — the HAL decides internally whether that actually
produces a BLE transmission. Calling it often with an unchanged value is
cheap and expected; it is NOT the same as commanding a fast BLE write rate.

Validated on hardware:

```text
Command sent once, never repeated           -> reliable (20s test)
Command repeated at ~1 Hz                    -> reliable (20s test)
Command changing at human/decision pace
  (~4s between changes in testing)           -> reliable (32s test)
```

NOT validated:

```text
Command changing continuously faster than ~1 Hz
  (e.g. a closed-loop controller adjusting steering every tick)
```

If your application needs updates faster than ~1 Hz on a value that is
genuinely changing that often (not just being re-sent unchanged), test
that specific pattern before relying on it — see section 9a for the full
background this recommendation comes from.

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

# 14a. Opt-In Telemetry Interfaces (IMU, Link Status, Drive Encoders)

## enableImu()

```cpp
bool enableImu();
```

Subscribes to the hub's accelerometer/gyroscope. **NOT called
automatically by `connect()`.** Call this explicitly, after `connect()`,
only if the application genuinely needs IMU data.

Why opt-in, not automatic: accelerometer/gyroscope notifications stream at
up to ~95 Hz / ~56 Hz (delta interval = 1) while the vehicle is actually
moving (mechanical vibration causes constant value changes). That
notification volume alone was found to destabilize the BLE connection
during real driving — see section 9a, "Problem 2". Steering telemetry
(the only subscription `connect()` sets up unconditionally) did not cause
this even at ~132 Hz measured. Enabling IMU is a deliberate trade-off the
client opts into, not a free capability.

**Mitigation status — NOT a confirmed fix.** `enableImu()` requests a
coarser delta interval (10 instead of 1), which cut the notification rate
to ~16 Hz / ~5 Hz in an isolated, non-driving hand-rotation test. A
follow-up test that combined active driving (hybrid virtual/direct TX,
send-on-change — section 9a) with IMU enabled at this same delta still
stalled the connection partway through, silently: no exception raised,
telemetry and encoders simply stopped updating. Root cause not identified
— possibly the combined weight of ~5-6 simultaneous subscriptions
(steering, accel, gyro, both drive encoders, RSSI) rather than IMU rate
in isolation. **Do not treat `enableImu()` as safe during active
driving.** It is more likely to be safe for passive monitoring (vehicle
stationary, or between drive segments) based on what has actually been
tested.

Returns:

- true if the subscription requests were sent successfully
- false on failure

---

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
Values only update after a successful `enableImu()` call; before that,
these return a zeroed/stale sample.

Units:

Raw, hub-native units. The SDK does NOT scale to g / deg-per-second, and
does NOT remap axes to the vehicle chassis. Calibration and interpretation
are the client's responsibility, same as with steering telemetry.

Semantics:

Same latest-only semantics as `Telemetry` (see sections 13-14): no
history, no queue, newest sample always wins.

---

## enableLinkStatus()

```cpp
bool enableLinkStatus();
```

Subscribes to the hub's RSSI property. **NOT called automatically by
`connect()`.** Call this explicitly, after `connect()`, before relying on
`getLinkStatus()`.

Why opt-in: for interface consistency with `enableImu()` and
`enableDriveEncoders()` — not because RSSI is risky. This signal ran
alongside every driving test in this document's history with no observed
instability.

Returns:

- true if the subscription request was sent successfully
- false on failure

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
Values only update after a successful `enableLinkStatus()` call.

Intended use:

Detect a degrading BLE link (falling RSSI, or `timestamp_ns` not advancing)
and stop the vehicle proactively, before the connection actually drops.
The SDK does not do this automatically — no hidden control policy, same
design principle as everywhere else in this document.

---

## enableDriveEncoders()

```cpp
bool enableDriveEncoders();
```

Subscribes to both drive motors' built-in rotation encoders. **NOT called
automatically by `connect()`.** Call this explicitly, after `connect()`,
before relying on `getDriveEncoders()`.

Why opt-in: for interface consistency with `enableImu()` and
`enableLinkStatus()` — not because drive encoders are risky. This is the
one telemetry stream actually measured (~264 Hz combined, send-on-change
TX, wheels suspended) running cleanly alongside driving with zero
exceptions.

Returns:

- true if the subscription requests were sent successfully
- false on failure

---

## getDriveEncoders()

```cpp
DriveEncoders getDriveEncoders() const;
```

```cpp
struct DriveEncoders
{
    int32_t left_ticks;
    int32_t right_ticks;
    TimestampNs timestamp_ns;
};
```

Source:

Each drive motor's own built-in rotation encoder (LWP3 mode 2 / POS).
Values only update after a successful `enableDriveEncoders()` call.

Sign convention:

Positive-going ticks mean "forward" on that wheel, matching
`Command::throttle_left/throttle_right`.

Units:

Raw encoder ticks. No ticks-per-revolution or wheel-circumference scaling
is applied — that mapping is the client's responsibility. Rough field
calibration measured in testing: ~15 ticks/cm (~1515 ticks/m) at throttle
30, on carpet — a starting point only, expect it to vary with surface,
battery level, and throttle.

Verification status:

Confirmed correct on hardware for both symmetric and differential driving
(see section 9a).

Also confirmed useful diagnostically: in testing, the two drive motors
showed a real ~10-18% speed difference from each other at identical
commanded throttle (mechanical, not a software bug), which explained a
vehicle drifting off a straight line under open-loop symmetric throttle.
A client can use the per-wheel encoder deltas to correct for this in
closed loop.

---

# 14b. Configuration File

Tuning knobs and the hub's MAC address have compiled-in defaults, matching
the values validated on hardware throughout this document's history. They
can optionally be overridden from a plain-text file at a fixed location in
the project, `config/gt4.conf`:

```text
# key=value, one per line. '#' starts a comment.
mac_address=28:3C:90:9C:82:14
imu_delta=10
tx_rate_limit_ms=50
keepalive_interval_ms=1000
epsilon_deg=3.0
stall_max_sweep_ms=1500
stall_poll_ms=50
stall_window_ms=200
stall_epsilon_raw=2
```

The file is entirely optional. A missing file, or a missing/malformed
value for a given key, silently falls back to that field's compiled-in
default — never a hard error. Unknown keys are ignored (forward
compatible with future tunable fields).

**What's excluded on purpose:** protocol constants — BLE UUIDs, LWP3 port
IDs, message opcodes, defined in `Lwp3Constants.hpp` — are hardware facts,
not configuration, and are NOT part of this file. Making them
"configurable" would misleadingly imply they could vary; they can't, for a
fixed hub model.

`PorscheGt4`'s constructor calls `LWP3::loadConfig()` (declared in
`Lwp3Config.hpp`) once and applies the tuning fields internally.
`examples/` and `tests/` also read `mac_address` from the same call for
`connect()`, so the hub's MAC address needs editing in exactly one place,
not once per file.

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
- that the vehicle can always be stopped in software

The vehicle remains a physical system subject to mechanical and wireless limitations.

On that last point specifically: during development, the BLE connection
dropped mid-drive on multiple occasions (root causes: section 9a). Each
time, the hub kept executing the last command it had received —
indefinitely, wheels still spinning — because **the hub has no
command-timeout/watchdog of its own**. There is no software-side way to
stop the vehicle once the connection is actually gone: neither
`disconnect()` nor `sendCommand({0,0,0})` can reach a hub that's no longer
connected. This is a hardware/firmware characteristic of the hub, not
something this SDK can fix, and it is not specific to any one drive
architecture — it will recur if the connection drops for any reason,
known or not. Clients must always be able to cut power to the hub by hand
and must not rely on software-only stop as the sole safety mechanism.

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

Calling `sendCommand()` at 50 Hz is expected and fine even though the HAL
does not transmit at 50 Hz internally — see section 12. The actual BLE
write rate is decided by the HAL (send-on-change + ~1Hz keepalive), not by
how often the client calls `sendCommand()`.

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
