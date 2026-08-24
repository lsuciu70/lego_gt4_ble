/**
 * @file Lwp3Gt4.hpp
 * @brief High-Performance HAL for Porsche GT4 (Lego Move Hub 88019).
 * * Target Environment: Ubuntu 24.04 LTS (HP OmniBook 7 AeroNGAI 13)
 * Protocol: LEGO Wireless Protocol v3 (LWP3) over BLE.
 * * Version 7.0: Implements "The Mechanical Truth" - a deterministic
 * control loop with epsilon-matched latency profiling and non-blocking
 * transmission gates.
 */

#pragma once
#include <simpleble/SimpleBLE.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string_view>
#include <thread>
#include <vector>

namespace LWP3 {

using TimestampNs = uint64_t;

/**
 * @brief Represents a single control frame sent to the vehicle.
 * Positive throttle always means "that wheel spins forward", independent of
 * left/right mounting — the HAL corrects for the mirrored motor mounting
 * internally. Set throttle_left == throttle_right to drive straight, as
 * before; independent values enable differential drive (skid-turning,
 * traction/slip response, etc.) in the application layer.
 */
struct Command {
    int32_t steer;           ///< Steering angle in degrees (-100 to 100)
    int32_t throttle_left;   ///< Left wheel motor power percentage (-100 to 100)
    int32_t throttle_right;  ///< Right wheel motor power percentage (-100 to 100)
};

/**
 * @brief Represents the latest state received from the vehicle hardware.
 */
struct Telemetry {
    int32_t steer_pos;         ///< Absolute steering position relative to calibrated center.
    TimestampNs timestamp_ns;  ///< Monotonic timestamp (CLOCK_MONOTONIC_RAW) of ingestion.
};

/**
 * @brief Raw 3-axis reading from the Move Hub's internal accelerometer or gyroscope.
 * Units are the hub's native raw units (uncalibrated) — scale/axis mapping is left
 * to the application layer, consistent with the HAL's zero-control-policy design.
 */
struct ImuSample {
    int16_t x;
    int16_t y;
    int16_t z;
    TimestampNs timestamp_ns;  ///< Monotonic timestamp of ingestion.
};

/**
 * @brief BLE link quality snapshot, from the hub's own RSSI property.
 */
struct LinkStatus {
    int8_t rssi_dbm;           ///< Received Signal Strength Indicator, in dBm (negative).
    TimestampNs timestamp_ns;  ///< Monotonic timestamp of ingestion.
};

/**
 * @brief Cumulative rotation counters from each drive motor's built-in encoder.
 * Raw tick counts (mode 2 / POS on each drive motor port), sign-corrected so
 * that a positive-going count matches "forward" on that wheel, consistent
 * with Command::throttle_left/throttle_right. No distance/velocity scaling
 * is applied — that mapping (ticks per revolution, wheel circumference)
 * belongs in the application layer.
 */
struct DriveEncoders {
    int32_t left_ticks;
    int32_t right_ticks;
    TimestampNs timestamp_ns;  ///< Monotonic timestamp of ingestion (most recent of the two).
};

/**
 * @brief High-fidelity latency metrics for ADAS estimation.
 */
struct LatencyStats {
    float mean_ms;  ///< Average time between Command TX and physical Epsilon-match.
    float p50_ms;   ///< Median response time.
    float p99_ms;   ///< Worst-case jitter boundary (99th percentile).
};

/**
 * @brief Tracking structure for inflight commands used in latency matching.
 */
struct TxRecord {
    int32_t steer_target;  ///< The target position we are waiting for the motor to reach.
    TimestampNs tx_time;   ///< Timestamp when the packet was sent to the D-Bus.
};

class PorscheGt4 {
   public:
    PorscheGt4();
    ~PorscheGt4();

    /**
     * @brief Establishes a BLE connection and initializes the LWP3 handshake.
     * @param address The MAC address of the Move Hub.
     * @return true if connected and virtual ports are established, false otherwise.
     */
    bool connect(std::string_view address);

    /**
     * @brief Disconnects the peripheral and stops the background TX thread.
     */
    void disconnect();

    /**
     * @brief Performs physical discovery of the steering rack's mechanical limits.
     * Finds the hardware center and sets the soft-stop boundaries.
     * @return true if calibration succeeded.
     */
    bool autoCalibrate();

    /**
     * @brief Dispatches a control command using "Latest Wins" atomic semantics.
     * This function is non-blocking and thread-safe.
     * @param cmd The target steering and throttle values.
     */
    void sendCommand(const Command& cmd) noexcept;

    /**
     * @brief Retrieves the most recent telemetry state in a lock-free manner.
     * @return A Telemetry struct containing the latest steering position.
     */
    Telemetry getLatestTelemetry() const noexcept;

    /**
     * @brief Retrieves the latest raw accelerometer sample from the hub's internal IMU.
     */
    ImuSample getAccel() const noexcept;

    /**
     * @brief Retrieves the latest raw gyroscope sample from the hub's internal IMU.
     */
    ImuSample getGyro() const noexcept;

    /**
     * @brief Retrieves the latest BLE link status (RSSI), reported by the hub itself.
     * Intended for the application layer to detect a degrading link before it drops.
     */
    LinkStatus getLinkStatus() const noexcept;

    /**
     * @brief Retrieves the latest cumulative encoder ticks from both drive motors.
     */
    DriveEncoders getDriveEncoders() const noexcept;

    /**
     * @brief Calculates real-time latency statistics using Epsilon-Matching.
     * This provides a model of physical system response, not just transport time.
     * @return Statistical report of system lag.
     */
    LatencyStats getLatencyStats();

    /**
     * @brief Checks if the system is ready for ADAS-level control.
     * Requires valid calibration and established virtual drive ports.
     */
    bool isReady() const noexcept;

   private:
    // Core BLE Infrastructure
    mutable SimpleBLE::Peripheral porsche;
    std::jthread _txThread;
    std::atomic<bool> _running{false};
    std::atomic<bool> _isCalibrated{false};
    std::condition_variable _txCv;
    std::mutex _txMtx;

    // Control State (Atomic Latest Semantics)
    std::atomic<Command> _latestCmd{Command{0, 0, 0}};
    std::atomic<bool> _hasNewCmd{false};
    std::atomic<Telemetry> _telemetryLatch{Telemetry{}};
    std::atomic<ImuSample> _accelLatch{ImuSample{}};
    std::atomic<ImuSample> _gyroLatch{ImuSample{}};
    std::atomic<LinkStatus> _linkStatus{LinkStatus{}};
    std::atomic<DriveEncoders> _driveEncoders{DriveEncoders{}};

    // Internal Telemetry State
    std::atomic<int32_t> _rawSteerPos{0};
    std::atomic<bool> _telemetryActive{false};
    std::atomic<uint8_t> _virtualDrivePort{0xFF};

    // Requirement 4: Epsilon Latency Matching
    // Logic: Only records latency if |current_pos - target_pos| < _epsilon.
    std::mutex _statsMtx;
    std::vector<TxRecord> _inflight;
    std::vector<float> _latencySamples;
    const float _epsilon = 3.0f;  // Degrees of mechanical play/slop.

    // Requirement 1: Non-blocking Rate Gate
    // Logic: Ensures D-Bus stability without sleeping the TX thread.
    std::atomic<TimestampNs> _lastTxTime{0};
    const uint64_t _txRateLimitNs = 15'000'000;  // 15ms safety gate.

    /** @brief The high-priority TX loop handling dual-packet LWP3 dispatches. */
    void txLoop(std::stop_token st);

    /** @brief Handles Port 52 subscription and Virtual Drive Port (0x61) creation. */
    void setupHandshake();

    /** @brief Ingests raw BLE notifications and performs epsilon-matching. */
    void updateTelemetry(int32_t pos, TimestampNs now);

    /** @brief Utility for high-precision monotonic timing. */
    TimestampNs getNowNs() const;

    // Packet Builders
    SimpleBLE::ByteArray buildSteerCmd(int32_t abs_angle);
    SimpleBLE::ByteArray buildDriveCmd(int8_t left, int8_t right);

    /** @brief Moves the steering rack until a physical stall is detected. */
    int32_t sweep_to_limit(int8_t speed);

    /** @brief Blocks until a command is written to ensure calibration stability. */
    void sendReliable(const SimpleBLE::ByteArray& data);

    int32_t hardwareCenter = 0;
};

}  // namespace LWP3
