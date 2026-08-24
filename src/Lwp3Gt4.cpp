#include "Lwp3Gt4.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>

#include "Lwp3Constants.hpp"

using namespace std::chrono_literals;

namespace LWP3 {

PorscheGt4::PorscheGt4() {
    _config = loadConfig();
    _epsilon = _config.epsilon_deg;
    _txRateLimitNs = _config.tx_rate_limit_ms * 1'000'000ULL;
    _keepaliveIntervalNs = _config.keepalive_interval_ms * 1'000'000ULL;
    _latencySamples.reserve(100);
}

PorscheGt4::~PorscheGt4() {
    disconnect();
}

TimestampNs PorscheGt4::getNowNs() const {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<TimestampNs>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

bool PorscheGt4::connect(std::string_view address) {
    auto adapters = SimpleBLE::Adapter::get_adapters();
    if (adapters.empty()) return false;
    auto adapter = adapters[0];
    adapter.scan_for(1000);
    auto results = adapter.scan_get_results();

    bool found = false;
    for (auto& p : results) {
        if (p.address() == address) {
            porsche = p;
            found = true;
            break;
        }
    }

    if (!found) return false;

    try {
        porsche.connect();
        std::this_thread::sleep_for(1s);
        setupHandshake();
    } catch (...) {
        try {
            porsche.disconnect();
        } catch (...) {
        }
        return false;
    }

    _running = true;
    _connected.store(true);
    _txLoopExited.store(false);
    _txThread = std::jthread([this](std::stop_token st) { txLoop(st); });
    return true;
}

void PorscheGt4::setupHandshake() {
    porsche.notify(SERVICE_UUID, CHAR_UUID, [this](SimpleBLE::ByteArray data) {
        auto* raw = reinterpret_cast<const uint8_t*>(data.data());
        if (data.size() >= 8 && raw[2] == 0x45 && raw[3] == PORT_STEER) {
            int32_t val;
            std::memcpy(&val, &raw[4], sizeof(int32_t));
            updateTelemetry(val, getNowNs());
        }
        if (data.size() >= 10 && raw[2] == 0x45 && raw[3] == PORT_ACCEL) {
            ImuSample s;
            std::memcpy(&s.x, &raw[4], sizeof(int16_t));
            std::memcpy(&s.y, &raw[6], sizeof(int16_t));
            std::memcpy(&s.z, &raw[8], sizeof(int16_t));
            s.timestamp_ns = getNowNs();
            _accelLatch.store(s);
        }
        if (data.size() >= 10 && raw[2] == 0x45 && raw[3] == PORT_GYRO) {
            ImuSample s;
            std::memcpy(&s.x, &raw[4], sizeof(int16_t));
            std::memcpy(&s.y, &raw[6], sizeof(int16_t));
            std::memcpy(&s.z, &raw[8], sizeof(int16_t));
            s.timestamp_ns = getNowNs();
            _gyroLatch.store(s);
        }
        if (data.size() >= 6 && raw[2] == 0x01 && raw[3] == HUB_PROP_RSSI &&
            raw[4] == HUB_PROP_OP_UPDATE) {
            _linkStatus.store({static_cast<int8_t>(raw[5]), getNowNs()});
        }
        if (data.size() >= 8 && raw[2] == 0x45 && raw[3] == PORT_DRIVE_L) {
            int32_t val;
            std::memcpy(&val, &raw[4], sizeof(int32_t));
            // Left channel command is sign-flipped for the mirrored mount (see
            // buildDriveCmd/buildSingleDriveCmd); flip the encoder reading back
            // so a positive-going count means "forward" on this wheel too,
            // matching the right side.
            auto prev = _driveEncoders.load();
            _driveEncoders.store({-val, prev.right_ticks, getNowNs()});
        }
        if (data.size() >= 8 && raw[2] == 0x45 && raw[3] == PORT_DRIVE_R) {
            int32_t val;
            std::memcpy(&val, &raw[4], sizeof(int32_t));
            auto prev = _driveEncoders.load();
            _driveEncoders.store({prev.left_ticks, val, getNowNs()});
        }
        if (data.size() >= 5 && raw[2] == 0x04 && raw[4] == 0x02) {
            _virtualDrivePort.store(raw[3]);
        }
    });

    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x0A, 0x00, 0x41, PORT_STEER, 0x02, 0x01, 0x00, 0x00, 0x00, 0x01});
    std::this_thread::sleep_for(200ms);
    // Accel/gyro, RSSI, and drive encoders are NOT subscribed here — only
    // steering (needed for calibration/closed-loop control) and the virtual
    // drive port (needed for buildDriveCmd()) are mandatory. Everything else
    // is opt-in via enableImu()/enableLinkStatus()/enableDriveEncoders(), so
    // the application only pays for the telemetry it actually needs.
    porsche.write_command(SERVICE_UUID, CHAR_UUID, {0x06, 0x00, 0x61, 0x01, 0x32, 0x33});
    for (int i = 0; i < 20 && _virtualDrivePort.load() == 0xFF; ++i)
        std::this_thread::sleep_for(100ms);
}

void PorscheGt4::txLoop(std::stop_token st) {
    // Send-on-change + low-rate keepalive. NOT a fixed high-frequency
    // retransmit — see the comment on _txRateLimitNs/_keepaliveIntervalNs in
    // the header and client_handout_ble.md section 9a. `lastSent` is plain
    // (non-atomic) local state: only this thread ever reads or writes it.
    Command lastSent{0, 0, 0};
    bool hasSentOnce = false;   // true after the first SUCCESSFUL transmission (change baseline)
    bool hasAttempted = false;  // true after the first attempt, success or failure (rate-floor gate)

    while (!st.stop_requested()) {
        std::unique_lock lock(_txMtx);
        _txCv.wait_for(lock, 100ms, [&] { return _hasNewCmd.load() || st.stop_requested(); });
        if (st.stop_requested()) break;

        TimestampNs now = getNowNs();
        Command target = _latestCmd.load();
        _hasNewCmd.store(false);
        lock.unlock();

        if (!_isCalibrated.load() || _virtualDrivePort.load() == 0xFF) continue;

        bool changed = !hasSentOnce || target.steer != lastSent.steer ||
                       target.throttle_left != lastSent.throttle_left ||
                       target.throttle_right != lastSent.throttle_right;
        bool keepaliveDue = (now - _lastTxTime.load()) >= _keepaliveIntervalNs;
        if (!changed && !keepaliveDue) continue;
        // Safety floor even if the command is changing every tick (e.g. a
        // continuous steering correction) — see header comment: the safe
        // ceiling for that case is untested, this is a conservative guess.
        // Gated on hasAttempted (not hasSentOnce) so this floor also applies
        // while every attempt is failing (e.g. a dropped connection) —
        // otherwise a run of failures never sets hasSentOnce, "changed"
        // stays permanently true, and this check would never fire.
        if (hasAttempted && (now - _lastTxTime.load()) < _txRateLimitNs) continue;

        // Record the attempt time (not just successes) BEFORE writing, so a
        // failing connection still gets throttled by the floor/keepalive
        // gates above instead of retrying on every subsequent wake — a
        // stale _lastTxTime after a failed write previously reopened both
        // gates immediately, causing a ~50Hz retry storm during an outage.
        _lastTxTime.store(now);
        hasAttempted = true;

        try {
            auto left = static_cast<int8_t>(target.throttle_left);
            auto right = static_cast<int8_t>(target.throttle_right);

            if (left == right) {
                // Symmetric: single atomic packet via the virtual port.
                porsche.write_command(SERVICE_UUID, CHAR_UUID, buildDriveCmd(left, right));
            } else {
                // Differential: direct writes, but ALWAYS both motors
                // together, even though only one value may have changed.
                // Confirmed on hardware: writing just one physical motor
                // port while the pair is still grouped under the virtual
                // port (created at connect(), see setupHandshake()) stops
                // the untouched motor too, as if the hub treats a lone
                // write as breaking synchronization within the group.
                porsche.write_command(SERVICE_UUID, CHAR_UUID,
                                      buildSingleDriveCmd(PORT_DRIVE_L, static_cast<int8_t>(-left)));
                porsche.write_command(SERVICE_UUID, CHAR_UUID,
                                      buildSingleDriveCmd(PORT_DRIVE_R, right));
            }
            porsche.write_command(SERVICE_UUID, CHAR_UUID,
                                  buildSteerCmd(hardwareCenter + target.steer));

            lastSent = target;
            hasSentOnce = true;

            // Record for Epsilon Matching
            std::lock_guard<std::mutex> sLock(_statsMtx);
            _inflight.push_back({target.steer, now});
            if (_inflight.size() > 20) _inflight.erase(_inflight.begin());
        } catch (const std::exception& e) {
            std::cerr << "[txLoop] write_command threw: " << e.what() << "\n" << std::flush;
        } catch (...) {
            std::cerr << "[txLoop] write_command threw non-std exception\n" << std::flush;
        }
    }

    // Signal actual exit, independent of stop_token — disconnect() waits on
    // this (bounded) rather than joining blind, since an in-flight
    // write_command() above can block indefinitely on a stuck D-Bus call.
    {
        std::lock_guard<std::mutex> exitLock(_txExitMtx);
        _txLoopExited.store(true);
    }
    _txExitCv.notify_all();
}

void PorscheGt4::updateTelemetry(int32_t pos, TimestampNs now) {
    int32_t rel_pos = pos - hardwareCenter;
    _rawSteerPos.store(pos);
    _telemetryLatch.store({rel_pos, now});
    _telemetryActive = true;

    // Requirement 4: Epsilon Matching Rule
    // Match current position against the history of targets
    std::lock_guard<std::mutex> lock(_statsMtx);
    for (auto it = _inflight.begin(); it != _inflight.end();) {
        if (std::abs(rel_pos - it->steer_target) < _epsilon) {
            float lat_ms = static_cast<float>(now - it->tx_time) / 1'000'000.0f;
            if (_latencySamples.size() >= 100) _latencySamples.erase(_latencySamples.begin());
            _latencySamples.push_back(lat_ms);
            it = _inflight.erase(it);  // Match found, remove from inflight
        } else if (now - it->tx_time > 500'000'000) {
            it = _inflight.erase(it);  // Clean stale commands (>500ms)
        } else {
            ++it;
        }
    }
}

void PorscheGt4::sendCommand(const Command& cmd) noexcept {
    _latestCmd.store(cmd);
    _hasNewCmd.store(true);
    _txCv.notify_one();
}

LatencyStats PorscheGt4::getLatencyStats() {
    std::lock_guard<std::mutex> lock(_statsMtx);
    if (_latencySamples.empty()) return {0.0f, 0.0f, 0.0f};

    std::vector<float> sorted = _latencySamples;
    std::sort(sorted.begin(), sorted.end());

    float sum = 0;
    for (float v : sorted) sum += v;

    return {sum / static_cast<float>(sorted.size()), sorted[sorted.size() / 2],
            sorted[static_cast<size_t>(sorted.size() * 0.99f)]};
}

SimpleBLE::ByteArray PorscheGt4::buildDriveCmd(int8_t left, int8_t right) {
    // Subcommand 0x02: Start Speed for the virtual port (Speed1, Speed2,
    // MaxPower) — one atomic packet, both wheels together, closed-loop.
    // Only used for symmetric commands (left == right); see txLoop(). Was
    // previously believed unsafe for ANY use based on tests at ~50Hz
    // retransmission; re-validated as safe, including switching to/from
    // buildSingleDriveCmd() mid-session, once transmission is send-on-change
    // rather than a fixed high-frequency retransmit. See
    // client_handout_ble.md section 9a for the full history.
    int8_t inv = static_cast<int8_t>(-left);
    std::array<uint8_t, 9> buf = {0x09, 0x00, 0x81, _virtualDrivePort.load(),
                                   0x11, 0x02,
                                   static_cast<uint8_t>(inv),    // Speed1 — left motor
                                   static_cast<uint8_t>(right),  // Speed2 — right motor
                                   100};                          // MaxPower
    return SimpleBLE::ByteArray(reinterpret_cast<const char*>(buf.data()), 9);
}

SimpleBLE::ByteArray PorscheGt4::buildSingleDriveCmd(uint8_t port, int8_t power) {
    // Subcommand 0x01: StartPower for a single physical motor port (same
    // format already used for the steering sweep in sweep_to_limit()). Used
    // only for differential commands (left != right); see txLoop() and
    // client_handout_ble.md section 9a for why both ports are always
    // written together even though only one value may have changed.
    std::array<uint8_t, 7> buf = {0x07, 0x00, 0x81, port, 0x11, 0x01,
                                   static_cast<uint8_t>(power)};
    return SimpleBLE::ByteArray(reinterpret_cast<const char*>(buf.data()), 7);
}

SimpleBLE::ByteArray PorscheGt4::buildSteerCmd(int32_t abs_angle) {
    std::array<uint8_t, 14> buf = {0x0E, 0x00, 0x81, 0x34, 0x11, 0x0D};
    std::memcpy(&buf[6], &abs_angle, sizeof(int32_t));
    buf[10] = 50;
    buf[11] = 100;
    buf[12] = 0x7E;
    buf[13] = 0x03;
    return SimpleBLE::ByteArray(reinterpret_cast<const char*>(buf.data()), 14);
}

bool PorscheGt4::autoCalibrate() {
    for (int i = 0; i < 30 && !_telemetryActive.load(); i++) std::this_thread::sleep_for(100ms);
    if (!_telemetryActive.load()) return false;

    try {
        int32_t init = _rawSteerPos.load();
        int32_t rawL = sweep_to_limit(-20);
        sendReliable(buildSteerCmd(init));
        int32_t rawR = sweep_to_limit(20);

        hardwareCenter = (rawL + rawR) / 2;
        sendReliable(buildSteerCmd(hardwareCenter));
    } catch (...) {
        return false;
    }

    _isCalibrated.store(true);
    return true;
}

int32_t PorscheGt4::sweep_to_limit(int8_t speed) {
    // Stall detection instead of a fixed sweep duration: friction differs
    // between the two directions (measured ~18% more raw travel in 1s one
    // way than the other), so a fixed timer doesn't reach the true
    // mechanical limit symmetrically and biases the computed center.
    // Cutting power as soon as the rack stops moving is also *gentler* on
    // the gears than the old fixed-1s sweep, which kept pushing for the
    // full second even after an early stall.
    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x07, 0x00, 0x81, 0x34, 0x11, 0x01, static_cast<uint8_t>(speed)});

    const auto kMaxSweepDuration = std::chrono::milliseconds(_config.stall_max_sweep_ms);  // hard safety ceiling
    const auto kPollInterval = std::chrono::milliseconds(_config.stall_poll_ms);
    const auto kStallWindow = std::chrono::milliseconds(_config.stall_window_ms);  // no meaningful movement for this long => stalled
    const int32_t kStallEpsilonRaw = _config.stall_epsilon_raw;

    std::this_thread::sleep_for(150ms);  // let it get moving before checking for stall
    auto start = std::chrono::steady_clock::now();
    int32_t lastPos = _rawSteerPos.load();
    auto lastMoveTime = start;

    while (std::chrono::steady_clock::now() - start < kMaxSweepDuration) {
        std::this_thread::sleep_for(kPollInterval);
        int32_t pos = _rawSteerPos.load();
        if (std::abs(pos - lastPos) > kStallEpsilonRaw) {
            lastPos = pos;
            lastMoveTime = std::chrono::steady_clock::now();
        } else if (std::chrono::steady_clock::now() - lastMoveTime > kStallWindow) {
            break;  // stalled against the mechanical limit
        }
    }

    porsche.write_command(SERVICE_UUID, CHAR_UUID, {0x07, 0x00, 0x81, 0x34, 0x11, 0x01, 0x00});
    std::this_thread::sleep_for(200ms);  // settle (measured to barely matter, but cheap)
    return _rawSteerPos.load();
}

void PorscheGt4::sendReliable(const SimpleBLE::ByteArray& data) {
    porsche.write_command(SERVICE_UUID, CHAR_UUID, data);
    std::this_thread::sleep_for(500ms);
}

Telemetry PorscheGt4::getLatestTelemetry() const noexcept {
    return _telemetryLatch.load();
}
bool PorscheGt4::enableImu() {
    // Delta interval = 10 (raw units), not 1. LWP3's PORT_INPUT_FORMAT_SETUP
    // delta interval is the minimum raw-value change before the hub sends a
    // new notification. delta=1 (report on any change) measured ~95Hz/~56Hz
    // while the vehicle was moving; delta=10 measured ~16Hz/~5Hz in a
    // passive (non-driving) hand-rotation test, with the reported values
    // still tracking real motion.
    //
    // delta=10 is better than delta=1 but NOT a confirmed fix: a follow-up
    // test driving (hybrid virtual/direct, send-on-change TX) WITH IMU
    // enabled at this delta still stalled silently partway through — no
    // exception raised, telemetry and encoders simply stopped updating.
    // Root cause not identified (possibly the combined weight of all 5-6
    // simultaneous subscriptions, not IMU rate alone). Treat enableImu() as
    // UNSAFE to rely on during active driving until this is investigated
    // further — see README.md "IMU" section.

    const uint8_t kImuDelta = _config.imu_delta;
    try {
        porsche.write_command(
            SERVICE_UUID, CHAR_UUID,
            {0x0A, 0x00, 0x41, PORT_ACCEL, 0x00, kImuDelta, 0x00, 0x00, 0x00, 0x01});
        std::this_thread::sleep_for(150ms);
        porsche.write_command(
            SERVICE_UUID, CHAR_UUID,
            {0x0A, 0x00, 0x41, PORT_GYRO, 0x00, kImuDelta, 0x00, 0x00, 0x00, 0x01});
        std::this_thread::sleep_for(150ms);
    } catch (...) {
        return false;
    }
    return true;
}
ImuSample PorscheGt4::getAccel() const noexcept {
    return _accelLatch.load();
}
ImuSample PorscheGt4::getGyro() const noexcept {
    return _gyroLatch.load();
}
bool PorscheGt4::enableLinkStatus() {
    try {
        porsche.write_command(SERVICE_UUID, CHAR_UUID,
                              {0x05, 0x00, 0x01, HUB_PROP_RSSI, HUB_PROP_OP_ENABLE_UPDATES});
        std::this_thread::sleep_for(150ms);
    } catch (...) {
        return false;
    }
    return true;
}
LinkStatus PorscheGt4::getLinkStatus() const noexcept {
    return _linkStatus.load();
}
bool PorscheGt4::enableDriveEncoders() {
    // Mode 2 = cumulative rotation counter (POS) on each drive motor's own
    // built-in encoder, confirmed by live probing.
    try {
        porsche.write_command(SERVICE_UUID, CHAR_UUID,
                              {0x0A, 0x00, 0x41, PORT_DRIVE_L, 0x02, 0x01, 0x00, 0x00, 0x00, 0x01});
        std::this_thread::sleep_for(150ms);
        porsche.write_command(SERVICE_UUID, CHAR_UUID,
                              {0x0A, 0x00, 0x41, PORT_DRIVE_R, 0x02, 0x01, 0x00, 0x00, 0x00, 0x01});
        std::this_thread::sleep_for(150ms);
    } catch (...) {
        return false;
    }
    return true;
}
DriveEncoders PorscheGt4::getDriveEncoders() const noexcept {
    return _driveEncoders.load();
}
bool PorscheGt4::isReady() const noexcept {
    return _isCalibrated.load() && _virtualDrivePort.load() != 0xFF;
}
void PorscheGt4::disconnect() {
    // Idempotent: a second call (e.g. the destructor calling disconnect()
    // again after the application already called it explicitly) is a cheap
    // no-op instead of repeating a slow porsche.disconnect() call for
    // nothing. Measured on hardware: porsche.disconnect() alone can take
    // ~2-3s even on a clean connection (BlueZ/D-Bus, not our code); paying
    // that twice for the same disconnect is pure waste.
    if (!_connected.exchange(false)) return;

    _running = false;
    bool txThreadStuck = false;

    // Stop and join the TX thread before touching `porsche`, so the background
    // thread can't race with disconnect()/a subsequent connect() re-assigning it.
    if (_txThread.joinable()) {
        _txThread.request_stop();
        _txCv.notify_one();

        // join() has no timeout, and an in-flight write_command() inside
        // txLoop() can block indefinitely on a stuck D-Bus/BlueZ call
        // (observed on hardware, not caused by our TX logic). Wait for
        // txLoop's own exit signal — independent of stop_token — with a
        // bounded timeout instead of joining blind.
        std::unique_lock<std::mutex> exitLock(_txExitMtx);
        bool exited = _txExitCv.wait_for(exitLock, 3s, [this] { return _txLoopExited.load(); });
        exitLock.unlock();

        if (exited) {
            _txThread.join();
        } else {
            // Presumed stuck inside a blocking write_command(). Detach
            // rather than hang disconnect() forever: std::jthread::detach()
            // (not a raw pthread_detach) so the jthread object's own state
            // is updated and a later connect()/destructor won't try to
            // join it again. The OS thread keeps running and still
            // touches `porsche`/`this` if it ever unblocks — an accepted
            // residual risk given the alternative is an indefinite hang.
            // See client_handout_ble.md for the full rationale.
            std::cerr << "[disconnect] TX thread did not exit within timeout; detaching.\n"
                      << std::flush;
            _txThread.detach();
            txThreadStuck = true;
        }
    }

    // Skip porsche.disconnect() if the TX thread is presumed stuck: calling
    // it concurrently from this thread while the detached one may still be
    // mid-call on the same SimpleBLE::Peripheral object would be a new,
    // unrelated hazard on top of the already-accepted risk above.
    if (!txThreadStuck) {
        try {
            porsche.disconnect();
        } catch (...) {
        }
    }

    // A fresh connect() always requires a fresh calibration: the physical
    // rig (or the hub) may have changed between sessions.
    _isCalibrated.store(false);
    _virtualDrivePort.store(0xFF);
    _telemetryActive.store(false);
    hardwareCenter = 0;
}

}  // namespace LWP3
