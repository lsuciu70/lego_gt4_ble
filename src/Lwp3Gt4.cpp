#include "Lwp3Gt4.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include "Lwp3Constants.hpp"

using namespace std::chrono_literals;

namespace LWP3 {

PorscheGt4::PorscheGt4() {
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
        if (data.size() >= 5 && raw[2] == 0x04 && raw[4] == 0x02) {
            _virtualDrivePort.store(raw[3]);
        }
    });

    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x0A, 0x00, 0x41, PORT_STEER, 0x02, 0x01, 0x00, 0x00, 0x00, 0x01});
    std::this_thread::sleep_for(200ms);
    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x0A, 0x00, 0x41, PORT_ACCEL, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01});
    std::this_thread::sleep_for(150ms);
    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x0A, 0x00, 0x41, PORT_GYRO, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01});
    std::this_thread::sleep_for(150ms);
    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x05, 0x00, 0x01, HUB_PROP_RSSI, HUB_PROP_OP_ENABLE_UPDATES});
    std::this_thread::sleep_for(150ms);
    porsche.write_command(SERVICE_UUID, CHAR_UUID, {0x06, 0x00, 0x61, 0x01, 0x32, 0x33});
    for (int i = 0; i < 20 && _virtualDrivePort.load() == 0xFF; ++i)
        std::this_thread::sleep_for(100ms);
}

void PorscheGt4::txLoop(std::stop_token st) {
    while (!st.stop_requested()) {
        std::unique_lock lock(_txMtx);
        // Wait for command or timeout (Keepalive)
        _txCv.wait_for(lock, 100ms, [&] { return _hasNewCmd.load() || st.stop_requested(); });
        if (st.stop_requested()) return;

        TimestampNs now = getNowNs();

        // Requirement 1: Non-blocking Rate Gate
        if (now - _lastTxTime.load() < _txRateLimitNs) {
            continue;  // Skip this tick, don't sleep the thread
        }

        Command target = _latestCmd.load();
        _hasNewCmd.store(false);
        lock.unlock();

        if (!_isCalibrated.load() || _virtualDrivePort.load() == 0xFF) continue;

        try {
            porsche.write_command(SERVICE_UUID, CHAR_UUID, buildDriveCmd(target.throttle));
            porsche.write_command(SERVICE_UUID, CHAR_UUID,
                                  buildSteerCmd(hardwareCenter + target.steer));

            _lastTxTime.store(now);

            // Record for Epsilon Matching
            std::lock_guard<std::mutex> sLock(_statsMtx);
            _inflight.push_back({target.steer, now});
            if (_inflight.size() > 20) _inflight.erase(_inflight.begin());
        } catch (...) {
        }
    }
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

SimpleBLE::ByteArray PorscheGt4::buildDriveCmd(int8_t speed) {
    // Subcommand 0x02: Start Speed for virtual port (Speed1, Speed2, MaxPower).
    // PORT_DRIVE_L (0x32) is mounted mirror-image to PORT_DRIVE_R (0x33), so
    // Speed1 (left) must be negated for both wheels to drive in the same direction.
    int8_t inv = static_cast<int8_t>(-speed);
    std::array<uint8_t, 9> buf = {0x09, 0x00, 0x81, _virtualDrivePort.load(),
                                   0x11, 0x02,
                                   static_cast<uint8_t>(inv),    // Speed1 — left motor
                                   static_cast<uint8_t>(speed),  // Speed2 — right motor
                                   100};                          // MaxPower
    return SimpleBLE::ByteArray(reinterpret_cast<const char*>(buf.data()), 9);
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
    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x07, 0x00, 0x81, 0x34, 0x11, 0x01, static_cast<uint8_t>(speed)});
    std::this_thread::sleep_for(1s);
    porsche.write_command(SERVICE_UUID, CHAR_UUID, {0x07, 0x00, 0x81, 0x34, 0x11, 0x01, 0x00});
    return _rawSteerPos.load();
}

void PorscheGt4::sendReliable(const SimpleBLE::ByteArray& data) {
    porsche.write_command(SERVICE_UUID, CHAR_UUID, data);
    std::this_thread::sleep_for(500ms);
}

Telemetry PorscheGt4::getLatestTelemetry() const noexcept {
    return _telemetryLatch.load();
}
ImuSample PorscheGt4::getAccel() const noexcept {
    return _accelLatch.load();
}
ImuSample PorscheGt4::getGyro() const noexcept {
    return _gyroLatch.load();
}
LinkStatus PorscheGt4::getLinkStatus() const noexcept {
    return _linkStatus.load();
}
bool PorscheGt4::isReady() const noexcept {
    return _isCalibrated.load() && _virtualDrivePort.load() != 0xFF;
}
void PorscheGt4::disconnect() {
    _running = false;

    // Stop and join the TX thread before touching `porsche`, so the background
    // thread can't race with disconnect()/a subsequent connect() re-assigning it.
    if (_txThread.joinable()) {
        _txThread.request_stop();
        _txCv.notify_one();
        _txThread.join();
    }

    try {
        porsche.disconnect();
    } catch (...) {
    }

    // A fresh connect() always requires a fresh calibration: the physical
    // rig (or the hub) may have changed between sessions.
    _isCalibrated.store(false);
    _virtualDrivePort.store(0xFF);
    _telemetryActive.store(false);
    hardwareCenter = 0;
}

}  // namespace LWP3
