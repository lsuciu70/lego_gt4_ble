#pragma once

#include <cstdint>
#include <string>

namespace LWP3 {

/**
 * @brief Tuning knobs and the hub's MAC address, with compiled-in defaults
 * matching this HAL's validated hardware behavior.
 *
 * loadConfig() looks for an optional override file at a fixed path inside
 * the project (config/gt4.conf); a missing file, or a missing/malformed
 * individual key within it, silently falls back to the default below for
 * that field. Protocol constants (BLE UUIDs, LWP3 port IDs, opcodes) are
 * NOT part of this — those are hardware facts, not configuration, and stay
 * in Lwp3Constants.hpp.
 */
struct Config {
    std::string mac_address = "28:3C:90:9C:82:14";
    // delta=1 confirmed safe when IMU is used WITHOUT drive encoders active
    // at the same time; the two together reliably break the connection.
    uint8_t imu_delta = 1;
    // Steering and throttle are gated independently: hardware testing found
    // a genuinely changing steer value safe up to at least 100Hz, while a
    // genuinely changing throttle value broke the connection by 20Hz
    // (virtual port) or 10Hz (direct writes). See README.md "Drive
    // Architecture" for the full characterization.
    uint64_t steer_rate_limit_ms = 15;
    uint64_t throttle_rate_limit_ms = 200;
    uint64_t keepalive_interval_ms = 1000;
    float epsilon_deg = 3.0f;
    uint32_t stall_max_sweep_ms = 1500;
    uint32_t stall_poll_ms = 50;
    uint32_t stall_window_ms = 200;
    int32_t stall_epsilon_raw = 2;
};

/**
 * @brief Loads config/gt4.conf (key=value, '#' comments) from its fixed
 * location inside the project. Overrides only the fields actually present
 * and parseable; everything else keeps Config's compiled-in default.
 */
Config loadConfig();

}  // namespace LWP3
