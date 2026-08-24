#include "Lwp3Config.hpp"

#include <fstream>
#include <string>

// Baked in by CMake as an absolute path (see CMakeLists.txt), so the file
// is found regardless of the working directory the binary is run from.
// This fallback only matters if the target is built outside that CMake
// setup.
#ifndef GT4_CONFIG_PATH
#define GT4_CONFIG_PATH "config/gt4.conf"
#endif

namespace LWP3 {

namespace {

std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

}  // namespace

Config loadConfig() {
    Config cfg;  // Compiled-in defaults; the single source of truth for fallback.

    std::ifstream file(GT4_CONFIG_PATH);
    if (!file.is_open()) return cfg;

    std::string line;
    while (std::getline(file, line)) {
        size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);

        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = trim(line.substr(0, eq));
        std::string value = trim(line.substr(eq + 1));
        if (key.empty() || value.empty()) continue;

        try {
            if (key == "mac_address") {
                cfg.mac_address = value;
            } else if (key == "imu_delta") {
                cfg.imu_delta = static_cast<uint8_t>(std::stoul(value));
            } else if (key == "steer_rate_limit_ms") {
                cfg.steer_rate_limit_ms = std::stoull(value);
            } else if (key == "throttle_rate_limit_ms") {
                cfg.throttle_rate_limit_ms = std::stoull(value);
            } else if (key == "keepalive_interval_ms") {
                cfg.keepalive_interval_ms = std::stoull(value);
            } else if (key == "epsilon_deg") {
                cfg.epsilon_deg = std::stof(value);
            } else if (key == "stall_max_sweep_ms") {
                cfg.stall_max_sweep_ms = static_cast<uint32_t>(std::stoul(value));
            } else if (key == "stall_poll_ms") {
                cfg.stall_poll_ms = static_cast<uint32_t>(std::stoul(value));
            } else if (key == "stall_window_ms") {
                cfg.stall_window_ms = static_cast<uint32_t>(std::stoul(value));
            } else if (key == "stall_epsilon_raw") {
                cfg.stall_epsilon_raw = std::stoi(value);
            }
            // Unknown keys are ignored, not an error — forward compatible.
        } catch (...) {
            // Malformed value for a known key: keep the compiled-in default for it.
        }
    }

    return cfg;
}

}  // namespace LWP3
