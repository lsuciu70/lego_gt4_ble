// Exercises independent left/right throttle together with the two opt-in
// telemetry streams measured safe alongside driving (drive encoders, RSSI)
// — a combination validated on hardware during development but, until
// now, not covered by any test in this repo. Doubles as a usage example
// for enableDriveEncoders()/enableLinkStatus() plus differential drive.
//
// Deliberately does NOT enable IMU: see README.md "IMU" section — not
// confirmed safe combined with active driving.
//
// First run should be with the wheels suspended (no load), same as every
// other new-combination test in this project's history.

#include <cstdlib>
#include <iostream>
#include <thread>

#include "../include/Lwp3Config.hpp"
#include "../include/Lwp3Gt4.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    std::cout.setf(std::ios::unitbuf);  // flush every line, so output survives a hang/kill
    std::string mac = (argc > 1) ? argv[1] : LWP3::loadConfig().mac_address;

    LWP3::PorscheGt4 car;
    if (!car.connect(mac)) {
        std::cerr << "Failed to connect.\n";
        return 1;
    }

    std::cout << "Calibrating...\n";
    if (!car.autoCalibrate() || !car.isReady()) {
        std::cerr << "Calibration failed or hub not ready.\n";
        return 1;
    }

    if (!car.enableDriveEncoders()) std::cerr << "enableDriveEncoders() failed.\n";
    if (!car.enableLinkStatus()) std::cerr << "enableLinkStatus() failed.\n";

    std::cout << "Driving with differential throttle (left=25, right=35) for 5s...\n";
    auto start = std::chrono::steady_clock::now();
    auto nextStatus = start;
    while (std::chrono::steady_clock::now() - start < 5s) {
        car.sendCommand({0, 25, 35});  // throttle_left != throttle_right => differential path

        auto now = std::chrono::steady_clock::now();
        if (now >= nextStatus) {
            auto enc = car.getDriveEncoders();
            auto link = car.getLinkStatus();
            std::cout << "encoders L=" << enc.left_ticks << " R=" << enc.right_ticks
                      << " | rssi=" << static_cast<int>(link.rssi_dbm) << " dBm\n";
            nextStatus = now + 500ms;
        }
        std::this_thread::sleep_for(20ms);
    }

    car.sendCommand({0, 0, 0});
    std::this_thread::sleep_for(500ms);

    auto stats = car.getLatencyStats();
    std::cout << "\nLatency mean=" << stats.mean_ms << "ms p50=" << stats.p50_ms
              << "ms p99=" << stats.p99_ms << "ms\n";

    car.disconnect();

    // std::_Exit(0), not return 0: measured on hardware, letting `car`'s
    // destructor + normal static teardown run can add 25+ seconds after
    // disconnect() has already fully returned — a SimpleBLE/BlueZ-internal
    // delay on process exit, unrelated to anything this SDK does. See
    // README.md "Known Issue: Slow Process Exit" for the full story.
    std::_Exit(0);
}
