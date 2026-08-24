#include <cmath>
#include <iostream>
#include <thread>

#include "../include/Lwp3Gt4.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    std::string mac = (argc > 1) ? argv[1] : "28:3C:90:9C:82:14";
    LWP3::PorscheGt4 car;

    if (!car.connect(mac)) return 1;
    car.autoCalibrate();

    std::cout << "Starting Slalom...\n";
    auto start_time = std::chrono::steady_clock::now();

    for (int i = 0; i < 500; ++i) {
        auto next_tick = start_time + (i * 20ms);

        float t_sec = i * 0.02f;
        int target_steer = static_cast<int>(50.0f * std::sin(t_sec * 3.0f));

        car.sendCommand({target_steer, 30, 30});

        if (i % 50 == 0) {
            auto stats = car.getLatencyStats();
            std::cout << "T: " << t_sec << "s | Steer: " << target_steer
                      << " | Latency: " << stats.mean_ms << "ms\n";
        }
        std::this_thread::sleep_until(next_tick);
    }

    car.sendCommand({0, 0, 0});
    std::this_thread::sleep_for(500ms);
    car.disconnect();
    return 0;
}
