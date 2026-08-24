#include <iomanip>
#include <iostream>
#include <thread>

#include "Lwp3Gt4.hpp"

int main(int argc, char** argv) {
    LWP3::PorscheGt4 car;
    if (!car.connect("28:3C:90:9C:82:14")) return 1;
    car.autoCalibrate();

    std::cout << "Profiling physical latency (20 cycles)..." << std::endl;

    for (int i = 0; i < 20; ++i) {
        // Use a slightly smaller target than the max to avoid hitting soft-margins
        int target = (i % 2 == 0) ? 35 : -35;
        car.sendCommand({target, 0, 0});

        // SAFETY: We wait for the car to get CLOSE, then move on.
        // If we wait for perfection, D-Bus will time out.
        int timeout_ticks = 0;
        while (std::abs(car.getLatestTelemetry().steer_pos - target) > 5 && timeout_ticks++ < 100) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    auto stats = car.getLatencyStats();
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\n--- FINAL HAL LATENCY PROFILE ---\n";
    std::cout << "Mean: " << stats.mean_ms << " ms\n";
    std::cout << "P50:  " << stats.p50_ms << " ms\n";
    std::cout << "P99:  " << stats.p99_ms << " ms\n";
    std::cout << "---------------------------------\n";

    car.disconnect();
    return 0;
}