#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "Lwp3Config.hpp"
#include "Lwp3Gt4.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    LWP3::PorscheGt4 car;
    if (!car.connect(LWP3::loadConfig().mac_address)) return 1;

    std::cout << "Calibrating..." << std::endl;
    car.autoCalibrate();

    // Final check for the Virtual Port before slalom
    if (!car.isReady()) {
        std::cerr << "Hub failed to initialize virtual drive port!" << std::endl;
        return 1;
    }

    std::cout << "Starting Slalom..." << std::endl;
    for (int i = 0; i < 400; ++i) {
        float t = i * 0.02f;
        int32_t steer = static_cast<int32_t>(45.0f * std::sin(t * 3.5f));

        car.sendCommand({steer, 35, 35});
        std::this_thread::sleep_for(20ms);
    }

    car.sendCommand({0, 0, 0});
    std::this_thread::sleep_for(500ms);
    car.disconnect();

    // std::_Exit(0), not return 0: measured on hardware, normal static
    // teardown after disconnect() can add 25+ seconds (SimpleBLE/BlueZ
    // process-exit delay, unrelated to this SDK). See README.md "Known
    // Issue: Slow Process Exit".
    std::_Exit(0);
}
