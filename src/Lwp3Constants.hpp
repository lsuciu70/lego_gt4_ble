#pragma once
#include <cstdint>

namespace LWP3 {
static constexpr const char* SERVICE_UUID = "00001623-1212-efde-1623-785feabcd123";
static constexpr const char* CHAR_UUID = "00001624-1212-efde-1623-785feabcd123";

static constexpr uint8_t PORT_DRIVE_L = 0x32;
static constexpr uint8_t PORT_DRIVE_R = 0x33;
static constexpr uint8_t PORT_STEER = 0x34;

// Internal Move Hub sensor cluster (mode 0 on each, confirmed by live probing).
static constexpr uint8_t PORT_ACCEL = 0x38;  // 3-axis accelerometer.
static constexpr uint8_t PORT_GYRO = 0x39;   // 3-axis gyroscope.

// Hub Properties (message type 0x01) relevant to link/vehicle health.
static constexpr uint8_t HUB_PROP_RSSI = 0x05;
static constexpr uint8_t HUB_PROP_OP_ENABLE_UPDATES = 0x02;
static constexpr uint8_t HUB_PROP_OP_UPDATE = 0x06;
}  // namespace LWP3
