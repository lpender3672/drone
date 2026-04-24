#include <Eigen/Dense>
#include <ekf16d.h>
#include <sensor_interface.h>

#include <Arduino.h>
#include "bno055_imu.h"
#include "bno055_mag.h"
#include "bmp280_baro.h"
#include "ublox_gnss.h"

#include "tuned_ekf_params.h"

// Memory diagnostics
extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern char *__brkval;
int freeram() { return (char *)&_heap_end - __brkval; }

// EKF instance - use pointer for deferred initialization
EKF16d* ekf = nullptr;

// Sensor instances
BNO055Imu*   imuSensor;
UbloxGnss*   gnssSensor;
BNO055Mag*   magSensor;
BMP280Baro*  baroSensor;

// Sensor loggers - separate from sensor drivers
sensors::TeensySensorLogger imu_log{"IMU"};
sensors::TeensySensorLogger mag_log{"Mag"};
sensors::TeensySensorLogger baro_log{"Baro"};
sensors::TeensySensorLogger gnss_log{"GNSS"};

struct SensorEntry {
    sensors::ISensor*  sensor;
    sensors::ILogger*  logger;
};
constexpr size_t NUM_SENSORS = 3;
SensorEntry sensor_entries[NUM_SENSORS];

void ekfDebug(const char* label) {
    Serial.printf("%s - free: %d\n", label, freeram());
    Serial.flush();
}

void printTimings() {
    static uint32_t last_print = 0;
    if (millis() - last_print < 1000) return;
    const uint32_t now_ms = millis();
    const uint32_t elapsed_ms = now_ms - last_print;
    last_print = now_ms;
    const float window_s = (elapsed_ms > 0) ? (elapsed_ms * 0.001f) : 1.0f;

    Serial.println("--- Sensor Timings (us) / Rate (Hz) ---");

    // IMU
    const uint32_t imu_n = imuSensor->consumeUpdatesSinceReport();
    const float imu_hz = imu_n / window_s;
    Serial.printf("%-6s: last=%5lu  max=%5lu\n", "IMU", imuSensor->lastExecUs(), imuSensor->maxExecUs());
    Serial.printf("        rate=%.1f Hz (%lu/%lu ms)\n", imu_hz, static_cast<unsigned long>(imu_n), static_cast<unsigned long>(elapsed_ms));

    // Mag
    const uint32_t mag_n = magSensor->consumeUpdatesSinceReport();
    const float mag_hz = mag_n / window_s;
    Serial.printf("%-6s: last=%5lu  max=%5lu\n", "Mag", magSensor->lastExecUs(), magSensor->maxExecUs());
    Serial.printf("        rate=%.1f Hz (%lu/%lu ms)\n", mag_hz, static_cast<unsigned long>(mag_n), static_cast<unsigned long>(elapsed_ms));

    // Baro
    const uint32_t baro_n = baroSensor->consumeUpdatesSinceReport();
    const float baro_hz = baro_n / window_s;
    Serial.printf("%-6s: last=%5lu  max=%5lu\n", "Baro", baroSensor->lastExecUs(), baroSensor->maxExecUs());
    Serial.printf("        rate=%.1f Hz (%lu/%lu ms)\n", baro_hz, static_cast<unsigned long>(baro_n), static_cast<unsigned long>(elapsed_ms));

    Serial.printf("Total: %lu us\n", imuSensor->lastExecUs() + magSensor->lastExecUs() + baroSensor->lastExecUs());
}

Eigen::Vector3d quat_to_euler(const Eigen::Quaterniond& q) {
    Eigen::Vector3d euler;

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    euler(0) = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        euler(1) = std::copysign(M_PI / 2, sinp);
    else
        euler(1) = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    euler(2) = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}

void printStates() {
  static uint32_t last_print = 0;
  if (millis() - last_print < 1000) return;
  last_print = millis();

  Serial.println("EKF States:");
  Serial.printf("Position:  %.6f, %.6f, %.3f\n", ekf->pos().x(), ekf->pos().y(), ekf->pos().z());
  Serial.printf("Velocity:  %.3f, %.3f, %.3f\n", ekf->vel().x(), ekf->vel().y(), ekf->vel().z());
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("Starting up...");
    Serial.flush();

    Wire.begin();
    Wire.setClock(400000);

    const bool sd_ok = sensors::TeensySensorLogger::initSd();
    if (!sd_ok) {
        Serial.println("WARNING: SD init failed; disabling sensor logging");
        Serial.flush();
    }

    Serial.println("Initializing EKF...");
    Serial.flush();

    // Create EKF on heap (deferred from static init)
    ekf = new EKF16d(TEENSY_PTYPE_DATA_PARAMS);

    // Reset with a default NavigationState — identity quaternion, zero vel/biases,
    // zero position (GNSS first fix will snap it). reset() handles sensible P0
    // per block (pos/vel/att/ba/bg/bbaro) instead of a uniform 0.1 identity,
    // and avoids the zero-quaternion degeneracy that initialize(Zero(), ...)
    // would introduce.
    ekf->reset(shared::NavigationState{});
    //ekf->debugCallback = ekfDebug;

    // Create sensors
    imuSensor  = new BNO055Imu(ekf, 10);   // 100 Hz
    gnssSensor = new UbloxGnss(ekf, 100);  // 10 Hz
    magSensor  = new BNO055Mag(imuSensor->bno(), ekf, 20);  // 50 Hz
    baroSensor = new BMP280Baro(ekf, 50);  // 20 Hz

    // Configure loggers
    imu_log.save_to_sd  = sd_ok;
    mag_log.save_to_sd  = sd_ok;
    baro_log.save_to_sd = sd_ok;
    gnss_log.save_to_sd = false;

    // Pair sensors with their loggers
    sensor_entries[0] = { imuSensor,  &imu_log  };
    sensor_entries[1] = { magSensor,  &mag_log  };
    sensor_entries[2] = { baroSensor, &baro_log };
    //sensor_entries[3] = { gnssSensor, &gnss_log };

    // Initialize all sensors
    Serial.println("Initializing sensors...");
    for (size_t i = 0; i < NUM_SENSORS; i++) {
        if (!sensor_entries[i].sensor->initialize()) {
            Serial.printf("FATAL: %s init failed\n", sensor_entries[i].sensor->name());
            while (1) delay(1000);
        }
    }

    Serial.println("All sensors initialized");
}

void loop() {
    uint32_t now_us = micros();
    uint32_t now_ms = now_us / 1000;

    for (size_t i = 0; i < NUM_SENSORS; i++) {
        auto& [sensor, logger] = sensor_entries[i];
        if (sensor->is_due(now_us)) sensor->update(now_us);
        sensor->log_to(*logger, now_ms);
    }

    printTimings();
    //printStates();
}
