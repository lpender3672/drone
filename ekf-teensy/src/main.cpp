#include <Eigen/Dense>
#include <ekf16d.h>

#include <Arduino.h>
#include "sensor_base.h"
#include "imu.h"
#include "gnss.h"
#include "mag.h"
#include "baro.h"

#include "tuned_ekf_params.h"

// Memory diagnostics
extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern char *__brkval;
int freeram() { return (char *)&_heap_end - __brkval; }

// EKF instance - use pointer for deferred initialization
EKF16d* ekf = nullptr;

// Sensor instances
ImuSensor*  imuSensor;
GnssSensor* gnssSensor;
MagSensor*  magSensor;
BaroSensor* baroSensor;

// Sensor array for polymorphic iteration
constexpr size_t NUM_SENSORS = 2;
SensorBase* sensors[NUM_SENSORS];

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
    uint32_t total = 0;
    for (size_t i = 0; i < NUM_SENSORS; i++) {
        const uint32_t n = sensors[i]->consumeUpdatesSinceReport();
        const float hz = n / window_s;
        Serial.printf("%-6s: last=%5lu  max=%5lu\n", 
                      sensors[i]->name(),
                      sensors[i]->lastExecUs(),
                      sensors[i]->maxExecUs());
        Serial.printf("        rate=%.1f Hz (%lu/%lu ms)\n", hz, static_cast<unsigned long>(n), static_cast<unsigned long>(elapsed_ms));
        total += sensors[i]->lastExecUs();
    }
    Serial.printf("Total: %lu us\n", total);
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
        euler(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
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

    const bool sd_ok = SensorBase::initSd();
    if (!sd_ok) {
        Serial.println("WARNING: SD init failed; disabling sensor logging");
        Serial.flush();
    }

    Serial.println("Initializing EKF...");
    Serial.flush();
    
    // Create EKF on heap (deferred from static init)
    ekf = new EKF16d(TEENSY_PTYPE_DATA_PARAMS);
    
    ekf->initialize(
        EKF16d::NominalVector::Zero(),
        EKF16d::CovMatrix::Identity() * 0.1
    );
    //ekf->debugCallback = ekfDebug;

    // Create sensors
    imuSensor  = new ImuSensor(ekf, 10);   // 100 Hz
    imuSensor->save_to_sd = sd_ok;
    gnssSensor = new GnssSensor(ekf, 100); // 10 Hz
    gnssSensor->save_to_sd = false && sd_ok;
    //magSensor  = new MagSensor(imuSensor->bno(), ekf, 20);  // 50 Hz
    //magSensor->save_to_sd = sd_ok;
    baroSensor = new BaroSensor(ekf, 50);  // 20 Hz
    baroSensor->save_to_sd = sd_ok;

    // Register in array
    sensors[0] = imuSensor;
    //sensors[1] = magSensor;
    sensors[1] = baroSensor;
    //sensors[3] = gnssSensor;

    // Initialize all sensors
    Serial.println("Initializing sensors...");
    for (size_t i = 0; i < NUM_SENSORS; i++) {
        if (!sensors[i]->initialize()) {
            Serial.printf("FATAL: %s init failed\n", sensors[i]->name());
            while (1) delay(1000);
        }
    }
    
    Serial.println("All sensors initialized");
}

void loop() {
    uint32_t now = millis();

    for (size_t i = 0; i < NUM_SENSORS; i++) {
        if (sensors[i]->shouldUpdate(now)) {
            sensors[i]->update();
            sensors[i]->markUpdated(now);
        }
    }

    printTimings();
    //printStates();
}
