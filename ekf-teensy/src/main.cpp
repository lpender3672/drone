#include <Eigen/Dense>
#include <ekf16d_opt.h>

#include <Arduino.h>
#include "sensor_base.h"
#include "imu.h"
#include "gnss.h"
#include "mag.h"
#include "baro.h"

#define EIGEN_NO_MALLOC
#define EIGEN_NO_DEBUG
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

// Memory diagnostics
extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern char *__brkval;
int freeram() { return (char *)&_heap_end - __brkval; }

// EKF instance
EKF16d_OPT ekf;

// Sensor instances
ImuSensor*  imuSensor;
GnssSensor* gnssSensor;
MagSensor*  magSensor;
BaroSensor* baroSensor;

// Sensor array for polymorphic iteration
constexpr size_t NUM_SENSORS = 4;
SensorBase* sensors[NUM_SENSORS];

void ekfDebug(const char* label) {
    Serial.printf("%s - free: %d\n", label, freeram());
}

void printTimings() {
    static uint32_t last_print = 0;
    if (millis() - last_print < 1000) return;
    last_print = millis();
    
    Serial.println("--- Sensor Timings (us) ---");
    uint32_t total = 0;
    for (size_t i = 0; i < NUM_SENSORS; i++) {
        Serial.printf("%-6s: last=%5lu  max=%5lu\n", 
                      sensors[i]->name(),
                      sensors[i]->lastExecUs(),
                      sensors[i]->maxExecUs());
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

  NominalState state = ekf.getState();
  
  Serial.println("EKF States:");
  Serial.printf("Position:  %.6f, %.6f, %.3f\n", state.p(0), state.p(1), state.p(2));
  Serial.printf("Velocity:  %.3f, %.3f, %.3f\n", state.v(0), state.v(1), state.v(2));
  
  Eigen::Vector3d euler_deg = quat_to_euler(state.q) / DEG_TO_RAD;
  Serial.printf("Attitude:  Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", 
                euler_deg(0), euler_deg(1), euler_deg(2));

}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Wire.begin();
    Wire.setClock(400000);

    Serial.println("Initializing EKF...");
    ekf.initialize(
        Eigen::Vector3d::Zero(),           // position
        Eigen::Vector3d::Zero(),           // velocity
        Eigen::Quaterniond::Identity(),    // attitude
        Eigen::Vector3d::Zero(),           // gyro bias
        Eigen::Vector3d::Zero(),           // accel bias
        0.0,                               // baro bias
        Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>::Identity() * 0.1
    );
    ekf.debugCallback = ekfDebug;

    // Create sensors
    imuSensor  = new ImuSensor(&ekf, 10);   // 100 Hz
    gnssSensor = new GnssSensor(&ekf, 100); // 10 Hz
    magSensor  = new MagSensor(imuSensor->bno(), &ekf, 20);  // 50 Hz
    baroSensor = new BaroSensor(&ekf, 50);  // 20 Hz

    // Register in array
    sensors[0] = imuSensor;
    sensors[1] = gnssSensor;
    sensors[2] = magSensor;
    sensors[3] = baroSensor;

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

    //printTimings();
    //printStates();
}
