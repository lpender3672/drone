#ifndef SENSOR_BARO_H
#define SENSOR_BARO_H

#include "sensor_base.h"
#include <ekf16d_opt.h>

// Placeholder - replace with your barometer library
// e.g., #include <Adafruit_BMP280.h>

class BaroSensor : public SensorBase {
private:
    EKF16d_OPT* ekf_;
    double baro_std_ = 1.0;  // meters
    double alt_ref_ = 0.0;
    bool ref_set_ = false;
    
    // TODO: Add your barometer instance here
    // Adafruit_BMP280 bmp_;

public:
    BaroSensor(EKF16d_OPT* ekf, uint32_t interval_ms = 50)
        : SensorBase("Baro", interval_ms), ekf_(ekf) {}

    bool initialize() override {
        // TODO: Initialize barometer hardware
        // if (!bmp_.begin()) return false;
        
        Serial.println("Baro sensor ready (stub)");
        return true;
    }

    void update() override {
        startTiming();

        // TODO: Read from actual barometer
        // float pressure = bmp_.readPressure();
        // float alt = bmp_.readAltitude(1013.25);
        
        float alt = 100.0f;  // Placeholder

        if (!ref_set_) {
            alt_ref_ = alt;
            ref_set_ = true;
        }

        double alt_relative = 0.1;
        double R_var = 0.1;

        ekf_->update_barometer(alt_relative, R_var);

        endTiming();
    }

    void setBaroStd(double std) { baro_std_ = std; }
    void resetReference() { ref_set_ = false; }
};

#endif