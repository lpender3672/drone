#ifndef SENSOR_BARO_H
#define SENSOR_BARO_H

#include "sensor_base.h"
#include <Adafruit_BMP280.h>
#include <ekf.h>
#include <math.h>
#include <Wire.h>

class BaroSensor : public SensorBase {
private:
    IEKF* ekf_;
    double baro_std_ = 1.0;  // meters
    bool ref_set_ = false;

    Adafruit_BMP280 bmp_;
    float p0_pa_ = 101325.0f;

    static double altitudeFromPressurePa(double p_pa, double p0_pa) {
        // ISA approximation.
        // h = 44330 * (1 - (p/p0)^(1/5.255))
        if (p_pa <= 0.0 || p0_pa <= 0.0) return 0.0;
        return 44330.0 * (1.0 - pow(p_pa / p0_pa, 1.0 / 5.255));
    }

public:
    BaroSensor(IEKF* ekf, uint32_t interval_ms = 50)
        : SensorBase("Baro", interval_ms), ekf_(ekf) {}

    bool initialize() override {
        auto read_chip_id = [](uint8_t addr) -> int {
            Wire.beginTransmission(addr);
            Wire.write(static_cast<uint8_t>(0xD0));
            if (Wire.endTransmission(false) != 0) {
                Wire.endTransmission(true);
                return -1;
            }
            const uint8_t n = Wire.requestFrom(static_cast<int>(addr), 1);
            if (n != 1) return -1;
            return Wire.read();
        };

        // Common BMP280 I2C addresses are 0x76 or 0x77.
        // Some boards are actually BME280 (chip id 0x60); pressure/temp are compatible.
        const int id76 = read_chip_id(0x76);
        const int id77 = read_chip_id(0x77);
        if (id76 >= 0) Serial.printf("Baro probe 0x76: chip id 0x%02X\n", id76);
        if (id77 >= 0) Serial.printf("Baro probe 0x77: chip id 0x%02X\n", id77);

        auto try_begin = [&]() -> bool {
            // Prefer the probed chip id so we don't "accept" the wrong device at the address.
            if (id76 == BMP280_CHIPID) return bmp_.begin(0x76, BMP280_CHIPID);
            if (id77 == BMP280_CHIPID) return bmp_.begin(0x77, BMP280_CHIPID);
            if (id76 == 0x60) return bmp_.begin(0x76, 0x60);
            if (id77 == 0x60) return bmp_.begin(0x77, 0x60);

            // Fall back to a brute-force try if probe didn't work.
            bool ok = bmp_.begin(0x76, BMP280_CHIPID);
            if (!ok) ok = bmp_.begin(0x77, BMP280_CHIPID);
            if (!ok) ok = bmp_.begin(0x76, 0x60);
            if (!ok) ok = bmp_.begin(0x77, 0x60);
            return ok;
        };

        const uint32_t prev_clock = 400000;
        bool ok = try_begin();
        if (!ok) {
            // Retry at 100kHz in case the bus/wiring is marginal at 400k.
            Wire.setClock(100000);
            ok = try_begin();
            Wire.setClock(prev_clock);
        }

        if (!ok) {
            Serial.println("BMP280 init failed (not found on I2C or wrong wiring/mode)");
            return false;
        }

        bmp_.setSampling(
            Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X2,   // temp
            Adafruit_BMP280::SAMPLING_X16,  // pressure
            Adafruit_BMP280::FILTER_X16,
            Adafruit_BMP280::STANDBY_MS_63
        );

        // Establish reference pressure for relative altitude.
        const float p_pa = bmp_.readPressure();
        if (p_pa > 0.0f) {
            p0_pa_ = p_pa;
            ref_set_ = true;
        }

        Serial.printf("BMP280 initialized (chip id 0x%02X)\n", bmp_.sensorID());
        return true;
    }

    void update() override {
        startTiming();

        const uint32_t now_ms = millis();

        const float p_pa = bmp_.readPressure();
        const float temp_c = bmp_.readTemperature();
        if (p_pa <= 0.0f) {
            endTiming();
            return;
        }

        if (!ref_set_) {
            p0_pa_ = p_pa;
            ref_set_ = true;
        }

        const double alt_relative = altitudeFromPressurePa(p_pa, p0_pa_);
        const double R_var = baro_std_ * baro_std_;

        ekf_->update_barometer(alt_relative, R_var);

        endTiming();

        struct BaroLogSample {
            float alt_relative_m;
            float pressure_pa;
            float temp_c;
            float var;
        } sample;
        sample.alt_relative_m = static_cast<float>(alt_relative);
        sample.pressure_pa = p_pa;
        sample.temp_c = temp_c;
        sample.var = static_cast<float>(R_var);
        saveValueIfEnabled(now_ms, sample);
    }

    void setBaroStd(double std) { baro_std_ = std; }
    void resetReference() { ref_set_ = false; }
};

#endif