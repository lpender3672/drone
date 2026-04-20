#ifndef EKF_TEENSY_BMP280_BARO_H
#define EKF_TEENSY_BMP280_BARO_H

#include <sensor_base.h>
#include <sensor_readings.h>
#include "teensy_sensor_logger.h"
#include <Adafruit_BMP280.h>
#include <observer.h>
#include <Wire.h>

/**
 * BMP280 barometer sensor implementation for Teensy.
 * Provides pressure, temperature, and altitude readings.
 */
class BMP280Baro : public sensors::Sensor<sensors::BaroReading>, public sensors::TeensySensorLogger {
private:
    shared::IObserverWithBiases* observer_;
    bool ref_set_ = false;
    float p0_pa_ = 101325.0f;  // Standard atmospheric pressure

    Adafruit_BMP280 bmp_;

    // ISA approximation for altitude from pressure
    // h = 44330 * (1 - (p/p0)^(1/5.255))
    static float altitudeFromPressurePa(float p_pa, float p0_pa) {
        if (p_pa <= 0.0f || p0_pa <= 0.0f) return 0.0f;
        return 44330.0f * (1.0f - powf(p_pa / p0_pa, 1.0f / 5.255f));
    }

public:
    BMP280Baro(shared::IObserverWithBiases* observer, uint32_t interval_ms = 50)
        : Sensor<sensors::BaroReading>("Baro", (uint64_t)interval_ms * 1000),
          TeensySensorLogger("Baro"),
          observer_(observer) {}

    void set_reference_pressure(float pressure_pa) {
        p0_pa_ = pressure_pa;
    }

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
            initialized_ = false;
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
        initialized_ = true;
        return true;
    }

    bool update(uint64_t current_time_us_64) override {
        uint32_t current_time_us = static_cast<uint32_t>(current_time_us_64);
        startTiming();

        const float p_pa = bmp_.readPressure();
        const float temp_c = bmp_.readTemperature();
        
        if (p_pa <= 0.0f) {
            endTiming();
            return false;
        }

        if (!ref_set_) {
            p0_pa_ = p_pa;
            ref_set_ = true;
        }

        const float alt_relative = altitudeFromPressurePa(p_pa, p0_pa_);

        // Update the standardized reading structure
        latest_reading_.timestamp_us = current_time_us;
        latest_reading_.pressure_pa = p_pa;
        latest_reading_.temperature_c = temp_c;
        latest_reading_.altitude_m = alt_relative;
        latest_reading_.valid = true;

        new_reading_available_ = true;

        observer_->feed_baro(latest_reading_);

        // Log data to SD card if enabled
        struct BaroLogSample {
            float alt_relative_m;
            float pressure_pa;
            float temp_c;
        } sample;
        sample.alt_relative_m = alt_relative;
        sample.pressure_pa = p_pa;
        sample.temp_c = temp_c;

        endTiming();

        uint32_t now_ms = current_time_us / 1000;
        saveValueIfEnabled(now_ms, sample);
        markUpdated(current_time_us);
        return true;
    }

    void resetReference() { ref_set_ = false; }
};

#endif  // EKF_TEENSY_BMP280_BARO_H
