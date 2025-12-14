#ifndef MOCK_SENSOR_DATA_H
#define MOCK_SENSOR_DATA_H

#include <Eigen/Dense>
#include <random>
#include <fstream>
#include <cmath>
#include "sensor_io.h"

// Simulates sensor data for controlled test scenarios with realistic baseline values
class MockSensorData {
public:
    MockSensorData(unsigned seed = 42, const std::string& csv_file = "../../data/sensor_data.csv") 
        : gen_(seed) {
        // Initialize with realistic baseline values from CSV data
        initializeFromCSV(csv_file);
    }
    
    Eigen::Vector3d noisyAccel(const Eigen::Vector3d& true_val, double std_dev) {
        std::normal_distribution<> d(0.0, std_dev);
        return true_val + Eigen::Vector3d(d(gen_), d(gen_), d(gen_));
    }
    
    Eigen::Vector3d noisyGyro(const Eigen::Vector3d& true_val, double std_dev) {
        std::normal_distribution<> d(0.0, std_dev);
        return true_val + Eigen::Vector3d(d(gen_), d(gen_), d(gen_));
    }
    
    double noisyBaro(double true_alt, double std_dev) {
        std::normal_distribution<> d(0.0, std_dev);
        double noisy_alt = true_alt + d(gen_);
        return 1013.25 * pow(1.0 - noisy_alt / 44330.0, 1.0 / 0.1903);
    }
    
    // Get realistic baseline values from actual sensor data
    Eigen::Vector3d getBaselineAccel() const { return baseline_accel_; }
    Eigen::Vector3d getBaselineGyro() const { return baseline_gyro_; }
    Eigen::Vector3d getBaselineMag() const { return baseline_mag_; }
    double getBaselinePressure() const { return baseline_pressure_; }

private:
    std::mt19937 gen_;
    Eigen::Vector3d baseline_accel_;
    Eigen::Vector3d baseline_gyro_;
    Eigen::Vector3d baseline_mag_;
    double baseline_pressure_;
    
    void initializeFromCSV(const std::string& csv_file) {
        // Set defaults in case file read fails
        baseline_accel_ = Eigen::Vector3d(0.0, 0.0, 1.0); // 1g down
        baseline_gyro_ = Eigen::Vector3d::Zero();
        baseline_mag_ = Eigen::Vector3d(1.0, 0.0, -0.5);
        baseline_pressure_ = 1013.25;
          
        try {
            SensorReader reader(csv_file);
            SensorData data;
            
            Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d mag_sum = Eigen::Vector3d::Zero();
            double pressure_sum = 0.0;
            int count = 0;
            int max_samples = 100; // Sample first 100 points for baseline
            
            while (reader.read(data) && count < max_samples) {
                accel_sum += data.accel;
                gyro_sum += data.gyro;
                mag_sum += data.mag;
                pressure_sum += data.pressure;
                count++;
            }
            
            if (count > 0) {
                baseline_accel_ = accel_sum / count;
                baseline_gyro_ = gyro_sum / count;
                baseline_mag_ = mag_sum / count;
                baseline_pressure_ = pressure_sum / count;
            }
        } catch (const std::exception&) {
            // Keep defaults if reading fails
        }
    }
};

#endif // MOCK_SENSOR_DATA_H
