// sense_hat_logger.cpp
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include "RTIMULib.h"

volatile bool running = true;

void signalHandler(int signum) {
    running = false;
}

int main() {
    signal(SIGINT, signalHandler);
    
    // --- 1. Load Settings ---
    // This looks for "RTIMULib.ini" in the current folder
    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");    
    // FIX: Disable Internal Fusion here (in settings) before Init
    // This prevents the library from running its own Kalman/RTQF filter
    settings->m_fusionType = RTFUSION_TYPE_NULL;
    
    // --- 2. Create IMU ---
    RTIMU *imu = RTIMU::createIMU(settings);
    RTPressure *pressure = RTPressure::createPressure(settings);

    if ((imu == NULL) || (imu->IMUInit() < 0)) {
        std::cerr << "Failed to initialize IMU" << std::endl;
        return 1;
    }
    
    if ((pressure == NULL) || (pressure->pressureInit() < 0)) {
        std::cerr << "Failed to initialize pressure sensor" << std::endl;
        return 1;
    }

    // --- 3. Disable Internal Fusion & Enable Sensors ---
    // We set fusion to NULL because we want raw sensor data for logging.
    // This prevents the library from wasting CPU on its own filter.
    
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    // --- 4. Check Calibration Status ---
    // The library automatically loads calibration from the .ini file.
    // We just check if it was successful to inform the user.
    bool compassCal = imu->getCompassCalibrationValid();
    bool accelCal = imu->getAccelCalibrationValid();

    std::cout << "---------------------------------------" << std::endl;
    std::cout << "Sense Hat Logger Initialized" << std::endl;
    std::cout << "Internal Fusion: DISABLED (Raw Data Logging)" << std::endl;
    std::cout << "Compass Calibrated: " << (compassCal ? "YES" : "NO (Run RTIMULibCal!)") << std::endl;
    std::cout << "Accel Calibrated:   " << (accelCal ? "YES" : "NO") << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    if (!compassCal) {
        std::cerr << "WARNING: Compass is not calibrated. Magnetometer data will be inaccurate." << std::endl;
        std::cerr << "Please run 'RTIMULibCal' to generate RTIMULib.ini" << std::endl;
    }
    
    std::ofstream logFile("sensor_data.csv");
    logFile << "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,pressure" << std::endl;
    
    auto startTime = std::chrono::high_resolution_clock::now();
    int sampleRate = imu->IMUGetPollInterval();
    
    std::cout << "Logging started. Press Ctrl+C to stop." << std::endl;
    std::cout << "Poll interval: " << sampleRate << " ms" << std::endl;
    
    int count = 0;
    
    while (running) {
        auto loopStart = std::chrono::high_resolution_clock::now();
        
        if (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
            auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                loopStart - startTime).count() / 1000000.0;
            
            logFile << std::fixed << std::setprecision(6) << timestamp << ","
                    << imuData.accel.x() << "," << imuData.accel.y() << "," << imuData.accel.z() << ","
                    << imuData.gyro.x() << "," << imuData.gyro.y() << "," << imuData.gyro.z() << ","
                    << imuData.compass.x() << "," << imuData.compass.y() << "," << imuData.compass.z() << ",";
            
            if (pressure->pressureRead(imuData)) {
                logFile << imuData.pressure << std::endl;
            } else {
                logFile << "0" << std::endl;
            }
            
            // Flush to disk periodically
            if (++count % 100 == 0) {
                logFile.flush();
            }
        }
        
        std::this_thread::sleep_until(loopStart + std::chrono::milliseconds(3));
    }
    
    logFile.close();
    std::cout << "\nLogging stopped. Data saved to sensor_data.csv" << std::endl;
    
    return 0;
}
