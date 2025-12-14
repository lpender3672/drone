#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include "RTIMULib.h"
#include "eskf.h"
#include "sensor_io.h"

volatile bool running = true;
void signalHandler(int signum) { running = false; }

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
    // We set fusion to NULL because you are running your own ESKF.
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
    std::cout << "Sensors Initialized" << std::endl;
    std::cout << "Internal Fusion: DISABLED (Using Custom ESKF)" << std::endl;
    std::cout << "Compass Calibrated: " << (compassCal ? "YES" : "NO (Run RTIMULibCal!)") << std::endl;
    std::cout << "Accel Calibrated:   " << (accelCal ? "YES" : "NO") << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    if (!compassCal) {
        std::cerr << "WARNING: Compass is not calibrated. Yaw will be inaccurate." << std::endl;
        std::cerr << "Please run 'RTIMULibCal' to generate RTIMULib.ini" << std::endl;
        // We don't exit, we just warn.
    }

    // --- 5. Warm Up ---
    std::cout << "Warming up sensors for 2 seconds..." << std::endl;
    auto warmupStart = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - warmupStart).count() < 2) {
        if (imu->IMURead()) { /* discard */ }
    }

    // --- 6. Setup ESKF ---
    SensorParams params;
    // Parameters from your data
    params.N_acc = 1.9862e-04;  params.B_acc = 2.2751e-05; params.tau_acc = 13.69;
    params.N_gyro = 1.9639e-04; params.B_gyro = 1.9129e-04; params.tau_gyro = 30.09;

    // Initial Location (Lat/Lon/Alt)
    Eigen::Vector3d initial_lla(45.0 * M_PI / 180.0, 0.0, 0.0);
    
    ESKF eskf(params, initial_lla);

    std::ofstream logFile("ins_data.csv");
    logFile << "time,lat,lon,alt,vn,ve,vd,roll,pitch,yaw" << std::endl;

    auto lastTime = std::chrono::steady_clock::now();
    
    std::cout << "ESKF Running. Press Ctrl+C to stop." << std::endl;

    while (running) {
        if (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
            
            // RTIMULib automatically applies calibration to these values
            // IF the RTIMULib.ini file is valid.
            
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - lastTime).count() / 1e6;
            lastTime = now;
            if (dt > 0.1) dt = 0.1;

            // Predict
            Eigen::Vector3d acc(imuData.accel.x(), imuData.accel.y(), imuData.accel.z());
            Eigen::Vector3d gyro(imuData.gyro.x(), imuData.gyro.y(), imuData.gyro.z());
            eskf.predict(acc, gyro, dt);

            // Updates
            if (imuData.compassValid) {
                eskf.updateMag(Eigen::Vector3d(imuData.compass.x(), imuData.compass.y(), imuData.compass.z()));
            }
            if (pressure->pressureRead(imuData) && imuData.pressureValid) {
                eskf.updateBaro(imuData.pressure);
            }

            // Log
            Eigen::Vector3d p = eskf.getPositionLLA();
            Eigen::Vector3d v = eskf.getVelocity();
            Eigen::Vector3d rpy = eskf.getAttitude().toRotationMatrix().eulerAngles(0, 1, 2);

            logFile << std::fixed << std::setprecision(9)
                    << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() / 1000.0 << ","
                    << p(0) << "," << p(1) << "," << p(2) << ","
                    << v(0) << "," << v(1) << "," << v(2) << ","
                    << rpy(0) << "," << rpy(1) << "," << rpy(2) << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    logFile.close();
    delete imu; delete pressure; delete settings;
    return 0;
}