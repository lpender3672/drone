#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include <cmath>

// [Source: 352] Include the derived EKF header
#include "eskf.h" 
#include "RTIMULib.h"

volatile bool running = true;
void signalHandler(int signum) { running = false; }

// [Source: 259] Constants for Pressure->Altitude conversion
const double T0_KELVIN = 288.15;
const double L_RATE = 0.0065;   // K/m
const double P0_HPA = 1013.25;
const double R_GAS = 287.053;   // J/(kg*K) [Approx for air, derived from Eq 59 context]
const double G_ACCEL = 9.80665;

// Helper: Convert Pressure (hPa) to Altitude (m) using Eq (59)
double calculateAltitude(double pressure_hPa) {
    // h = (T0 / L) * (1 - (p / p0)^( (L*R) / g ) ) ... Wait, PDF Eq 59 exponent is LR/gM? 
    // Actually standard barometric formula exponent is (R_specific * L) / g.
    // Let's use the exact exponent form implied by standard atmosphere if PDF Eq 59 is generic: 
    // Exponent = 1/5.255 approx.
    double exponent = (R_GAS * L_RATE) / G_ACCEL; 
    // Note: PDF Eq 59 has "LR/gM" which implies M is molar mass. 
    // Standard approx: (1 - (p/p0)^0.1903) * 44330.
    return 44330.0 * (1.0 - pow(pressure_hPa / P0_HPA, 0.1903));
}

int main() {
    signal(SIGINT, signalHandler);

    // --- 1. Load Settings ---
    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");    
    // Disable Internal Fusion to run raw ES-EKF
    settings->m_fusionType = RTFUSION_TYPE_NULL;
    
    // --- 2. Create Sensors ---
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

    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    // --- 3. Initial Static Alignment (Optional but recommended) ---
    // The PDF suggests initializing quaternion from gravity/mag[cite: 277].
    std::cout << "Collecting samples for initial alignment (2 seconds)..." << std::endl;
    Eigen::Vector3d sum_acc(0,0,0);
    Eigen::Vector3d sum_mag(0,0,0);
    int sample_count = 0;

    auto warmupStart = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - warmupStart).count() < 2) {
        if (imu->IMURead()) {
            RTIMU_DATA data = imu->getIMUData();
            sum_acc += Eigen::Vector3d(data.accel.x(), data.accel.y(), data.accel.z());
            sum_mag += Eigen::Vector3d(data.compass.x(), data.compass.y(), data.compass.z());
            sample_count++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Compute averages
    Eigen::Vector3d avg_acc = sum_acc / sample_count;
    Eigen::Vector3d avg_mag = sum_mag / sample_count;

    // Calculate Pitch/Roll from Gravity
    // Assuming stationary: acc = [0, 0, -g] in Navigation frame (NED)
    // Actually, RTIMULib acc is in body frame. 
    // pitch = atan2(acc.x, sqrt(acc.y^2 + acc.z^2))
    // roll  = atan2(-acc.y, -acc.z)
    double init_roll  = atan2(-avg_acc.y(), -avg_acc.z());
    double init_pitch = atan2(avg_acc.x(), sqrt(avg_acc.y()*avg_acc.y() + avg_acc.z()*avg_acc.z()));
    
    // Calculate Yaw from Mag (Simplified, normally requires tilt compensation)
    // For now, we initialize yaw to 0 or use basic atan2(mag.y, mag.x)
    double init_yaw = 0.0; 

    // Create Initial Quaternion (Body to Nav)
    Eigen::Quaterniond q_init;
    q_init = Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(init_pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(init_roll,  Eigen::Vector3d::UnitX());
    
    std::cout << "Initialized with Roll: " << init_roll * 180.0/M_PI 
              << " Pitch: " << init_pitch * 180.0/M_PI << std::endl;

    // --- 4. Initialize ES-EKF ---
    EsEkf ekf;

    // Initial State
    Eigen::Vector3d p0(45.0 * M_PI / 180.0, 0.0, 100.0); // Lat (rad), Lon (rad), Alt (m)
    Eigen::Vector3d v0(0,0,0); // NED Velocity
    Eigen::Vector3d ba0(0,0,0);
    Eigen::Vector3d bg0(0,0,0);

    // Initial Covariance [Source: 279]
    Eigen::Matrix<double, 15, 15> P0;
    P0.setIdentity();
    P0.block<3,3>(0,0) *= 1e-6; // Position (rad/m)
    P0.block<3,3>(3,3) *= 0.1;  // Velocity
    P0.block<3,3>(6,6) *= 0.01; // Attitude
    P0.block<3,3>(9,9) *= 0.01; // Accel Bias
    P0.block<3,3>(12,12) *= 0.001; // Gyro Bias

    ekf.initialize(p0, v0, q_init, ba0, bg0, P0);

    // --- 5. Main Loop ---
    std::ofstream logFile("ins_data.csv");
    logFile << "time,lat,lon,alt,vn,ve,vd,qw,qx,qy,qz" << std::endl;

    auto lastTime = std::chrono::steady_clock::now();
    std::cout << "ESKF Running. Press Ctrl+C to stop." << std::endl;

    while (running) {
        if (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
            
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - lastTime).count() / 1e6;
            lastTime = now;
            if (dt > 0.1) dt = 0.1; // Clamp large dts

            // --- PREDICT ---
            ImuMeasurement msg;
            msg.t = 0.0; // t not used in predict calc, only dt
            
            // [Source: 337] Convert 'g' to 'm/s^2'
            msg.acc = Eigen::Vector3d(imuData.accel.x(), imuData.accel.y(), imuData.accel.z()) * G_ACCEL;
            
            // [Source: 338] Gyro is already rad/s in RTIMULib
            msg.gyro = Eigen::Vector3d(imuData.gyro.x(), imuData.gyro.y(), imuData.gyro.z());

            ekf.predict(msg, dt);

            // --- UPDATE: Barometer ---
            // [Source: 262] Innovation z = h_baro - h_ins
            if (pressure->pressureRead(imuData) && imuData.pressureValid) {
                // Convert hPa to Altitude [Source: 259]
                double baro_alt = calculateAltitude(imuData.pressure);
                
                // Uncertainty R (variance)
                double R_baro = 2.0 * 2.0; // 2 meters std dev
                ekf.update_barometer(baro_alt, R_baro);
            }
            
            // Note: Magnetometer update is NOT included because the provided derivation (PDF)
            // does not define a measurement model for it in Section 8.
            
            // --- LOGGING ---
            NominalState state = ekf.getState();
            
            logFile << std::fixed << std::setprecision(9)
                    << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() / 1000.0 << ","
                    << state.p.x() << "," << state.p.y() << "," << state.p.z() << ","
                    << state.v.x() << "," << state.v.y() << "," << state.v.z() << ","
                    << state.q.w() << "," << state.q.x() << "," << state.q.y() << "," << state.q.z()
                    << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    logFile.close();
    delete imu; delete pressure; delete settings;
    return 0;
}