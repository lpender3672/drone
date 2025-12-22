#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include <cmath>

#include "eskf.h" 
#include "RTIMULib.h"
#include "gps_interface.h"
#include "sensor_io.h"

volatile bool running = true;
void signalHandler(int signum) { running = false; }

const double P0_HPA = 1013.25;
const double G_ACCEL = 9.80665;
const double DEG_TO_RAD = M_PI / 180.0;

double calculateAltitude(double pressure_hPa) {
    return 44330.0 * (1.0 - pow(pressure_hPa / P0_HPA, 0.1903));
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    // GPS configuration (can be overridden via args)
    std::string gps_device = "/dev/ttyACM0";
    int gps_baudrate = 115200;
    int gps_rate_hz = 5;
    
    if (argc > 1) gps_device = argv[1];
    if (argc > 2) gps_baudrate = std::stoi(argv[2]);
    if (argc > 3) gps_rate_hz = std::stoi(argv[3]);

    // --- 1. Initialize Sensors ---
    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");    
    settings->m_fusionType = RTFUSION_TYPE_NULL;
    
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

    // --- 2. Initialize GPS ---
    GPSInterface gps(gps_device, gps_baudrate);
    
    if (!gps.open()) {
        std::cerr << "Failed to open GPS device: " << gps_device << std::endl;
        std::cerr << "Continuing without GPS..." << std::endl;
    } else {
        std::cout << "GPS device opened: " << gps_device << std::endl;
        gps.setUpdateRate(gps_rate_hz);
        gps.configureUBX(0x01, 0x07, 1); // NAV-PVT
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // --- 3. Initial Alignment ---
    std::cout << "Collecting samples for initial alignment (2 seconds)..." << std::endl;
    Eigen::Vector3d sum_acc(0,0,0);
    int sample_count = 0;

    auto warmupStart = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - warmupStart).count() < 2) {
        if (imu->IMURead()) {
            RTIMU_DATA data = imu->getIMUData();
            sum_acc += Eigen::Vector3d(data.accel.x(), data.accel.y(), data.accel.z());
            sample_count++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    Eigen::Vector3d avg_acc = sum_acc / sample_count;
    double init_roll  = atan2(-avg_acc.y(), -avg_acc.z());
    double init_pitch = atan2(avg_acc.x(), sqrt(avg_acc.y()*avg_acc.y() + avg_acc.z()*avg_acc.z()));

    Eigen::Quaterniond q_init;
    q_init = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(init_pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(init_roll,  Eigen::Vector3d::UnitX());
    
    std::cout << "Initial Roll: " << init_roll * 180.0/M_PI 
              << " Pitch: " << init_pitch * 180.0/M_PI << std::endl;

    // --- 4. Wait for GPS fix to initialize position ---
    Eigen::Vector3d p0(45.0 * DEG_TO_RAD, 0.0, 100.0); // Default
    Eigen::Vector3d v0(0,0,0);
    bool gps_initialized = false;

    std::cout << "Waiting for GPS fix..." << std::endl;
    auto gps_wait_start = std::chrono::steady_clock::now();
    
    while (!gps_initialized && 
           std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - gps_wait_start).count() < 30) {
        GPSData gps_data;
        if (gps.readData(gps_data, 1000) && gps_data.valid_position) {
            p0 = Eigen::Vector3d(
                gps_data.latitude * DEG_TO_RAD,
                gps_data.longitude * DEG_TO_RAD,
                gps_data.altitude_msl
            );
            v0 = Eigen::Vector3d(gps_data.velocity_north, gps_data.velocity_east, gps_data.velocity_down);
            gps_initialized = true;
            std::cout << "GPS fix acquired. Initial position: " 
                      << gps_data.latitude << ", " << gps_data.longitude 
                      << ", " << gps_data.altitude_msl << "m" << std::endl;
        }
    }
    
    if (!gps_initialized) {
        std::cout << "No GPS fix after 30s, using default position" << std::endl;
    }

    // --- 5. Initialize ESKF ---
    EsEkf ekf;
    Eigen::Vector3d ba0(0,0,0), bg0(0,0,0);
    double bb0 = 0.0;

    Eigen::Matrix<double, 15, 15> P0;
    P0.setIdentity();
    P0.block<3,3>(0,0) *= gps_initialized ? 1e-10 : 1e-6; // Tighter if GPS fix
    P0.block<3,3>(3,3) *= 0.1;
    P0.block<3,3>(6,6) *= 0.01;
    P0.block<3,3>(9,9) *= 0.01;
    P0.block<3,3>(12,12) *= 0.001;

    ekf.initialize(p0, v0, q_init, ba0, bg0, bb0, P0);

    // --- 6. Main Loop ---
    INSWriter insWriter("data/ekf_data.csv", 100);

    int imu_count = 0, gps_count = 0;

    auto lastTime = std::chrono::steady_clock::now();
    auto lastGpsRead = lastTime;
    auto lastBaroRead = lastTime;
    auto startTime = lastTime;
    
    std::cout << "ESKF Running. Press Ctrl+C to stop." << std::endl;

    while (running) {
        auto now = std::chrono::steady_clock::now();
        auto loopStart = std::chrono::high_resolution_clock::now();
        
        // --- IMU Predict (high rate) ---
        if (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
            
            double dt = std::chrono::duration_cast<std::chrono::microseconds>(
                now - lastTime).count() / 1e6;
            lastTime = now;
            if (dt > 0.1) dt = 0.1;

            ImuMeasurement msg;
            msg.t = 0.0;
            msg.acc = Eigen::Vector3d(imuData.accel.x(), imuData.accel.y(), imuData.accel.z()) * G_ACCEL;
            msg.gyro = Eigen::Vector3d(imuData.gyro.x(), imuData.gyro.y(), imuData.gyro.z());

            ekf.predict(msg, dt);
            imu_count++;
        }

        auto baro_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastBaroRead).count();
        if (baro_elapsed >= 100) {
            lastBaroRead = now;
            RTIMU_DATA imuData;
            if (pressure->pressureRead(imuData) && imuData.pressureValid) {
                double baro_alt = calculateAltitude(imuData.pressure);
                ekf.update_barometer(baro_alt, 4.0);
            }
        }

        // --- GPS Update (lower rate, non-blocking) ---
        auto gps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastGpsRead).count();
        if (gps_elapsed >= 200) {
            lastGpsRead = now;
            GPSData gps_data;
            if (gps.readData(gps_data, 0)) { // 0 timeout = non-blocking
                gps_data.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                    now - startTime).count() / 1e6;
                
                if (gps_data.valid_position && gps_data.fix_type >= 2) {
                    gps_count++;
                    
                    // Position update: convert GPS accuracy to covariance
                    // horizontal_accuracy is CEP (circular error probable), ~1 sigma
                    double h_var = gps_data.horizontal_accuracy * gps_data.horizontal_accuracy;
                    double v_var = gps_data.vertical_accuracy * gps_data.vertical_accuracy;
                    
                    // Convert lat/lon accuracy from meters to radians
                    NominalState state = ekf.getState();
                    double R_N = 6378137.0 * (1.0 - 0.00669438 * sin(state.p.x()) * sin(state.p.x()));
                    double lat_var = h_var / (R_N * R_N);
                    double lon_var = h_var / (R_N * cos(state.p.x()) * R_N * cos(state.p.x()));

                    Eigen::Vector3d pos_gnss(
                        gps_data.latitude * DEG_TO_RAD,
                        gps_data.longitude * DEG_TO_RAD,
                        gps_data.altitude_msl
                    );
                    
                    Eigen::Matrix3d R_pos = Eigen::Matrix3d::Zero();
                    R_pos(0,0) = lat_var;
                    R_pos(1,1) = lon_var;
                    R_pos(2,2) = v_var;
                    
                    ekf.update_gnss_position(pos_gnss, R_pos);

                    // Velocity update
                    if (gps_data.valid_velocity) {
                        double speed_var = gps_data.speed_accuracy * gps_data.speed_accuracy;
                        
                        Eigen::Vector3d vel_gnss(gps_data.velocity_north, gps_data.velocity_east, gps_data.velocity_down);
                        Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * speed_var;
                        
                        ekf.update_gnss_velocity(vel_gnss, R_vel);
                    }

                    // Status output
                    if (gps_count % 5 == 0) {
                        std::cout << "\r[IMU:" << imu_count << " GPS:" << gps_count << "] "
                                << "Fix:" << (int)gps_data.fix_type << "D "
                                << "Sats:" << (int)gps_data.num_satellites << " "
                                << "HAcc:" << std::fixed << std::setprecision(1) 
                                << gps_data.horizontal_accuracy << "m    " << std::flush;
                    }
                }
            }
        }

        // --- Logging ---
        NominalState state = ekf.getState();
        double t = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - startTime).count() / 1000.0;
            
        INSData ins_data;
        ins_data.timestamp = t;
        ins_data.position_lla = state.p;
        ins_data.velocity = state.v;
        ins_data.attitude = state.q;
        ins_data.accel_bias = state.ba;
        ins_data.gyro_bias = state.bg;

        insWriter.write(ins_data);

        std::this_thread::sleep_until(loopStart + std::chrono::milliseconds(3));
    }

    std::cout << "\n\nStopped. IMU samples: " << imu_count << ", GPS fixes: " << gps_count << std::endl;
    
    gps.close();
    delete imu; delete pressure; delete settings;
    return 0;
}