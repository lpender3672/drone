#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include "eskf.h"

// Mock data structures for Windows compilation testing
struct MockIMUData {
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
    Eigen::Vector3d mag;
    double pressure;
    bool valid;
    
    MockIMUData() : valid(false) {}
};

volatile bool running = true;
void signalHandler(int signum) { running = false; }

// Mock function to simulate reading IMU data
MockIMUData readMockIMUData() {
    MockIMUData data;
    static double time = 0.0;
    time += 0.1; // 10Hz simulation
    
    // Generate some mock sensor data with noise
    data.accel = Eigen::Vector3d(0.1 * sin(time), 0.1 * cos(time), 9.81 + 0.05 * sin(2*time));
    data.gyro = Eigen::Vector3d(0.01 * cos(time), 0.01 * sin(time), 0.005 * sin(0.5*time));
    data.mag = Eigen::Vector3d(0.5, 0.0, -0.8); // Rough magnetic field
    data.pressure = 101325.0; // Sea level pressure
    data.valid = true;
    
    return data;
}

int main() {
    std::cout << "Windows Mock INS Runner" << std::endl;
    signal(SIGINT, signalHandler);

    // Initialize ESKF with mock parameters
    SensorParams params;
    params.N_acc = 0.01;    // Accelerometer noise
    params.N_gyro = 0.001;  // Gyroscope noise
    params.B_acc = 0.005;   // Accelerometer bias
    params.B_gyro = 0.0001; // Gyroscope bias
    params.tau_acc = 100.0; // Accelerometer correlation time
    params.tau_gyro = 100.0; // Gyroscope correlation time

    // Initial position (LLA) - Example: somewhere in the US
    Eigen::Vector3d init_lla(40.7128 * M_PI / 180.0, -74.0060 * M_PI / 180.0, 10.0); // NYC, 10m altitude
    
    ESKF eskf(params, init_lla);
    
    std::cout << "ESKF initialized with mock parameters" << std::endl;
    std::cout << "Initial Position (LLA): " << init_lla.transpose() << std::endl;
    std::cout << "Press Ctrl+C to stop..." << std::endl;

    auto last_time = std::chrono::high_resolution_clock::now();
    int iteration = 0;

    while (running) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_time);
        double dt = duration.count() / 1000000.0; // Convert to seconds
        
        if (dt >= 0.1) { // 10 Hz update rate
            MockIMUData imu_data = readMockIMUData();
            
            if (imu_data.valid) {
                // Predict step
                eskf.predict(imu_data.accel, imu_data.gyro, dt);
                
                // Optional: Update with magnetometer (compass)
                // eskf.updateMag(imu_data.mag);
                
                // Print status every 50 iterations (every 5 seconds at 10Hz)
                if (iteration % 50 == 0) {
                    Eigen::Vector3d pos = eskf.getPositionLLA();
                    Eigen::Vector3d vel = eskf.getVelocity();
                    Eigen::Quaterniond att = eskf.getAttitude();
                    
                    std::cout << std::fixed << std::setprecision(6);
                    std::cout << "Iter: " << iteration 
                              << " | Pos(LLA): " << pos.transpose() 
                              << " | Vel(NED): " << vel.transpose()
                              << " | Att(quat): " << att.w() << "," << att.x() << "," << att.y() << "," << att.z()
                              << std::endl;
                }
                
                iteration++;
            }
            
            last_time = current_time;
        }
        
        // Small sleep to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\nMock INS Runner stopped gracefully." << std::endl;
    return 0;
}
