#ifndef SENSOR_IO_H
#define SENSOR_IO_H

#include <fstream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <filesystem>

// Structure to hold sensor data for one timestep
struct SensorData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    double timestamp;
    Eigen::Vector3d accel;    // Accelerometer (g)
    Eigen::Vector3d gyro;     // Gyroscope (rad/s)
    Eigen::Vector3d mag;      // Magnetometer (uT)
    double pressure;          // Pressure (hPa)
    
    SensorData() : timestamp(0.0), pressure(0.0) {
        accel.setZero();
        gyro.setZero();
        mag.setZero();
    }
};

// Structure to hold INS output data for one timestep
struct INSData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    double timestamp;
    Eigen::Vector3d position_lla;  // [lat, lon, alt]
    Eigen::Vector3d velocity;      // [vN, vE, vD]
    Eigen::Quaterniond attitude;   // Quaternion [w, x, y, z]
    Eigen::Vector3d accel_bias;
    Eigen::Vector3d gyro_bias;
    
    INSData() : timestamp(0.0) {
        position_lla.setZero();
        velocity.setZero();
        attitude.setIdentity();
        accel_bias.setZero();
        gyro_bias.setZero();
    }
};

// Class for writing sensor data to CSV
class SensorWriter {
public:
    SensorWriter(const std::string& filename) {
        file_.open(filename);
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        // Write header
        file_ << "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
              << "mag_x,mag_y,mag_z,pressure" << std::endl;
        count_ = 0;
    }
    
    ~SensorWriter() {
        if (file_.is_open()) {
            file_.close();
        }
    }
    
    void write(const SensorData& data) {
        file_ << std::fixed << std::setprecision(6) 
              << data.timestamp << ","
              << data.accel.x() << "," << data.accel.y() << "," << data.accel.z() << ","
              << data.gyro.x() << "," << data.gyro.y() << "," << data.gyro.z() << ","
              << data.mag.x() << "," << data.mag.y() << "," << data.mag.z() << ","
              << data.pressure << std::endl;
        
        // Flush periodically
        if (++count_ % 100 == 0) {
            file_.flush();
        }
    }
    
    void flush() {
        file_.flush();
    }
    
private:
    std::ofstream file_;
    int count_;
};

// Class for reading sensor data from CSV
class SensorReader {
public:
    SensorReader(const std::string& filename) {
        file_.open(filename);
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open file: " + 
                         std::filesystem::absolute(filename).string());
        }
        // Skip header line
        std::string header;
        std::getline(file_, header);
    }
    
    ~SensorReader() {
        if (file_.is_open()) {
            file_.close();
        }
    }
    
    bool read(SensorData& data) {
        std::string line;
        if (!std::getline(file_, line)) {
            return false;
        }
        
        std::stringstream ss(line);
        std::string field;
        std::vector<double> values;
        
        while (std::getline(ss, field, ',')) {
            values.push_back(std::stod(field));
        }
        
        if (values.size() != 11) {
            return false;
        }
        
        data.timestamp = values[0];
        data.accel << values[1], values[2], values[3];
        data.gyro << values[4], values[5], values[6];
        data.mag << values[7], values[8], values[9];
        data.pressure = values[10];
        
        return true;
    }
    
    bool hasMore() const {
        return !file_.eof();
    }
    
private:
    std::ifstream file_;
};

// Class for writing INS output data to CSV
class INSWriter {
public:
    INSWriter(const std::string& filename) {
        file_.open(filename);
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        // Write header
        file_ << "timestamp,lat,lon,alt,vN,vE,vD,qw,qx,qy,qz,"
              << "ba_x,ba_y,ba_z,bg_x,bg_y,bg_z" << std::endl;
        count_ = 0;
    }
    
    ~INSWriter() {
        if (file_.is_open()) {
            file_.close();
        }
    }
    
    void write(const INSData& data) {
        file_ << std::fixed << std::setprecision(9) 
              << data.timestamp << ","
              << data.position_lla.x() << "," << data.position_lla.y() << "," << data.position_lla.z() << ","
              << data.velocity.x() << "," << data.velocity.y() << "," << data.velocity.z() << ","
              << data.attitude.w() << "," << data.attitude.x() << "," 
              << data.attitude.y() << "," << data.attitude.z() << ","
              << data.accel_bias.x() << "," << data.accel_bias.y() << "," << data.accel_bias.z() << ","
              << data.gyro_bias.x() << "," << data.gyro_bias.y() << "," << data.gyro_bias.z() 
              << std::endl;
        
        // Flush periodically
        if (++count_ % 100 == 0) {
            file_.flush();
        }
    }
    
    void flush() {
        file_.flush();
    }
    
private:
    std::ofstream file_;
    int count_;
};

// Class for reading INS output data from CSV
class INSReader {
public:
    INSReader(const std::string& filename) {
        file_.open(filename);
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        // Skip header line
        std::string header;
        std::getline(file_, header);
    }
    
    ~INSReader() {
        if (file_.is_open()) {
            file_.close();
        }
    }
    
    bool read(INSData& data) {
        std::string line;
        if (!std::getline(file_, line)) {
            return false;
        }
        
        std::stringstream ss(line);
        std::string field;
        std::vector<double> values;
        
        while (std::getline(ss, field, ',')) {
            values.push_back(std::stod(field));
        }
        
        if (values.size() != 17) {
            return false;
        }
        
        data.timestamp = values[0];
        data.position_lla << values[1], values[2], values[3];
        data.velocity << values[4], values[5], values[6];
        data.attitude = Eigen::Quaterniond(values[7], values[8], values[9], values[10]);
        data.accel_bias << values[11], values[12], values[13];
        data.gyro_bias << values[14], values[15], values[16];
        
        return true;
    }
    
    bool hasMore() const {
        return !file_.eof();
    }
    
private:
    std::ifstream file_;
};

#endif // SENSOR_IO_H
