#ifndef SENSOR_IO_H
#define SENSOR_IO_H

#include <fstream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <filesystem>
#include <iostream>

// Structure to hold sensor data for one timestep
struct IMUData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    double timestamp;
    Eigen::Vector3d accel;    // Accelerometer (g)
    Eigen::Vector3d gyro;     // Gyroscope (rad/s)
    Eigen::Vector3d mag;      // Magnetometer (uT)
    double pressure;          // Pressure (hPa)
    
    IMUData() : timestamp(0.0), pressure(0.0) {
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


/**
 * Structure to hold GPS data from Neo M9N module
 */
struct GPSData {
    // Time information
    double timestamp;           // System timestamp (seconds)
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t nanosecond;
    
    // Position (WGS84)
    double latitude;            // degrees
    double longitude;           // degrees
    double altitude_msl;        // meters above mean sea level
    double altitude_ellipsoid;  // meters above ellipsoid
    
    // Velocity
    double velocity_north;      // m/s
    double velocity_east;       // m/s
    double velocity_down;       // m/s
    double ground_speed;        // m/s
    double heading_motion;      // degrees
    
    // Accuracy estimates
    float horizontal_accuracy;  // meters
    float vertical_accuracy;    // meters
    float speed_accuracy;       // m/s
    float heading_accuracy;     // degrees
    
    // Fix information
    uint8_t fix_type;          // 0=no fix, 2=2D, 3=3D, 4=GNSS+DR, 5=Time only
    uint8_t num_satellites;    // Number of satellites used
    float pdop;                // Position dilution of precision
    
    // Status flags
    bool valid_time;
    bool valid_date;
    bool valid_position;
    bool valid_velocity;
    bool gnss_fix_ok;
    bool diff_correction;      // Differential corrections applied
    
    GPSData() {
        timestamp = 0.0;
        year = 0;
        month = 0;
        day = 0;
        hour = 0;
        minute = 0;
        second = 0;
        nanosecond = 0;
        
        latitude = 0.0;
        longitude = 0.0;
        altitude_msl = 0.0;
        altitude_ellipsoid = 0.0;
        
        velocity_north = 0.0;
        velocity_east = 0.0;
        velocity_down = 0.0;
        ground_speed = 0.0;
        heading_motion = 0.0;
        
        horizontal_accuracy = 0.0f;
        vertical_accuracy = 0.0f;
        speed_accuracy = 0.0f;
        heading_accuracy = 0.0f;
        
        fix_type = 0;
        num_satellites = 0;
        pdop = 99.9f;
        
        valid_time = false;
        valid_date = false;
        valid_position = false;
        valid_velocity = false;
        gnss_fix_ok = false;
        diff_correction = false;
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
    
    void write(const IMUData& data) {
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
            throw std::runtime_error("Failed to open file: " + filename);
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
    
    bool read(IMUData& data) {
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

// GPS data CSV writer class
class GPSWriter {
public:
    GPSWriter(const std::string& filename) : filename_(filename) {
        file_.open(filename);
        if (!file_.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }
        
        // Write CSV header
        file_ << "timestamp,latitude,longitude,altitude_msl,altitude_ellipsoid,"
              << "vel_north,vel_east,vel_down,ground_speed,heading,"
              << "h_accuracy,v_accuracy,speed_accuracy,heading_accuracy,"
              << "fix_type,num_sats,pdop,"
              << "year,month,day,hour,minute,second,nanosecond,"
              << "valid_time,valid_date,valid_position,valid_velocity,gnss_fix_ok,diff_correction"
              << std::endl;
    }
    
    ~GPSWriter() {
        if (file_.is_open()) {
            file_.close();
        }
    }
    
    void write(const GPSData& data) {
        if (!file_.is_open()) {
            return;
        }
        
        file_ << std::fixed << std::setprecision(9);
        file_ << data.timestamp << ","
              << data.latitude << ","
              << data.longitude << ","
              << std::setprecision(3)
              << data.altitude_msl << ","
              << data.altitude_ellipsoid << ","
              << std::setprecision(4)
              << data.velocity_north << ","
              << data.velocity_east << ","
              << data.velocity_down << ","
              << data.ground_speed << ","
              << std::setprecision(2)
              << data.heading_motion << ","
              << data.horizontal_accuracy << ","
              << data.vertical_accuracy << ","
              << data.speed_accuracy << ","
              << data.heading_accuracy << ","
              << (int)data.fix_type << ","
              << (int)data.num_satellites << ","
              << data.pdop << ","
              << data.year << ","
              << (int)data.month << ","
              << (int)data.day << ","
              << (int)data.hour << ","
              << (int)data.minute << ","
              << (int)data.second << ","
              << data.nanosecond << ","
              << data.valid_time << ","
              << data.valid_date << ","
              << data.valid_position << ","
              << data.valid_velocity << ","
              << data.gnss_fix_ok << ","
              << data.diff_correction
              << std::endl;
        
        file_.flush();
    }
    
    bool isOpen() const {
        return file_.is_open();
    }

private:
    std::string filename_;
    std::ofstream file_;
};

#endif // SENSOR_IO_H
