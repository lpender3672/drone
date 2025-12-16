#include "gps_interface.h"
#include <iostream>
#include <cstring>
#include <cerrno>
#include <sys/select.h>
#include <sys/time.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <chrono>

// C++14 requires out-of-line definitions for static constexpr members
constexpr uint8_t GPSInterface::UBX_SYNC1;
constexpr uint8_t GPSInterface::UBX_SYNC2;
constexpr uint8_t GPSInterface::UBX_CLASS_NAV;
constexpr uint8_t GPSInterface::UBX_CLASS_RXM;
constexpr uint8_t GPSInterface::UBX_CLASS_INF;
constexpr uint8_t GPSInterface::UBX_CLASS_ACK;
constexpr uint8_t GPSInterface::UBX_CLASS_CFG;
constexpr uint8_t GPSInterface::UBX_NAV_POSLLH;
constexpr uint8_t GPSInterface::UBX_NAV_STATUS;
constexpr uint8_t GPSInterface::UBX_NAV_DOP;
constexpr uint8_t GPSInterface::UBX_NAV_PVT;
constexpr uint8_t GPSInterface::UBX_NAV_VELNED;
constexpr uint8_t GPSInterface::UBX_NAV_TIMEUTC;
constexpr uint8_t GPSInterface::UBX_CFG_PRT;
constexpr uint8_t GPSInterface::UBX_CFG_MSG;
constexpr uint8_t GPSInterface::UBX_CFG_RATE;
constexpr uint8_t GPSInterface::UBX_CFG_NAV5;
constexpr uint8_t GPSInterface::UBX_ACK_NAK;
constexpr uint8_t GPSInterface::UBX_ACK_ACK;
constexpr size_t GPSInterface::RX_BUFFER_SIZE;

GPSInterface::GPSInterface(const std::string& device, int baudrate)
    : device_(device), baudrate_(baudrate), fd_(-1) {
    rx_buffer_.reserve(RX_BUFFER_SIZE);
}

GPSInterface::~GPSInterface() {
    close();
}

bool GPSInterface::open() {
    if (isOpen()) {
        std::cerr << "GPS: Port already open" << std::endl;
        return true;
    }
    
    // Open serial port
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "GPS: Failed to open " << device_ << ": " 
                  << strerror(errno) << std::endl;
        return false;
    }
    
    // Save current serial port settings
    if (tcgetattr(fd_, &tty_old_) != 0) {
        std::cerr << "GPS: Failed to get terminal attributes: " 
                  << strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    // Configure serial port
    if (!configureSerialPort()) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    std::cout << "GPS: Opened " << device_ << " at " << baudrate_ << " baud" << std::endl;
    
    // Give the GPS module time to initialize
    usleep(100000); // 100ms
    
    return true;
}

void GPSInterface::close() {
    if (isOpen()) {
        // Restore old serial port settings
        tcsetattr(fd_, TCSANOW, &tty_old_);
        ::close(fd_);
        fd_ = -1;
        std::cout << "GPS: Closed " << device_ << std::endl;
    }
}

bool GPSInterface::configureSerialPort() {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "GPS: Failed to get terminal attributes: " 
                  << strerror(errno) << std::endl;
        return false;
    }
    
    // Set baud rate
    speed_t speed;
    switch (baudrate_) {
        case 9600:    speed = B9600; break;
        case 19200:   speed = B19200; break;
        case 38400:   speed = B38400; break;
        case 57600:   speed = B57600; break;
        case 115200:  speed = B115200; break;
        case 230400:  speed = B230400; break;
        case 460800:  speed = B460800; break;
        case 921600:  speed = B921600; break;
        default:
            std::cerr << "GPS: Unsupported baud rate: " << baudrate_ << std::endl;
            return false;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // 8N1 mode
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines
    
    // Raw input mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Disable software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // Raw output mode
    tty.c_oflag &= ~OPOST;
    
    // Non-blocking reads with timeout
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 0.1 second timeout
    
    // Apply settings
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "GPS: Failed to set terminal attributes: " 
                  << strerror(errno) << std::endl;
        return false;
    }
    
    // Flush any existing data
    tcflush(fd_, TCIOFLUSH);
    
    return true;
}

int GPSInterface::readBytes(uint8_t* buffer, size_t max_bytes, int timeout_ms) {
    if (!isOpen()) {
        return -1;
    }
    
    fd_set read_fds;
    struct timeval timeout;
    
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);
    
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    
    int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    
    if (ret < 0) {
        std::cerr << "GPS: Select error: " << strerror(errno) << std::endl;
        return -1;
    } else if (ret == 0) {
        // Timeout
        return 0;
    }
    
    ssize_t bytes_read = read(fd_, buffer, max_bytes);
    if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "GPS: Read error: " << strerror(errno) << std::endl;
            return -1;
        }
        return 0;
    }
    
    return bytes_read;
}

int GPSInterface::writeBytes(const uint8_t* buffer, size_t length) {
    if (!isOpen()) {
        return -1;
    }
    
    ssize_t bytes_written = write(fd_, buffer, length);
    if (bytes_written < 0) {
        std::cerr << "GPS: Write error: " << strerror(errno) << std::endl;
        return -1;
    }
    
    return bytes_written;
}

bool GPSInterface::readData(GPSData& data, int timeout_ms) {
    if (!isOpen()) {
        std::cerr << "GPS: Port not open" << std::endl;
        return false;
    }
    
    uint8_t buffer[256];
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        int bytes_read = readBytes(buffer, sizeof(buffer), 100);
        
        if (bytes_read > 0) {
            // Add to receive buffer
            rx_buffer_.insert(rx_buffer_.end(), buffer, buffer + bytes_read);
            
            // Try to parse data
            if (parseData()) {
                data = gps_data_;
                return true;
            }
        }
        
        // Check timeout
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        
        if (elapsed >= timeout_ms) {
            break;
        }
    }
    
    return false;
}

bool GPSInterface::parseData() {
    while (rx_buffer_.size() >= 8) { // Minimum message size
        // Look for UBX sync bytes
        if (rx_buffer_[0] == UBX_SYNC1 && rx_buffer_.size() >= 2 && rx_buffer_[1] == UBX_SYNC2) {
            // UBX message format: SYNC1 SYNC2 CLASS ID LEN_L LEN_H PAYLOAD CK_A CK_B
            if (rx_buffer_.size() >= 6) {
                uint16_t payload_len = rx_buffer_[4] | (rx_buffer_[5] << 8);
                size_t msg_len = 8 + payload_len; // Header(6) + Payload + Checksum(2)
                
                if (rx_buffer_.size() >= msg_len) {
                    // Verify checksum
                    uint8_t ck_a = 0, ck_b = 0;
                    calculateUBXChecksum(&rx_buffer_[2], 4 + payload_len, ck_a, ck_b);
                    
                    if (ck_a == rx_buffer_[6 + payload_len] && 
                        ck_b == rx_buffer_[7 + payload_len]) {
                        // Valid UBX message
                        bool parsed = parseUBX(&rx_buffer_[2], 4 + payload_len);
                        
                        // Remove message from buffer
                        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + msg_len);
                        
                        if (parsed) {
                            return true;
                        }
                    } else {
                        // Invalid checksum, discard sync byte
                        rx_buffer_.erase(rx_buffer_.begin());
                    }
                } else {
                    // Not enough data yet
                    break;
                }
            } else {
                // Not enough data yet
                break;
            }
        }
        // Look for NMEA sentence
        else if (rx_buffer_[0] == '$') {
            // Find end of sentence (CR LF)
            auto it = std::find(rx_buffer_.begin() + 1, rx_buffer_.end(), '\n');
            if (it != rx_buffer_.end()) {
                std::string sentence(rx_buffer_.begin(), it + 1);
                rx_buffer_.erase(rx_buffer_.begin(), it + 1);
                
                if (parseNMEA(sentence)) {
                    return true;
                }
            } else {
                // Not enough data yet, but check for buffer overflow
                if (rx_buffer_.size() > 100) {
                    rx_buffer_.erase(rx_buffer_.begin());
                }
                break;
            }
        }
        // Invalid data, discard byte
        else {
            rx_buffer_.erase(rx_buffer_.begin());
        }
    }
    
    return false;
}

bool GPSInterface::parseUBX(const uint8_t* data, size_t length) {
    if (length < 4) {
        return false;
    }
    
    uint8_t msg_class = data[0];
    uint8_t msg_id = data[1];
    uint16_t payload_len = data[2] | (data[3] << 8);
    
    if (length < 4 + payload_len) {
        return false;
    }
    
    const uint8_t* payload = &data[4];
    
    // Handle different message types
    if (msg_class == UBX_CLASS_NAV && msg_id == UBX_NAV_PVT) {
        return parseUBXNavPVT(payload, payload_len);
    }
    
    return false;
}

bool GPSInterface::parseUBXNavPVT(const uint8_t* payload, size_t length) {
    // UBX-NAV-PVT message is 92 bytes
    if (length < 92) {
        return false;
    }
    
    // Update system timestamp
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    gps_data_.timestamp = ts.tv_sec + ts.tv_nsec / 1e9;
    
    // Parse time (iTOW at offset 0, year at offset 4, etc.)
    uint32_t iTOW = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
    gps_data_.year = payload[4] | (payload[5] << 8);
    gps_data_.month = payload[6];
    gps_data_.day = payload[7];
    gps_data_.hour = payload[8];
    gps_data_.minute = payload[9];
    gps_data_.second = payload[10];
    
    // Valid flags (offset 11)
    uint8_t valid = payload[11];
    gps_data_.valid_date = (valid & 0x01) != 0;
    gps_data_.valid_time = (valid & 0x02) != 0;
    
    // Time accuracy (offset 12)
    uint32_t tAcc = payload[12] | (payload[13] << 8) | (payload[14] << 16) | (payload[15] << 24);
    gps_data_.nanosecond = payload[16] | (payload[17] << 8) | (payload[18] << 16) | (payload[19] << 24);
    
    // Fix type (offset 20)
    gps_data_.fix_type = payload[20];
    
    // Flags (offset 21)
    uint8_t flags = payload[21];
    gps_data_.gnss_fix_ok = (flags & 0x01) != 0;
    gps_data_.diff_correction = (flags & 0x02) != 0;
    
    // Number of satellites (offset 23)
    gps_data_.num_satellites = payload[23];
    
    // Position (WGS84)
    int32_t lon_raw = payload[24] | (payload[25] << 8) | (payload[26] << 16) | (payload[27] << 24);
    int32_t lat_raw = payload[28] | (payload[29] << 8) | (payload[30] << 16) | (payload[31] << 24);
    int32_t height_raw = payload[32] | (payload[33] << 8) | (payload[34] << 16) | (payload[35] << 24);
    int32_t hMSL_raw = payload[36] | (payload[37] << 8) | (payload[38] << 16) | (payload[39] << 24);
    
    gps_data_.longitude = lon_raw * 1e-7; // degrees
    gps_data_.latitude = lat_raw * 1e-7;  // degrees
    gps_data_.altitude_ellipsoid = height_raw * 1e-3; // meters
    gps_data_.altitude_msl = hMSL_raw * 1e-3; // meters
    
    // Position accuracy (offset 40)
    uint32_t hAcc_raw = payload[40] | (payload[41] << 8) | (payload[42] << 16) | (payload[43] << 24);
    uint32_t vAcc_raw = payload[44] | (payload[45] << 8) | (payload[46] << 16) | (payload[47] << 24);
    gps_data_.horizontal_accuracy = hAcc_raw * 1e-3; // meters
    gps_data_.vertical_accuracy = vAcc_raw * 1e-3;   // meters
    
    // Velocity NED (offset 48)
    int32_t velN_raw = payload[48] | (payload[49] << 8) | (payload[50] << 16) | (payload[51] << 24);
    int32_t velE_raw = payload[52] | (payload[53] << 8) | (payload[54] << 16) | (payload[55] << 24);
    int32_t velD_raw = payload[56] | (payload[57] << 8) | (payload[58] << 16) | (payload[59] << 24);
    
    gps_data_.velocity_north = velN_raw * 1e-3; // m/s
    gps_data_.velocity_east = velE_raw * 1e-3;  // m/s
    gps_data_.velocity_down = velD_raw * 1e-3;  // m/s
    
    // Ground speed (offset 60)
    int32_t gSpeed_raw = payload[60] | (payload[61] << 8) | (payload[62] << 16) | (payload[63] << 24);
    gps_data_.ground_speed = gSpeed_raw * 1e-3; // m/s
    
    // Heading of motion (offset 64)
    int32_t headMot_raw = payload[64] | (payload[65] << 8) | (payload[66] << 16) | (payload[67] << 24);
    gps_data_.heading_motion = headMot_raw * 1e-5; // degrees
    
    // Speed accuracy (offset 68)
    uint32_t sAcc_raw = payload[68] | (payload[69] << 8) | (payload[70] << 16) | (payload[71] << 24);
    gps_data_.speed_accuracy = sAcc_raw * 1e-3; // m/s
    
    // Heading accuracy (offset 72)
    uint32_t headAcc_raw = payload[72] | (payload[73] << 8) | (payload[74] << 16) | (payload[75] << 24);
    gps_data_.heading_accuracy = headAcc_raw * 1e-5; // degrees
    
    // PDOP (offset 76)
    uint16_t pDOP_raw = payload[76] | (payload[77] << 8);
    gps_data_.pdop = pDOP_raw * 0.01;
    
    // Set validity flags
    gps_data_.valid_position = gps_data_.gnss_fix_ok && 
                               (gps_data_.fix_type == 2 || gps_data_.fix_type == 3);
    gps_data_.valid_velocity = gps_data_.gnss_fix_ok;
    
    return true;
}

bool GPSInterface::parseNMEA(const std::string& sentence) {
    // Verify checksum
    if (!verifyNMEAChecksum(sentence)) {
        return false;
    }
    
    // Simple NMEA parsing (GGA, RMC, VTG)
    // This is a basic implementation - UBX binary is preferred
    
    if (sentence.find("$GPGGA") == 0 || sentence.find("$GNGGA") == 0) {
        // Parse GGA sentence for position
        // Format: $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
        // This is a simplified parser
        return true; // Placeholder
    }
    
    return false;
}

void GPSInterface::calculateUBXChecksum(const uint8_t* data, size_t length, 
                                       uint8_t& ck_a, uint8_t& ck_b) {
    ck_a = 0;
    ck_b = 0;
    
    for (size_t i = 0; i < length; i++) {
        ck_a += data[i];
        ck_b += ck_a;
    }
}

bool GPSInterface::verifyNMEAChecksum(const std::string& sentence) {
    // Find checksum delimiter
    size_t star_pos = sentence.find('*');
    if (star_pos == std::string::npos || star_pos < 1) {
        return false;
    }
    
    // Calculate checksum (XOR of all characters between $ and *)
    uint8_t checksum = 0;
    for (size_t i = 1; i < star_pos; i++) {
        checksum ^= sentence[i];
    }
    
    // Parse checksum from sentence
    if (star_pos + 2 >= sentence.length()) {
        return false;
    }
    
    uint8_t expected_checksum = std::stoi(sentence.substr(star_pos + 1, 2), nullptr, 16);
    
    return checksum == expected_checksum;
}

bool GPSInterface::sendUBXCommand(uint8_t msg_class, uint8_t msg_id, 
                                 const uint8_t* payload, uint16_t payload_len) {
    if (!isOpen()) {
        return false;
    }
    
    // Build UBX message
    std::vector<uint8_t> message;
    message.reserve(8 + payload_len);
    
    // Sync bytes
    message.push_back(UBX_SYNC1);
    message.push_back(UBX_SYNC2);
    
    // Header
    message.push_back(msg_class);
    message.push_back(msg_id);
    message.push_back(payload_len & 0xFF);
    message.push_back((payload_len >> 8) & 0xFF);
    
    // Payload
    if (payload_len > 0) {
        message.insert(message.end(), payload, payload + payload_len);
    }
    
    // Calculate checksum
    uint8_t ck_a, ck_b;
    calculateUBXChecksum(&message[2], 4 + payload_len, ck_a, ck_b);
    message.push_back(ck_a);
    message.push_back(ck_b);
    
    // Send message
    int bytes_written = writeBytes(message.data(), message.size());
    
    return bytes_written == (int)message.size();
}

bool GPSInterface::setUpdateRate(uint16_t rate_hz) {
    if (rate_hz < 1 || rate_hz > 25) {
        std::cerr << "GPS: Invalid update rate (must be 1-25 Hz)" << std::endl;
        return false;
    }
    
    // UBX-CFG-RATE message
    uint8_t payload[6];
    uint16_t meas_rate = 1000 / rate_hz; // Measurement period in ms
    
    payload[0] = meas_rate & 0xFF;
    payload[1] = (meas_rate >> 8) & 0xFF;
    payload[2] = 0x01; // navRate (1 = every measurement)
    payload[3] = 0x00;
    payload[4] = 0x01; // timeRef (1 = GPS time)
    payload[5] = 0x00;
    
    return sendUBXCommand(UBX_CLASS_CFG, UBX_CFG_RATE, payload, 6);
}

bool GPSInterface::configureUBX(uint8_t msg_class, uint8_t msg_id, uint8_t rate) {
    // UBX-CFG-MSG message
    uint8_t payload[3];
    payload[0] = msg_class;
    payload[1] = msg_id;
    payload[2] = rate;
    
    return sendUBXCommand(UBX_CLASS_CFG, UBX_CFG_MSG, payload, 3);
}

bool GPSInterface::configureNMEA(const std::string& msg_id, bool enable) {
    // This would require mapping NMEA message IDs to UBX configuration
    // For simplicity, not fully implemented here
    std::cerr << "GPS: NMEA configuration not fully implemented" << std::endl;
    return false;
}

bool GPSInterface::pollUBXMessage(uint8_t msg_class, uint8_t msg_id, int timeout_ms) {
    // Send poll request (empty payload)
    if (!sendUBXCommand(msg_class, msg_id, nullptr, 0)) {
        return false;
    }
    
    // Wait for response
    GPSData data;
    return readData(data, timeout_ms);
}
