#ifndef GPS_INTERFACE_H
#define GPS_INTERFACE_H

#include <string>
#include <cstdint>
#include <vector>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <ctime>
#include "sensor_io.h"

/**
 * Class for interfacing with u-blox Neo M9N GPS module via serial port
 */
class GPSInterface {
public:
    /**
     * Constructor
     * @param device Serial device path (e.g., "/dev/ttyACM0")
     * @param baudrate Baud rate (default: 9600, M9N supports up to 921600)
     */
    GPSInterface(const std::string& device = "/dev/ttyACM0", int baudrate = 9600);
    
    /**
     * Destructor - closes serial port
     */
    ~GPSInterface();
    
    /**
     * Open and configure the serial port
     * @return true if successful, false otherwise
     */
    bool open();
    
    /**
     * Close the serial port
     */
    void close();
    
    /**
     * Check if the serial port is open
     * @return true if open, false otherwise
     */
    bool isOpen() const { return fd_ >= 0; }
    
    /**
     * Read and parse GPS data
     * @param data Output GPS data structure
     * @param timeout_ms Timeout in milliseconds (default: 1000ms)
     * @return true if new valid data received, false otherwise
     */
    bool readData(GPSData& data, int timeout_ms = 1000);
    
    /**
     * Configure GPS module for specific settings
     * @param rate_hz Update rate in Hz (1-25 Hz for M9N)
     * @return true if successful
     */
    bool setUpdateRate(uint16_t rate_hz);
    
    /**
     * Enable/disable specific NMEA messages
     * @param msg_id NMEA message ID (e.g., "GGA", "RMC", "VTG")
     * @param enable true to enable, false to disable
     * @return true if successful
     */
    bool configureNMEA(const std::string& msg_id, bool enable);
    
    /**
     * Configure UBX binary protocol messages
     * @param msg_class UBX message class
     * @param msg_id UBX message ID
     * @param rate Output rate (0=disable, 1=every solution)
     * @return true if successful
     */
    bool configureUBX(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    
    /**
     * Send raw UBX command to GPS module
     * @param msg_class UBX message class
     * @param msg_id UBX message ID
     * @param payload Payload data
     * @param payload_len Length of payload
     * @return true if successful
     */
    bool sendUBXCommand(uint8_t msg_class, uint8_t msg_id, 
                       const uint8_t* payload, uint16_t payload_len);
    
    /**
     * Get the latest GPS data (non-blocking)
     * @return Reference to internal GPS data structure
     */
    const GPSData& getLatestData() const { return gps_data_; }
    
    /**
     * Poll for a specific UBX message
     * @param msg_class UBX message class to poll
     * @param msg_id UBX message ID to poll
     * @param timeout_ms Timeout in milliseconds
     * @return true if successful
     */
    bool pollUBXMessage(uint8_t msg_class, uint8_t msg_id, int timeout_ms = 1000);

private:
    // Serial port
    std::string device_;
    int baudrate_;
    int fd_;
    struct termios tty_old_;
    
    // GPS data
    GPSData gps_data_;
    
    // Receive buffer
    std::vector<uint8_t> rx_buffer_;
    static constexpr size_t RX_BUFFER_SIZE = 1024;
    
    // UBX protocol constants
    static constexpr uint8_t UBX_SYNC1 = 0xB5;
    static constexpr uint8_t UBX_SYNC2 = 0x62;
    
    // UBX message classes
    static constexpr uint8_t UBX_CLASS_NAV = 0x01;
    static constexpr uint8_t UBX_CLASS_RXM = 0x02;
    static constexpr uint8_t UBX_CLASS_INF = 0x04;
    static constexpr uint8_t UBX_CLASS_ACK = 0x05;
    static constexpr uint8_t UBX_CLASS_CFG = 0x06;
    
    // UBX NAV message IDs
    static constexpr uint8_t UBX_NAV_POSLLH = 0x02;
    static constexpr uint8_t UBX_NAV_STATUS = 0x03;
    static constexpr uint8_t UBX_NAV_DOP = 0x04;
    static constexpr uint8_t UBX_NAV_PVT = 0x07;  // Position Velocity Time (recommended)
    static constexpr uint8_t UBX_NAV_VELNED = 0x12;
    static constexpr uint8_t UBX_NAV_TIMEUTC = 0x21;
    
    // UBX CFG message IDs
    static constexpr uint8_t UBX_CFG_PRT = 0x00;   // Port configuration
    static constexpr uint8_t UBX_CFG_MSG = 0x01;   // Message configuration
    static constexpr uint8_t UBX_CFG_RATE = 0x08;  // Navigation rate
    static constexpr uint8_t UBX_CFG_NAV5 = 0x24;  // Navigation engine settings
    
    // UBX ACK message IDs
    static constexpr uint8_t UBX_ACK_NAK = 0x00;
    static constexpr uint8_t UBX_ACK_ACK = 0x01;
    
    /**
     * Configure serial port settings
     * @return true if successful
     */
    bool configureSerialPort();
    
    /**
     * Parse received data (NMEA or UBX)
     * @return true if complete message parsed
     */
    bool parseData();
    
    /**
     * Parse NMEA sentence
     * @param sentence NMEA sentence string
     * @return true if successfully parsed
     */
    bool parseNMEA(const std::string& sentence);
    
    /**
     * Parse UBX binary message
     * @param data Message data (without sync bytes)
     * @param length Message length
     * @return true if successfully parsed
     */
    bool parseUBX(const uint8_t* data, size_t length);
    
    /**
     * Parse UBX-NAV-PVT message (most comprehensive message)
     * @param payload Payload data
     * @param length Payload length
     * @return true if successfully parsed
     */
    bool parseUBXNavPVT(const uint8_t* payload, size_t length);
    
    /**
     * Calculate UBX checksum (Fletcher algorithm)
     * @param data Message data (class, id, length, payload)
     * @param length Data length
     * @param ck_a Output checksum A
     * @param ck_b Output checksum B
     */
    void calculateUBXChecksum(const uint8_t* data, size_t length, 
                             uint8_t& ck_a, uint8_t& ck_b);
    
    /**
     * Verify NMEA checksum
     * @param sentence NMEA sentence (including $ and checksum)
     * @return true if checksum valid
     */
    bool verifyNMEAChecksum(const std::string& sentence);
    
    /**
     * Read bytes from serial port
     * @param buffer Output buffer
     * @param max_bytes Maximum bytes to read
     * @param timeout_ms Timeout in milliseconds
     * @return Number of bytes read, -1 on error
     */
    int readBytes(uint8_t* buffer, size_t max_bytes, int timeout_ms);
    
    /**
     * Write bytes to serial port
     * @param buffer Data to write
     * @param length Number of bytes to write
     * @return Number of bytes written, -1 on error
     */
    int writeBytes(const uint8_t* buffer, size_t length);
};

#endif // GPS_INTERFACE_H
