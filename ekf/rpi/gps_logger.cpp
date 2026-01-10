// gps_logger.cpp
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include "gps_interface.h"

volatile bool running = true;

void signalHandler(int signum) {
    running = false;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    
    // Parse command line arguments
    std::string device = "/dev/ttyACM0";
    int baudrate = 9600;
    int update_rate_hz = 5; // Default 5 Hz
    std::string output_file = "data/gps_data.csv";
    
    if (argc > 1) {
        device = argv[1];
    }
    if (argc > 2) {
        baudrate = std::stoi(argv[2]);
    }
    if (argc > 3) {
        update_rate_hz = std::stoi(argv[3]);
    }
    if (argc > 4) {
        output_file = argv[4];
    }
    
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "GPS Logger Initialized" << std::endl;
    std::cout << "Device: " << device << std::endl;
    std::cout << "Baud Rate: " << baudrate << std::endl;
    std::cout << "Target Update Rate: " << update_rate_hz << " Hz" << std::endl;
    std::cout << "Output File: " << output_file << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    
    // --- 1. Initialize GPS Interface ---
    GPSInterface gps(device, baudrate);
    
    if (!gps.open()) {
        std::cerr << "Failed to open GPS device: " << device << std::endl;
        std::cerr << "Make sure the device exists and you have permission to access it." << std::endl;
        std::cerr << "Try: sudo usermod -a -G dialout $USER" << std::endl;
        return 1;
    }
    
    std::cout << "GPS device opened successfully" << std::endl;
    
    // --- 2. Configure GPS Module ---
    std::cout << "Configuring GPS module..." << std::endl;
    
    // Set update rate
    if (gps.setUpdateRate(update_rate_hz)) {
        std::cout << "GPS update rate set to " << update_rate_hz << " Hz" << std::endl;
    } else {
        std::cerr << "Warning: Failed to set GPS update rate" << std::endl;
    }
    
    // Enable UBX-NAV-PVT message (most comprehensive)
    if (gps.configureUBX(0x01, 0x07, 1)) { // NAV-PVT
        std::cout << "UBX-NAV-PVT message enabled" << std::endl;
    } else {
        std::cerr << "Warning: Failed to enable UBX-NAV-PVT" << std::endl;
    }
    
    // Give GPS time to apply configuration
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // --- 3. Create GPS Writer ---
    GPSWriter writer(output_file);
    
    if (!writer.isOpen()) {
        std::cerr << "Failed to create output file: " << output_file << std::endl;
        return 1;
    }
    
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "Logging started. Press Ctrl+C to stop." << std::endl;
    std::cout << "Waiting for GPS fix..." << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    
    auto startTime = std::chrono::high_resolution_clock::now();
    int sample_count = 0;
    int valid_fix_count = 0;
    bool first_fix = false;
    
    while (running) {
        auto loopStart = std::chrono::high_resolution_clock::now();
        
        GPSData data;
        if (gps.readData(data, 1000)) { // 1 second timeout
            // Update timestamp relative to start
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                loopStart - startTime).count() / 1000000.0;
            data.timestamp = elapsed;
            
            // Write data to file
            writer.write(data);
            sample_count++;
            
            // Display status information
            if (data.valid_position) {
                valid_fix_count++;
                
                if (!first_fix) {
                    first_fix = true;
                    std::cout << "\n*** GPS FIX ACQUIRED ***" << std::endl;
                }
                
                // Print status every 10 valid samples
                if (valid_fix_count % 10 == 0) {
                    std::cout << "\r[" << sample_count << " samples] "
                              << "Fix: " << (int)data.fix_type << "D, "
                              << "Sats: " << (int)data.num_satellites << ", "
                              << "Pos: " << std::fixed << std::setprecision(7)
                              << data.latitude << ", " << data.longitude << ", "
                              << std::setprecision(1) << data.altitude_msl << "m, "
                              << "H.Acc: " << std::setprecision(2) << data.horizontal_accuracy << "m, "
                              << "V.Acc: " << data.vertical_accuracy << "m    " << std::flush;
                }
            } else {
                // Print "no fix" status every 5 samples
                if (sample_count % 5 == 0) {
                    std::cout << "\r[" << sample_count << " samples] "
                              << "No fix - Sats: " << (int)data.num_satellites
                              << ", PDOP: " << std::fixed << std::setprecision(1) << data.pdop
                              << "    " << std::flush;
                }
            }
        } else {
            // Timeout or read error
            if (sample_count % 10 == 0) {
                std::cout << "\r[" << sample_count << " samples] "
                          << "Waiting for GPS data...    " << std::flush;
            }
        }
        
        // Small sleep to prevent busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "\n\n---------------------------------------" << std::endl;
    std::cout << "Logging stopped." << std::endl;
    std::cout << "Total samples: " << sample_count << std::endl;
    std::cout << "Valid fixes: " << valid_fix_count << std::endl;
    std::cout << "Data saved to: " << output_file << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    
    gps.close();
    
    return 0;
}
