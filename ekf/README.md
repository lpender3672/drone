# Extended Kalman Filter for Inertial Navigation

This project implements an Extended Kalman Filter (ESKF) for inertial navigation systems, supporting both Raspberry Pi (production) and Windows (development/testing) platforms.

## Features

- **15-state ESKF implementation**: Position, velocity, attitude, and sensor biases
- **Curved Earth navigation**: WGS84 ellipsoid with Somigliana gravity model
- **Cross-platform support**: Raspberry Pi (RTIMULib) and Windows (MSYS2/MinGW)
- **Comprehensive unit testing**: Google Test framework with 4 test scenarios
- **Mock data simulation**: Windows-only executable for development

## Project Structure

```
├── CMakeLists.txt          # Cross-platform CMake configuration
├── CMakePresets.json       # Build presets for different platforms
├── eskf.h                  # Core ESKF implementation (header-only)
├── main.cpp                # Raspberry Pi main executable
├── main_mock.cpp           # Windows mock executable
├── ins_data.csv           # Sample INS data
├── tests/
│   └── test_eskf_simple.cpp  # Unit tests
└── README.md
```

## Building

### Prerequisites

**Raspberry Pi:**
```bash
sudo apt update
sudo apt install build-essential cmake libeigen3-dev libgtest-dev
# Install RTIMULib for IMU access
```

**Windows (MSYS2):**
```bash
# Install via MSYS2 package manager
pacman -S mingw-w64-x86_64-cmake mingw-w64-x86_64-ninja
pacman -S mingw-w64-x86_64-eigen3 mingw-w64-x86_64-gtest
```

### Build Commands

**Raspberry Pi:**
```bash
cmake --preset rpi-release
cmake --build --preset rpi-release
```

**Windows Testing:**
```bash
cmake --preset msys2-testing
cmake --build --preset msys2-testing
```

### Running Tests

```bash
# Run unit tests
ctest --preset msys2-testing --verbose

# Run Windows mock executable
./build/msys2-testing/ins_runner_mock.exe
```

## ESKF Implementation Details

### State Vector (15 states)
- **Position (3)**: Latitude, Longitude, Altitude (LLA)
- **Velocity (3)**: North, East, Down (NED frame)
- **Attitude (3)**: Error-state quaternion representation
- **Accelerometer Bias (3)**: X, Y, Z bias estimates
- **Gyroscope Bias (3)**: X, Y, Z bias estimates

### Key Features
- **Error-State Formulation**: Maintains quaternion normalization
- **Curved Earth Model**: WGS84 ellipsoid navigation equations
- **Gravity Modeling**: Somigliana closed-form gravity
- **Sensor Bias Estimation**: First-order Markov process
- **Magnetometer Updates**: Heading correction capability

### Coordinate Frames
- **Body Frame**: IMU sensor axes
- **NED Frame**: North-East-Down navigation frame  
- **LLA**: Latitude-Longitude-Altitude geodetic coordinates

## Test Results

All 4 unit tests pass successfully:

1. **Initialization Test**: Verifies proper filter startup
2. **Stationary Prediction**: Tests zero-motion dynamics
3. **Gravity Prediction**: Validates gravitational acceleration
4. **Attitude Rotation**: Confirms quaternion propagation

```
[==========] Running 4 tests from 1 test suite.
[----------] 4 tests from ESKFTest
[ RUN      ] ESKFTest.Initialization
[       OK ] ESKFTest.Initialization (0 ms)
[ RUN      ] ESKFTest.PredictStationary
[       OK ] ESKFTest.PredictStationary (0 ms)
[ RUN      ] ESKFTest.PredictWithGravity
[       OK ] ESKFTest.PredictWithGravity (0 ms)
[ RUN      ] ESKFTest.PredictAttitudeRotation
[       OK ] ESKFTest.PredictAttitudeRotation (0 ms)
[----------] 4 tests from ESKFTest (0 ms total)
[  PASSED  ] 4 tests.
```

## Usage Example

```cpp
// Initialize sensor parameters
SensorParams params;
params.N_acc = 0.01;    // Accelerometer noise (m/s²)
params.N_gyro = 0.001;  // Gyroscope noise (rad/s)
params.B_acc = 0.005;   // Accelerometer bias (m/s²)
params.B_gyro = 0.0001; // Gyroscope bias (rad/s)
params.tau_acc = 100.0; // Accelerometer correlation time (s)
params.tau_gyro = 100.0; // Gyroscope correlation time (s)

// Initial position (LLA in radians/meters)
Eigen::Vector3d init_pos(lat_rad, lon_rad, alt_meters);

// Create ESKF instance
ESKF eskf(params, init_pos);

// Prediction step
Eigen::Vector3d accel_meas, gyro_meas;
double dt = 0.01; // 100 Hz
eskf.predict(accel_meas, gyro_meas, dt);

// Magnetometer update (optional)
Eigen::Vector3d mag_meas, mag_noise;
eskf.updateMag(mag_meas, mag_noise);

// Get current estimates
Eigen::Vector3d position = eskf.getPositionLLA();
Eigen::Vector3d velocity = eskf.getVelocity();
Eigen::Quaterniond attitude = eskf.getAttitude();
```

## Platform-Specific Notes

### Raspberry Pi
- Uses RTIMULib for hardware IMU access
- Optimized for real-time performance
- Production deployment target

### Windows Development
- MSYS2/MinGW64 toolchain for native development
- Mock IMU data simulation for testing
- Google Test integration for unit testing
- Static linking for portability

## Contributing

This ESKF implementation follows modern C++ practices with header-only design for easy integration. The cross-platform CMake configuration supports both embedded (Raspberry Pi) and desktop (Windows) development workflows.

## References

- Quaternion-based Error State Kalman Filter for INS
- WGS84 Earth Ellipsoid Model
- Somigliana Gravity Formula
- RTIMULib for Raspberry Pi IMU Integration
