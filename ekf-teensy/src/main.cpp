#include <Eigen/Dense>
#include <eskf.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

const double G_ACCEL = 9.80665;

extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern char *__brkval;

int freeram() {
  return (char *)&_heap_end - __brkval;
}

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
// GPS
SFE_UBLOX_GNSS myGNSS;

// EKF instance
EsEkf ekf;

// Timing variables
uint32_t lastIMUTime = 0;
uint32_t lastGPSTime = 0;
uint32_t lastMagTime = 0;
uint32_t lastBaroTime = 0;

// Sensor delays (milliseconds)
const uint32_t GPS_DELAY_MS = 200;
const uint32_t MAG_DELAY_MS = 30;
const uint32_t BARO_DELAY_MS = 100;

// GPS reference position (set on first fix)
float latRef = 0.0f;
float lonRef = 0.0f;
float altRef = 0.0f;
bool gpsRefSet = false;

// Execution time tracking variables
uint32_t imuExecutionTime = 0;
uint32_t gpsExecutionTime = 0;
uint32_t magExecutionTime = 0;
uint32_t baroExecutionTime = 0;
uint32_t totalExecutionTime = 0;
uint32_t maxImuTime = 0;
uint32_t maxGpsTime = 0;
uint32_t maxMagTime = 0;
uint32_t maxBaroTime = 0;

void ekfDebug(const char* label) {
    Serial.printf("%s - free memory: %i\n", label, freeram());
    Serial.flush();
}

void setup() {
    Serial.begin(115200);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV128);

    pinMode(10, OUTPUT); // CS
    pinMode(9, OUTPUT);  // DC
    pinMode(8, OUTPUT);  // RST
    
    // Reset sequence
    digitalWrite(8, LOW);
    delay(10);
    digitalWrite(8, HIGH);
    delay(10);
    
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV64);
    

    // Initialize your sensors here
    // initIMU();
    // initGPS();
    // initMag();
    // initBaro();
    
    Serial.println("EKF initialized");

    // Initialize IMU
    Wire.begin();
    Wire.setClock(400000);

    if (!bno.begin()) {
        Serial.println("Could not find a valid BNO055 sensor, check wiring!");
        while (1) {
            delay(1000);
            Serial.println("BNO055 init failed - retrying...");
        }
    }

    delay(1000);
    bno.setExtCrystalUse(true);
    bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_AMG);

    Serial.println("BNO055 initialized successfully");

    // Initialize GPS
    if (myGNSS.begin() == false) {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.setNavigationFrequency(25);
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save (only) the communications port settings to flash and BBR
    
    Serial.println("u-blox GNSS initialized successfully");


    ekf.initialize(Eigen::Vector3d(0,0,0),
                  Eigen::Vector3d(0,0,0),
                  Eigen::Quaterniond(1,0,0,0),
                  Eigen::Vector3d(0,0,0),
                  Eigen::Vector3d(0,0,0),
                  0.0,
                  Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>::Identity() * 0.1);

    ekf.debugCallback = ekfDebug;
}

void updateIMU() {
    uint32_t startTime = micros();

    sensors_event_t angVelData, accelData;
    bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_GRAVITY);
    
    // For now, using dummy data
    ImuMeasurement msg;
    msg.t = 0.0;
    msg.acc = Eigen::Vector3d(accelData.acceleration.x, accelData.acceleration.y, accelData.acceleration.z) / G_ACCEL;
    msg.gyro = Eigen::Vector3d(angVelData.gyro.x, angVelData.gyro.y, angVelData.gyro.z);

    // Calculate dt
    static uint32_t lastIMUUpdate = 0;
    float dt = (millis() - lastIMUUpdate) * 0.001f;
    lastIMUUpdate = millis();
    
    if (dt > 0.0f && dt < 0.1f) {  // Sanity check
        // Update EKF with IMU data
        ekf.predict(msg, dt);
    }
    
    // Calculate execution time
    uint32_t endTime = micros();
    imuExecutionTime = endTime - startTime;
    if (imuExecutionTime > maxImuTime) {
        maxImuTime = imuExecutionTime;
    }
}

void updateGPS() {
    uint32_t startTime = micros();
    
    // Check if GPS data is available
    if (myGNSS.getPVT() == false) {
        // No new data available
        return;
    }
    
    // Read GPS data
    float lat, lon, alt;     // degrees, degrees, meters
    float velN, velE, velD;  // m/s
    uint8_t fixType, numSats;         // 0=no fix, 3=3D fix
    
    lat = myGNSS.getLatitude() * 1e-7;
    lon = myGNSS.getLongitude() * 1e-7;
    alt = myGNSS.getAltitude() * 1e-3;
    velN = myGNSS.getNedNorthVel() * 1e-3;
    velE = myGNSS.getNedEastVel() * 1e-3;
    velD = myGNSS.getNedDownVel() * 1e-3;
    fixType = myGNSS.getFixType();
    numSats = myGNSS.getSIV();

    //Serial.print("GPS Data: ");
    Serial.print("Lat: "); Serial.print(lat, 6); Serial.print(", ");
    Serial.print("Lon: "); Serial.print(lon, 6); Serial.print(", ");
    Serial.print("Alt: "); Serial.print(alt, 2); Serial.print(" m, ");
    Serial.print("VelN: "); Serial.print(velN, 2); Serial.print(" m/s, ");
    Serial.print("VelE: "); Serial.print(velE, 2); Serial.print(" m/s, ");
    Serial.print("VelD: "); Serial.print(velD, 2); Serial.print(" m/s, ");
    Serial.print("FixType: "); Serial.print(fixType); Serial.print(", ");
    Serial.print("NumSats: "); Serial.println(numSats);
    
    if (fixType >= 3) {

    }
    
    // Calculate execution time
    uint32_t endTime = micros();
    gpsExecutionTime = endTime - startTime;
    if (gpsExecutionTime > maxGpsTime) {
        maxGpsTime = gpsExecutionTime;
    }
}

void updateMag() {
    uint32_t startTime = micros();

    sensors_event_t magData;
    bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    
    // Read magnetometer data
    float mx, my, mz;  // milligauss
    
    mx = magData.magnetic.x; // convert uT to milligauss
    my = magData.magnetic.y;
    mz = magData.magnetic.z;

    
    // Calculate execution time
    uint32_t endTime = micros();
    magExecutionTime = endTime - startTime;
    if (magExecutionTime > maxMagTime) {
        maxMagTime = magExecutionTime;
    }
}

void updateBaro() {
    uint32_t startTime = micros();
    
    
    // Calculate execution time
    uint32_t endTime = micros();
    baroExecutionTime = endTime - startTime;
    if (baroExecutionTime > maxBaroTime) {
        maxBaroTime = baroExecutionTime;
    }
}

void outputState() {
    static uint32_t lastOutput = 0;
    
    // Output at 10Hz
    if (millis() - lastOutput < 100) return;
    lastOutput = millis();
    
}

void outputTimings() {
    Serial.print("IMU Execution Time: "); Serial.print(imuExecutionTime); Serial.print(" us, Max: "); Serial.println(maxImuTime);
    Serial.print("GPS Execution Time: "); Serial.print(gpsExecutionTime); Serial.print(" us, Max: "); Serial.println(maxGpsTime);
    Serial.print("Magnetometer Execution Time: "); Serial.print(magExecutionTime); Serial.print(" us, Max: "); Serial.println(maxMagTime);
    Serial.print("Barometer Execution Time: "); Serial.print(baroExecutionTime); Serial.print(" us, Max: "); Serial.println(maxBaroTime);
    totalExecutionTime = imuExecutionTime + gpsExecutionTime + magExecutionTime + baroExecutionTime;
    Serial.print("Total Execution Time: "); Serial.print(totalExecutionTime); Serial.println(" us");
}

void loop() {
    uint32_t currentTime = millis();
    
    // Update IMU at highest rate (example: 100Hz)
    if (currentTime - lastIMUTime >= 10) {
        updateIMU();
        lastIMUTime = currentTime;
    }
    
    // Update GPS at lower rate (example: 10Hz)
    if (currentTime - lastGPSTime >= 100) {
        updateGPS();
        lastGPSTime = currentTime;
    }
    
    // Update Magnetometer (example: 50Hz)
    if (currentTime - lastMagTime >= 20) {
        updateMag();
        lastMagTime = currentTime;
    }
    
    // Update Barometer (example: 20Hz)
    if (currentTime - lastBaroTime >= 50) {
        updateBaro();
        lastBaroTime = currentTime;
    }
    
    // Output current state
    //outputState();
    //outputTimings();

}
