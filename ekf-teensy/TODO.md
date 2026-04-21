# Teensy TODO

## Decouple sensor logging from sensor drivers

`TeensySensorLogger` is mixed into every sensor driver (`BNO055Imu`, `BNO055Mag`,
`BMP280Baro`, `UbloxGnss`). Each driver defines its own bespoke log struct and calls
`saveValueIfEnabled()` directly, coupling hardware drivers to SD card details.

**What to do:**
- Remove `TeensySensorLogger` inheritance from sensor drivers
- Sensors already populate `latest_reading_` (a standard `sensor_readings.h` struct
  with `to_array()`/`from_array()` serialization) — use that as the log payload
- Add a separate logging layer in `main.cpp` (or a dedicated `SensorLogger` class)
  that reads `latest_reading_` from each sensor after update and writes to SD

This keeps drivers responsible for one thing: reading hardware and calling `feed_*`.
