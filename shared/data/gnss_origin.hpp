#pragma once

namespace shared {

// Geodetic origin used to bridge local-NED truth and the geodetic
// coordinates the navigation filter expects. Owned by the vehicle and
// pushed into every block that needs it (GPS sensor, EKF init, baro
// ground reference, alt-hold setpoint).
struct GnssOrigin {
    double lat_deg = 52.2053;  // Cambridge, UK
    double lon_deg =  0.1218;
    double alt_m   = 10.0;
};

} // namespace shared
