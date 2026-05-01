#pragma once

#include "../../../shared/data/state.hpp"

// Data types moved to shared/data/state.hpp so the embedded target can
// use the same Scalar/PidInput/MotorEfforts/etc. types. This header
// re-exports them under `namespace sim` so existing sim code keeps
// compiling unchanged.

namespace sim {
using shared::Vec2;
using shared::Vec3;
using shared::Vec4;
using shared::Mat3;
using shared::Mat4;
using shared::Quat;

using shared::Scalar;
using shared::MotorOutput;
using shared::PidInput;
using shared::MotorEfforts;
using shared::AttitudeReference;
using shared::BodyTorque;
using shared::NedForce;
using shared::NoInput;
} // namespace sim
