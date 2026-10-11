// RAVEN-II (left / gold arm) definitions used by the adapter.
#pragma once

#include <array>
#include <cmath>

namespace raven2_ambf_crtk
{
using Joints = std::array<double, 7>;

namespace raven_def
{
const double PI = M_PI;

// shoulder, elbow, insertion (m), tool roll, wrist, grasper 1, grasper 2
const Joints HOME = {PI / 6, PI / 2, -0.07, -PI / 4, 0, PI / 4, PI / 4};
const Joints JOINT_LIMIT_LOW = {0.0, PI / 4, -0.17, -PI * 2, -2, -2, -2};
const Joints JOINT_LIMIT_HIGH = {PI / 2, (PI * 3) / 4, 0.1, PI * 2, 2, 2, 2};

// DH parameters; theta (and d for the prismatic joint 3) are the joint variables
const double DH_ALPHA[7] = {0, -75 * PI / 180, 128 * PI / 180, 0, PI / 2, PI / 2, 0};
const double DH_A[7] = {0, 0, 0, 0, 0, 0.013, 0};
const double DH_D[7] = {0, 0, 0, -0.0, 0, 0, 0};
const double DH_THETA_INSERTION = PI / 2;

// IK constants: sin/cos of alpha_2 (-75 deg) and of the right arm's alpha_3 (52 deg), d4, a5
const double IKIN_PARAM[6] = {std::sin(-75 * PI / 180), std::cos(-75 * PI / 180),
                              std::sin(52 * PI / 180),  std::cos(52 * PI / 180),
                              DH_D[3], DH_A[5]};

// IK singularity tolerance, as in the C++ RAVEN IK (r2_kinematics.cpp)
const double EPS = 1.0e-5;

// motor transmissions and encoders (the GREEN_ARM values the simulated gold arm uses)
const double LINEAR_JOINT_ENC_PER_M = 200000;
const double ROTARY_JOINT_ENC_PER_REV = 2048 * 4;
const double GEAR_BOX_GP42_TR = 49.0 / 4.0, GEAR_BOX_GP32_TR = 26.0 / 7.0;
const double CABLE_RADIUS_1050 = 1.19 / 2.0, CABLE_RADIUS_1024 = 0.61 / 2.0;
const double PARTIAL_PULLEY_LINK1_RADIUS = 62.5 + CABLE_RADIUS_1050;
const double PARTIAL_PULLEY_LINK2_RADIUS = 40.85 + CABLE_RADIUS_1050;
const double CAPSTAN_LINK2_LARGE_RADIUS = 7.82 + CABLE_RADIUS_1050;
const double CAPSTAN_LINK2_SMALL_RADIUS = 5.60 + CABLE_RADIUS_1050;
const double CAPSTAN_RADIUS_GP42 = 11.35 / 2, CAPSTAN_RADIUS_GP32 = 4.7 + CABLE_RADIUS_1024;
const double CAPSTAN_TOOL_RADIUS = 10.6 + CABLE_RADIUS_1024;

// joint -> motor ratio, indexed like raven_state (0-2 shoulder/elbow/insertion, 3 unused, 4-7 tool joints)
const double MOTOR_TR[8] = {
    (PARTIAL_PULLEY_LINK1_RADIUS / CAPSTAN_RADIUS_GP42) * GEAR_BOX_GP42_TR,
    (PARTIAL_PULLEY_LINK2_RADIUS / CAPSTAN_LINK2_SMALL_RADIUS) * (CAPSTAN_LINK2_LARGE_RADIUS / CAPSTAN_RADIUS_GP42) * GEAR_BOX_GP42_TR,
    (1.0 / ((2 * PI * CAPSTAN_RADIUS_GP42) / 1000.0)) * GEAR_BOX_GP42_TR * 2 * PI,
    0,
    (CAPSTAN_TOOL_RADIUS / CAPSTAN_RADIUS_GP32) * GEAR_BOX_GP32_TR * 120.0 / 180.0,
    (CAPSTAN_TOOL_RADIUS / CAPSTAN_RADIUS_GP32) * GEAR_BOX_GP32_TR * 200.0 / 180.0,
    (CAPSTAN_TOOL_RADIUS / CAPSTAN_RADIUS_GP32) * GEAR_BOX_GP32_TR * 100.0 / 90.0,
    (CAPSTAN_TOOL_RADIUS / CAPSTAN_RADIUS_GP32) * GEAR_BOX_GP32_TR * 100.0 / 90.0};
}  // namespace raven_def
}  // namespace raven2_ambf_crtk
