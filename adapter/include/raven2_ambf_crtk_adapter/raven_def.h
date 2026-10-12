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

// Joint values are the AMBF model's: shoulder, elbow, insertion (m), tool roll, wrist, grasper 1, grasper 2.
// They equal the real RAVEN's jpos except for the insertion and the tool roll, which are offset like the
// real RAVEN's joint_states (URDF): real Z_INS = insertion + 0.47 m, real tool roll = tool roll + 45 deg.
const double REAL_INSERTION_OFFSET = 0.47;
const double REAL_TOOL_ROLL_OFFSET = PI / 4;
const Joints REAL_OFFSET = {0, 0, REAL_INSERTION_OFFSET, REAL_TOOL_ROLL_OFFSET, 0, 0, 0};  // real = AMBF + offset

const Joints HOME = {PI / 6, PI / 2, -0.07, -PI / 4, 0, PI / 4, PI / 4};  // the real RAVEN's home pose

// Software joint limits of the real RAVEN with the RAVEN grasper tool (raven2 crtk branch: defines.h, tools.cpp):
// shoulder 0..90 deg, elbow 45..135 deg, Z_INS 0.23..0.56 m (the AMBF model stops at 0.30 m), tool roll
// -182..182 deg, wrist -70..70 deg, graspers -105..105 deg; here in AMBF joint values.
const Joints JOINT_LIMIT_LOW = {0.0, PI / 4, -0.17, -182 * PI / 180 - REAL_TOOL_ROLL_OFFSET,
                                -70 * PI / 180, -105 * PI / 180, -105 * PI / 180};
const Joints JOINT_LIMIT_HIGH = {PI / 2, 3 * PI / 4, 0.56 - REAL_INSERTION_OFFSET, 182 * PI / 180 - REAL_TOOL_ROLL_OFFSET,
                                 70 * PI / 180, 105 * PI / 180, 105 * PI / 180};

// DH parameters; theta (and d for the prismatic joint 3) are the joint variables. The tool joints map to
// DH angles as in the real RAVEN (r2_kinematics.cpp joint2theta): theta4 = real tool roll,
// theta5 = wrist - 90 deg, theta6 = (grasper 2 - grasper 1) / 2.
const double DH_ALPHA[7] = {0, -75 * PI / 180, 128 * PI / 180, 0, PI / 2, PI / 2, 0};
const double DH_A[7] = {0, 0, 0, 0, 0, 0.013, 0};
const double DH_D[7] = {0, 0, 0, -0.0, 0, 0, 0};
const double DH_THETA_INSERTION = PI / 2;
const double DH_WRIST_OFFSET = -PI / 2;
// The chain above starts with Rz(shoulder) Rx(-75 deg) Rz(elbow); the real RAVEN's left arm starts with
// Rz(shoulder + 205 deg) Rx(75 deg) Rz(elbow + 180 deg), which is the same chain turned by 25 deg about the
// shoulder axis. Poses are given in the real RAVEN's frame 0 (origin at the RCM, z along the shoulder axis).
const double DH_BASE_ROTATION = 25 * PI / 180;

// IK constants: sin/cos of alpha_2 (-75 deg) and of the right arm's alpha_3 (52 deg), d4, a5
const double IKIN_PARAM[6] = {std::sin(-75 * PI / 180), std::cos(-75 * PI / 180),
                              std::sin(52 * PI / 180),  std::cos(52 * PI / 180),
                              DH_D[3], DH_A[5]};

// IK singularity tolerance, as in the C++ RAVEN IK (r2_kinematics.cpp)
const double EPS = 1.0e-5;

// Runlevels of the real RAVEN (raven2 include/raven/defines.h)
const int RL_E_STOP = 0, RL_INIT = 1, RL_PEDAL_UP = 2, RL_PEDAL_DN = 3;
const int GOLD_ARM = 0;  // raven_state type of the left arm (crtk branch numbering)

// CRTK interface of the real RAVEN (raven2 crtk branch: crtk_motion_api.cpp, update_device_state.cpp, rt_raven.cpp)
// CRTK base frame <- frame 0 for the gold arm (T_crtk_in_r0_gold): same origin (the RCM), rotated
const double CRTK_BASE_ROTATION[3][3] = {{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
// joint servo commands (servo_jr / jp / jv): largest change of the joint setpoint per 1 ms cycle
const double SERVO_RAD_PER_MS = 0.0025, SERVO_M_PER_MS = 0.00005;
// Cartesian servo commands: largest translation per cycle (servo_cr / cv); servo_cp target checks
const double SERVO_CR_M_PER_MS = 0.1;
const double SERVO_CP_STEP = 0.015, SERVO_CP_MAX_STEP = 0.075;  // clipped to 15 mm from the tool, refused beyond 75 mm
const double SERVO_CP_MAX_JUMP = 0.5, SERVO_CP_MAX_ROTATION = 0.5;  // m and rad from the measured pose
const double WORKSPACE_LOW[3] = {-0.03, -0.10, -0.20}, WORKSPACE_HIGH[3] = {0.13, 0.15, -0.015};  // servo_cp, CRTK base
// cycles without joint commands before Cartesian control (IK) takes over again
const int JOINT_CONTROL_COUNTDOWN = 10;
// the arm reports busy until this many cycles after the last servo command
const int BUSY_CYCLES = 1000;

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
