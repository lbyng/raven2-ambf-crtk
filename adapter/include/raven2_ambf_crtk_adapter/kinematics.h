// RAVEN-II left arm kinematics with the real RAVEN's conventions (raven2 r2_kinematics.cpp).
#pragma once

#include <Eigen/Dense>

#include "raven2_ambf_crtk_adapter/raven_def.h"

namespace raven2_ambf_crtk
{
// Tool pose (frame 6: origin on the grasper axis, x along the jaws' midline, z the grasper axis) in the real
// RAVEN's frame 0 (origin at the RCM, z along the shoulder axis), for the given joints.
Eigen::Matrix4d forwardKinematics(const Joints &joints);

enum class IkResult
{
  OK,
  RCM_SINGULARITY,  // the wrist is at the remote center of motion
  NO_SOLUTION       // unreachable pose
};

// Inverse kinematics: the solution closest to `current` (compared in DH space) for `pose`, a tool pose in frame 0
// like forwardKinematics returns, with the given grasp angle (grasper 1 + grasper 2). `limited` is true when a
// joint had to be clamped to its limit.
IkResult inverseKinematics(const Eigen::Matrix4d &pose, double grasp, const Joints &current, Joints &joints,
                           bool &limited);

// Clamps the joints to RAVEN_JOINT_LIMITS; returns true when one was clamped.
bool applyJointLimits(Joints &joints);

// tf.transformations.quaternion_matrix / quaternion_from_matrix, quaternions as (x, y, z, w).
Eigen::Matrix4d quaternionMatrix(const double q[4]);
void quaternionFromMatrix(const Eigen::Matrix4d &m, double q[4]);

// Wraps an angle into [-pi, pi] the way the Python code does.
double wrapToPi(double angle);
}  // namespace raven2_ambf_crtk
