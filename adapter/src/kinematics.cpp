// RAVEN-II left arm kinematics, ported step by step from raven_fk.py / raven_ik.py.
#include "raven2_ambf_crtk_adapter/kinematics.h"

#include <limits>

namespace raven2_ambf_crtk
{
using namespace raven_def;

namespace
{
const Eigen::Matrix4d T_B0 =
    (Eigen::Matrix4d() << 0, 0, 1, 0.30071, 0, -1, 0, 0.061, 1, 0, 0, -0.007, 0, 0, 0, 1).finished();

Eigen::Matrix4d link(double alpha, double theta, double a, double d)
{
  Eigen::Matrix4d m;
  m << std::cos(theta), -std::sin(theta), 0, a,
       std::sin(theta) * std::cos(alpha), std::cos(theta) * std::cos(alpha), -std::sin(alpha), -std::sin(alpha) * d,
       std::sin(theta) * std::sin(alpha), std::cos(theta) * std::sin(alpha), std::cos(alpha), std::cos(alpha) * d,
       0, 0, 0, 1;
  return m;
}

// Product of the link transforms a .. b-1, associated from the right like the recursive Python version.
Eigen::Matrix4d fwdTrans(int a, int b, const double *alpha, const double *theta, const double *dh_a, const double *d)
{
  Eigen::Matrix4d xf = link(alpha[a], theta[a], dh_a[a], d[a]);
  if (b > a + 1)
    xf = xf * fwdTrans(a + 1, b, alpha, theta, dh_a, d);
  return xf;
}

// joints -> DH values: the grasper difference for index 5, angles wrapped, insertion as is
void jointToDH(const Joints &joint, double dh[7])
{
  for (int i = 0; i < 7; i++)
  {
    if (i == 2)
      dh[i] = joint[i];
    else if (i == 5)
      dh[i] = joint[5] - joint[6];
    else
      dh[i] = wrapToPi(joint[i]);
  }
}

// DH values -> joints, splitting the grasper difference around the grasp angle
Joints dhToJoint(const double dh[7], double grasp)
{
  Joints joint{};
  for (int i = 0; i < 6; i++)
  {
    if (i == 2)
    {
      joint[i] = dh[i];
      continue;
    }
    if (i == 5)
    {
      joint[6] = (-dh[5] + grasp) / 2;
      joint[5] = (dh[5] + grasp) / 2;
    }
    else
    {
      joint[i] = dh[i];
    }
    joint[i] = wrapToPi(joint[i]);
  }
  return joint;
}
}  // namespace

double wrapToPi(double angle)
{
  if (!std::isfinite(angle))
    return angle;
  while (angle > PI)
    angle -= 2 * PI;
  while (angle < -PI)
    angle += 2 * PI;
  return angle;
}

Eigen::Matrix4d forwardKinematics(const Joints &joints)
{
  double dh[7], alpha[7], theta[7], a[7], d[7];
  jointToDH(joints, dh);
  for (int i = 0; i < 7; i++)
  {
    theta[i] = (i == 2) ? DH_THETA_INSERTION : dh[i];
    d[i] = (i == 2) ? dh[i] : DH_D[i];
    alpha[i] = DH_ALPHA[i];
    a[i] = DH_A[i];
  }
  return T_B0 * fwdTrans(0, 6, alpha, theta, a, d);
}

bool applyJointLimits(Joints &joints)
{
  bool limited = false;
  for (int i = 0; i < 7; i++)
  {
    if (joints[i] < JOINT_LIMIT_LOW[i])
    {
      joints[i] = JOINT_LIMIT_LOW[i];
      limited = true;
    }
    else if (joints[i] > JOINT_LIMIT_HIGH[i])
    {
      joints[i] = JOINT_LIMIT_HIGH[i];
      limited = true;
    }
  }
  return limited;
}

IkResult inverseKinematics(const Eigen::Matrix4d &pose, double grasp, const Joints &current, Joints &joints,
                           bool &limited)
{
  const double *P = IKIN_PARAM;
  const Eigen::Matrix4d T06 = T_B0.inverse() * pose;

  double iksol[8][7] = {};
  bool ikcheck[8];
  for (bool &c : ikcheck)
    c = true;

  // DH constants; theta 0, 1 (and 3, 4 for the singular wrist) and d 2 are filled per solution
  double dh_alpha[6], dh_theta[6], dh_d[6], dh_a[6];
  for (int i = 0; i < 6; i++)
  {
    dh_alpha[i] = DH_ALPHA[i];
    dh_theta[i] = (i == 2) ? DH_THETA_INSERTION : 0;
    dh_d[i] = DH_D[i];
    dh_a[i] = DH_A[i];
  }

  // Step 1: the wrist (frame 5) position, two candidates
  const Eigen::Matrix4d T60 = T06.inverse();
  Eigen::Vector4d p6rcm(T60(0, 3), T60(1, 3), 0, 0);
  const double norm = p6rcm.norm();
  if (!(norm > 0))
    return IkResult::NO_SOLUTION;
  Eigen::Vector4d p05[8];
  for (int i = 0; i < 2; i++)
  {
    Eigen::Vector4d p65 = (-1 + 2 * i) * P[5] * (p6rcm / norm);
    p65(3) = 1;
    const Eigen::Vector4d p = T06 * p65;
    for (int k = 0; k < 4; k++)
      p05[4 * i + k] = Eigen::Vector4d(p(0), p(1), p(2), 1);
  }

  // Step 2: the insertion, two signs per candidate
  for (int i = 0; i < 2; i++)
  {
    const double insertion = p05[4 * i].head<3>().norm();
    if (insertion <= P[5])
      return IkResult::RCM_SINGULARITY;  // like the C++ RAVEN IK, the whole IK fails here
    iksol[4 * i + 0][2] = iksol[4 * i + 1][2] = -P[4] - insertion;
    iksol[4 * i + 2][2] = iksol[4 * i + 3][2] = -P[4] + insertion;
  }

  // Step 3: theta 2 (elbow), two signs
  for (int i = 0; i < 8; i += 2)
  {
    const double z0p5 = p05[i](2);
    const double d = iksol[i][2] + P[4];
    double cth2 = -((z0p5 / d) + P[1] * P[3]) / (P[0] * P[2]);
    if (cth2 > 1 && cth2 < 1 + EPS)
      cth2 = 1;
    else if (cth2 < -1 && cth2 > -1 - EPS)
      cth2 = -1;
    if (cth2 > 1 || cth2 < -1)
    {
      ikcheck[i] = ikcheck[i + 1] = false;
    }
    else
    {
      iksol[i][1] = std::acos(cth2);
      iksol[i + 1][1] = -std::acos(cth2);
    }
  }

  // Step 4: theta 1 (shoulder)
  for (int i = 0; i < 8; i++)
  {
    if (!ikcheck[i])
      continue;
    const double cth2 = std::cos(iksol[i][1]), sth2 = std::sin(iksol[i][1]);
    const double d = iksol[i][2] + P[4];
    const double BB1 = sth2 * P[2];
    const double BB2 = cth2 * P[1] * P[2] - P[0] * P[3];
    Eigen::Matrix3d Bmx;
    Bmx << BB1, BB2, 0, -BB2, BB1, 0, 0, 0, 1;
    const Eigen::Vector3d scth1 = Bmx.inverse() * Eigen::Vector3d(p05[i](0), p05[i](1), 0) * (1 / d);
    iksol[i][0] = std::atan2(scth1(1), scth1(0));
  }

  // Step 5: theta 4, 5, 6 (tool roll, wrist, grasper difference)
  for (int i = 0; i < 8; i++)
  {
    if (!ikcheck[i])
      continue;
    dh_theta[0] = iksol[i][0];
    dh_theta[1] = iksol[i][1];
    dh_d[2] = iksol[i][2];
    const Eigen::Matrix4d T03 = fwdTrans(0, 3, dh_alpha, dh_theta, dh_a, dh_d);
    const Eigen::Matrix4d T36 = T03.inverse() * T06;
    const double c5 = -T36(2, 2);
    const double s5 = (T36(2, 3) - P[4]) / P[5];
    double c4, s4;
    if (std::fabs(c5) > EPS)
    {
      c4 = T36(0, 3) / (P[5] * c5);
      s4 = T36(1, 3) / (P[5] * c5);
    }
    else
    {
      c4 = T36(0, 2) / s5;
      s4 = T36(1, 2) / s5;
    }
    iksol[i][3] = std::atan2(s4, c4);
    iksol[i][4] = std::atan2(s5, c5);
    double c6, s6;
    if (std::fabs(s5) > EPS)
    {
      c6 = T36(2, 0) / s5;
      s6 = -T36(2, 1) / s5;
    }
    else
    {
      dh_theta[3] = iksol[i][3];
      dh_theta[4] = iksol[i][4];
      const Eigen::Matrix4d T05 = T03 * fwdTrans(3, 5, dh_alpha, dh_theta, dh_a, dh_d);
      const Eigen::Matrix4d T56 = T05.inverse() * T06;
      c6 = T56(0, 0);
      s6 = T56(2, 0);
    }
    iksol[i][5] = std::atan2(s6, c6);
  }

  // the valid solution closest to the current joints, compared in DH space
  double curr[7];
  jointToDH(current, curr);
  double best_err = 1e10;
  int best = -1;
  for (int i = 0; i < 8; i++)
  {
    if (!ikcheck[i])
      continue;
    double error = 0;
    for (int j = 0; j < 6; j++)
    {
      if (j == 2)
        error += 100 * std::pow(iksol[i][j] - curr[j], 2);
      else
        error += std::pow(wrapToPi(iksol[i][j] - curr[j]), 2);
    }
    if (error < best_err)
    {
      best_err = error;
      best = i;
    }
  }
  if (best < 0)
    return IkResult::NO_SOLUTION;

  joints = dhToJoint(iksol[best], grasp);
  limited = applyJointLimits(joints);
  return IkResult::OK;
}

Eigen::Matrix4d quaternionMatrix(const double quaternion[4])
{
  double q[4] = {quaternion[0], quaternion[1], quaternion[2], quaternion[3]};
  const double nq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  if (nq < 4 * std::numeric_limits<double>::epsilon())
    return Eigen::Matrix4d::Identity();
  const double s = std::sqrt(2.0 / nq);
  for (double &v : q)
    v *= s;
  double o[4][4];
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++)
      o[r][c] = q[r] * q[c];
  Eigen::Matrix4d m;
  m << 1.0 - o[1][1] - o[2][2], o[0][1] - o[2][3], o[0][2] + o[1][3], 0,
       o[0][1] + o[2][3], 1.0 - o[0][0] - o[2][2], o[1][2] - o[0][3], 0,
       o[0][2] - o[1][3], o[1][2] + o[0][3], 1.0 - o[0][0] - o[1][1], 0,
       0, 0, 0, 1;
  return m;
}

void quaternionFromMatrix(const Eigen::Matrix4d &M, double q[4])
{
  double t = M.trace();
  if (t > M(3, 3))
  {
    q[3] = t;
    q[2] = M(1, 0) - M(0, 1);
    q[1] = M(0, 2) - M(2, 0);
    q[0] = M(2, 1) - M(1, 2);
  }
  else
  {
    int i = 0, j = 1, k = 2;
    if (M(1, 1) > M(0, 0))
    {
      i = 1; j = 2; k = 0;
    }
    if (M(2, 2) > M(i, i))
    {
      i = 2; j = 0; k = 1;
    }
    t = M(i, i) - (M(j, j) + M(k, k)) + M(3, 3);
    q[i] = t;
    q[j] = M(i, j) + M(j, i);
    q[k] = M(k, i) + M(i, k);
    q[3] = M(k, j) - M(j, k);
  }
  const double s = 0.5 / std::sqrt(t * M(3, 3));
  for (int n = 0; n < 4; n++)
    q[n] *= s;
}
}  // namespace raven2_ambf_crtk
