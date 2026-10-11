// CRTK <-> AMBF adapter for the RAVEN-II left (gold) arm.
//
// Talks to the AMBF arm directly (base_link_L State / Command) and serves the CRTK topics:
// /arm1/measured_js, /arm1/measured_cp, /arm1/servo_jr, /arm1/servo_cr and /ambf/raven_state.
// Joint topics use the AMBF joint values; poses are in the real RAVEN's frame 0 ("frame0_L") and raven_state
// uses the real RAVEN's joint values.
// Servo commands move a joint goal; every cycle the setpoint sent to AMBF steps toward it at
// 2.5 rad/s (0.5 m/s for the insertion) over the real elapsed time.
#include <chrono>
#include <deque>
#include <iostream>
#include <limits>
#include <string>

#include <ambf_msgs/RigidBodyCmd.h>
#include <ambf_msgs/RigidBodyState.h>
#include <geometry_msgs/TransformStamped.h>
#include <raven_2/raven_state.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "raven2_ambf_crtk_adapter/kinematics.h"

namespace raven2_ambf_crtk
{
using namespace raven_def;

const char *const FRAME_ID = "frame0_L";  // the real RAVEN's frame 0 of the left arm

class Adapter
{
public:
  explicit Adapter(double rate) : rate_(rate) {}

  bool connect();
  void home();
  void run();

private:
  struct Command
  {
    bool cartesian;
    Joints delta;       // servo_jr
    double pos[3];      // servo_cr translation
    double ori[4];      // servo_cr rotation (x, y, z, w)
  };

  // limits applied to a single servo command, and the setpoint speed (per ms)
  const double max_radian_per_ms_ = 0.0025;
  const double max_meters_per_ms_ = 0.0005;
  // largest joint change accepted for a single servo_cr command; anything bigger is not a small move
  const double max_ik_step_m_ = 0.1;
  const double max_ik_step_rad_ = PI;

  void onState(const ambf_msgs::RigidBodyState::ConstPtr &msg);
  void onServoJr(const sensor_msgs::JointState::ConstPtr &msg);
  void onServoCr(const geometry_msgs::TransformStamped::ConstPtr &msg);

  void servoJr(const Joints &delta);
  void servoCr(const double pos[3], const double ori[4]);
  void stepTowardGoal();
  void sendCommand(const Joints &joints);
  void publishMeasured();
  void publishRavenState();

  double rate_;
  ros::NodeHandle nh_;
  ros::Subscriber state_sub_, servo_jr_sub_, servo_cr_sub_;
  ros::Publisher command_pub_, measured_js_pub_, measured_cp_pub_, raven_state_pub_;

  bool have_state_ = false;
  Joints joint_positions_{}, joint_velocities_{};
  Eigen::Matrix4d cartesian_pose_ = Eigen::Matrix4d::Identity();

  Joints setpoint_{};   // sent to AMBF (raven_state jpos_d)
  Joints goal_{};       // where the servo commands want the arm
  std::chrono::steady_clock::time_point last_step_;
  bool stepped_ = false;
  std::deque<Command> commands_;
  int js_limit_count_ = 0;

  // raven_state status, as the real RAVEN reports it: initialising while homing, then pedal down
  // (the simulation has no pedal, so it always takes commands once homed)
  int runlevel_ = RL_INIT;
  int sublevel_ = SL_AUTO_INIT;
  int last_seq_ = 0;  // number of servo commands received

  ambf_msgs::RigidBodyCmd command_;
  uint32_t raven_state_seq_ = 0;
  double old_mpos_[8] = {0};
  std::chrono::steady_clock::time_point last_publish_;
  bool published_ = false;
};

bool Adapter::connect()
{
  // wait for the arm: AMBF takes a moment to load the model
  std::string state_topic;
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(30.0);
  while (ros::ok() && state_topic.empty())
  {
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    for (const auto &t : topics)
    {
      const std::string suffix = "/base_link_L/State";
      if (t.name.size() >= suffix.size() && t.name.compare(t.name.size() - suffix.size(), suffix.size(), suffix) == 0)
        state_topic = t.name;
    }
    if (!state_topic.empty())
      break;
    if (ros::WallTime::now() > deadline)
    {
      std::cerr << "AMBF RAVEN-II not found (no base_link_L topic). Is ambf_simulator running with raven_straight.yaml?"
                << std::endl;
      return false;
    }
    ROS_WARN_THROTTLE(5, "Waiting for AMBF RAVEN-II to load...");
    ros::WallDuration(0.5).sleep();
  }
  if (state_topic.empty())
    return false;

  const auto nodelay = ros::TransportHints().tcpNoDelay();
  const std::string ns = state_topic.substr(0, state_topic.size() - std::string("/State").size());
  state_sub_ = nh_.subscribe(state_topic, 1, &Adapter::onState, this, nodelay);
  command_pub_ = nh_.advertise<ambf_msgs::RigidBodyCmd>(ns + "/Command", 1);

  const ros::WallTime state_deadline = ros::WallTime::now() + ros::WallDuration(5.0);
  while (ros::ok() && !have_state_ && ros::WallTime::now() < state_deadline)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  if (!have_state_)
  {
    std::cerr << "AMBF object not found: base_link_L" << std::endl;
    return false;
  }

  command_.joint_cmds.assign(7, 0.0);
  command_.joint_cmds_types.assign(7, ambf_msgs::RigidBodyCmd::TYPE_POSITION);
  command_.publish_children_names = command_.publish_joint_names = command_.publish_joint_positions = true;

  measured_js_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm1/measured_js", 1, true);
  measured_cp_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/arm1/measured_cp", 1, true);
  raven_state_pub_ = nh_.advertise<raven_2::raven_state>("/ambf/raven_state", 1, true);
  servo_jr_sub_ = nh_.subscribe("/arm1/servo_jr", 100, &Adapter::onServoJr, this, nodelay);
  servo_cr_sub_ = nh_.subscribe("/arm1/servo_cr", 100, &Adapter::onServoCr, this, nodelay);

  ROS_INFO("CRTK to AMBF Adapter initialized and connected to AMBF (%s), running at %.0f Hz.", ns.c_str(), rate_);
  return true;
}

void Adapter::home()
{
  // Ease into HOME along a 2 s minimum-jerk path instead of jumping there: a sudden shoulder/elbow
  // step swings the lightly damped wrist and graspers for seconds.
  const Joints start = joint_positions_;
  const int n = std::max(1, static_cast<int>(2.0 * rate_));
  ros::Rate rate(rate_);
  runlevel_ = RL_INIT;
  sublevel_ = SL_AUTO_INIT;
  setpoint_ = start;
  for (int k = 1; k <= n && ros::ok(); k++)
  {
    ros::spinOnce();
    publishMeasured();
    publishRavenState();
    if (!commands_.empty())
    {
      ROS_WARN_THROTTLE(1, "Homing (runlevel INIT): servo commands are ignored.");
      last_seq_ += static_cast<int>(commands_.size());
      commands_.clear();
    }
    double s = static_cast<double>(k) / n;
    s = 10 * std::pow(s, 3) - 15 * std::pow(s, 4) + 6 * std::pow(s, 5);
    for (int i = 0; i < 7; i++)
      setpoint_[i] = start[i] + s * (HOME[i] - start[i]);
    sendCommand(setpoint_);
    rate.sleep();
  }
  setpoint_ = goal_ = HOME;
  // the real RAVEN leaves the sublevel at SL_AUTO_INIT after initialising (only an E-stop resets it)
  runlevel_ = RL_PEDAL_DN;
  std::cout << "Set Home" << std::endl;
}

void Adapter::run()
{
  ros::Rate rate(rate_);
  while (ros::ok())
  {
    ros::spinOnce();  // AMBF state and queued servo commands
    publishMeasured();
    publishRavenState();

    // apply every command queued since the last cycle, so none is dropped or left waiting
    while (!commands_.empty())
    {
      const Command c = commands_.front();
      commands_.pop_front();
      last_seq_++;
      if (c.cartesian)
        servoCr(c.pos, c.ori);
      else
        servoJr(c.delta);
    }
    stepTowardGoal();
    sendCommand(setpoint_);
    rate.sleep();
  }
}

void Adapter::onState(const ambf_msgs::RigidBodyState::ConstPtr &msg)
{
  if (msg->joint_positions.size() < 7)
    return;
  for (int i = 0; i < 7; i++)
  {
    joint_positions_[i] = msg->joint_positions[i];
    joint_velocities_[i] = i < static_cast<int>(msg->joint_velocities.size()) ? msg->joint_velocities[i] : 0.0;
  }
  have_state_ = true;
}

void Adapter::onServoJr(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (msg->position.size() < 7)
  {
    ROS_WARN("servo_jr needs 7 joint deltas, got %zu. Command ignored.", msg->position.size());
    return;
  }
  Command c{};
  c.cartesian = false;
  for (int i = 0; i < 7; i++)
    c.delta[i] = msg->position[i];
  commands_.push_back(c);
}

void Adapter::onServoCr(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  Command c{};
  c.cartesian = true;
  c.pos[0] = msg->transform.translation.x;
  c.pos[1] = msg->transform.translation.y;
  c.pos[2] = msg->transform.translation.z;
  c.ori[0] = msg->transform.rotation.x;
  c.ori[1] = msg->transform.rotation.y;
  c.ori[2] = msg->transform.rotation.z;
  c.ori[3] = msg->transform.rotation.w;
  commands_.push_back(c);
}

void Adapter::servoJr(const Joints &delta)
{
  bool within_limits = true;
  for (int i = 0; i < 7; i++)
  {
    const double future = goal_[i] + delta[i];
    if (future < JOINT_LIMIT_LOW[i] || future > JOINT_LIMIT_HIGH[i])
    {
      ROS_ERROR("Joint %d will exceed limits: %.3f (min: %.3f, max: %.3f)", i, future, JOINT_LIMIT_LOW[i],
                JOINT_LIMIT_HIGH[i]);
      within_limits = false;
    }
  }
  if (!within_limits)
    return;

  // Deltas accumulate on the goal, not the measured position, so a joint that gets knocked off by
  // the motion of another one is driven back instead of staying off.
  for (int i = 0; i < 7; i++)
  {
    const double limit = i == 2 ? max_meters_per_ms_ : max_radian_per_ms_;
    if (std::abs(delta[i]) <= limit)
    {
      goal_[i] += delta[i];
    }
    else
    {
      goal_[i] += delta[i] > 0 ? limit : -limit;
      if (++js_limit_count_ % 500 == 0)
        ROS_ERROR("THAT WAS TOO FAR! joint %d jpos_d - %f, in %f ", i, delta[i] + goal_[i], delta[i]);
    }
  }
}

void Adapter::servoCr(const double pos[3], const double ori[4])
{
  Eigen::Matrix4d delta = quaternionMatrix(ori);
  delta(0, 3) = pos[0];
  delta(1, 3) = pos[1];
  delta(2, 3) = pos[2];
  const Eigen::Matrix4d target = forwardKinematics(goal_) * delta;

  // Keep the current grasp angle and take the IK branch closest to the current goal.
  Joints joints;
  bool limited = false;
  if (inverseKinematics(target, goal_[5] + goal_[6], goal_, joints, limited) != IkResult::OK)
  {
    ROS_ERROR("No IK solution for the desired cartesian pose (unreachable or at the RCM). Command ignored.");
    return;
  }

  // The tool roll is continuous: use the turn closest to the current goal.
  joints[3] += 2 * PI * std::nearbyint((goal_[3] - joints[3]) / (2 * PI));
  const bool limited_roll = applyJointLimits(joints);

  double step_rad = 0;
  for (int i = 0; i < 7; i++)
    if (i != 2)
      step_rad = std::max(step_rad, std::abs(joints[i] - goal_[i]));
  const double step_m = std::abs(joints[2] - goal_[2]);
  if (step_m > max_ik_step_m_ || step_rad > max_ik_step_rad_)
  {
    ROS_ERROR("Desired cartesian pose needs a joint jump of %.3f. Command ignored.", std::max(step_m, step_rad));
    return;
  }

  // Saturating the joints instead would put the tool somewhere other than where it was sent.
  if (limited || limited_roll)
  {
    ROS_ERROR("Desired cartesian pose is outside the Raven2 joint limits. Command ignored.");
    return;
  }

  // The setpoint follows at the joint speed limits: near the wrist singularity (wrist at +-90 deg, where the
  // grasper axis lines up with the shaft) a small move can need a large tool roll.
  goal_ = joints;
}

void Adapter::stepTowardGoal()
{
  const auto now = std::chrono::steady_clock::now();
  // cap the time step so that a stalled cycle does not turn into a jump
  const double dt_ms =
      stepped_ ? std::min(std::chrono::duration<double, std::milli>(now - last_step_).count(), 20.0) : 1.0;
  last_step_ = now;
  stepped_ = true;

  for (int i = 0; i < 7; i++)
  {
    const double limit = (i == 2 ? max_meters_per_ms_ : max_radian_per_ms_) * dt_ms;
    const double diff = goal_[i] - setpoint_[i];
    setpoint_[i] = std::abs(diff) <= limit ? goal_[i] : setpoint_[i] + (diff > 0 ? limit : -limit);
  }
}

void Adapter::sendCommand(const Joints &joints)
{
  for (int i = 0; i < 7; i++)
    command_.joint_cmds[i] = joints[i];
  command_.header.stamp = ros::Time::now();
  command_pub_.publish(command_);
}

void Adapter::publishMeasured()
{
  cartesian_pose_ = forwardKinematics(joint_positions_);
  const ros::Time now = ros::Time::now();

  sensor_msgs::JointState js;
  js.header.stamp = now;
  js.header.frame_id = "base_link";
  js.position.assign(joint_positions_.begin(), joint_positions_.end());
  js.velocity.assign(joint_velocities_.begin(), joint_velocities_.end());
  js.effort.assign(7, 0.0);
  measured_js_pub_.publish(js);

  geometry_msgs::TransformStamped cp;
  cp.header.stamp = now;
  cp.header.frame_id = FRAME_ID;
  cp.child_frame_id = "end_effector";
  cp.transform.translation.x = cartesian_pose_(0, 3);
  cp.transform.translation.y = cartesian_pose_(1, 3);
  cp.transform.translation.z = cartesian_pose_(2, 3);
  double q[4];
  quaternionFromMatrix(cartesian_pose_, q);
  cp.transform.rotation.x = q[0];
  cp.transform.rotation.y = q[1];
  cp.transform.rotation.z = q[2];
  cp.transform.rotation.w = q[3];
  measured_cp_pub_.publish(cp);
}

void Adapter::publishRavenState()
{
  // raven_state layout: 0-2 shoulder/elbow/insertion, 3 unused, 4-7 tool roll/wrist/graspers; the real RAVEN's
  // joint values, angles in degrees and velocities in degrees/s (the insertion too, as the real RAVEN does)
  const Joints offset = {0, 0, REAL_INSERTION_OFFSET, REAL_TOOL_ROLL_OFFSET, 0, 0, 0};
  double jpos[8] = {0}, jpos_d[8] = {0}, jvel[8] = {0}, mpos[8] = {0}, mpos_d[8] = {0};
  for (int i = 0; i < 7; i++)
  {
    const int k = i < 3 ? i : i + 1;
    jpos[k] = (joint_positions_[i] + offset[i]) * 180.0 / PI;
    jpos_d[k] = (setpoint_[i] + offset[i]) * 180.0 / PI;
    jvel[k] = joint_velocities_[i] * 180.0 / PI;
  }
  for (int k = 0; k < 8; k++)
  {
    mpos[k] = jpos[k] * MOTOR_TR[k];
    mpos_d[k] = jpos_d[k] * MOTOR_TR[k];
  }

  // time since the last publish (0 for the first one, like the real RAVEN)
  const auto now = std::chrono::steady_clock::now();
  const double dt = published_ ? std::chrono::duration<double>(now - last_publish_).count() : 0.0;
  last_publish_ = now;

  raven_2::raven_state msg;
  msg.hdr.seq = raven_state_seq_++;
  msg.hdr.stamp = ros::Time::now();
  msg.hdr.frame_id = FRAME_ID;
  msg.runlevel = runlevel_;
  msg.sublevel = sublevel_;
  msg.last_seq = last_seq_;
  msg.type[0] = GOLD_ARM_SERIAL;
  msg.dt = ros::Duration(dt);

  // pos / ori: the measured tool pose; pos_d / ori_d: the desired one, i.e. the pose of jpos_d
  const Eigen::Matrix4d desired_pose = forwardKinematics(setpoint_);
  for (int i = 0; i < 3; i++)
  {
    msg.pos[i] = static_cast<int>(cartesian_pose_(i, 3) * 1e6);
    msg.pos_d[i] = static_cast<int>(desired_pose(i, 3) * 1e6);
    for (int j = 0; j < 3; j++)
    {
      msg.ori[3 * i + j] = cartesian_pose_(i, j);
      msg.ori_d[3 * i + j] = desired_pose(i, j);
    }
  }
  for (int k = 0; k < 8; k++)
  {
    msg.jpos[k] = jpos[k];
    msg.jpos_d[k] = jpos_d[k];
    msg.jvel[k] = jvel[k];
    msg.mpos[k] = mpos[k];
    msg.mpos_d[k] = mpos_d[k];
    msg.mvel[k] = published_ && dt > 0 ? (mpos[k] - old_mpos_[k]) / dt : 0.0;
    old_mpos_[k] = mpos[k];
  }
  published_ = true;
  // encoder counts as the Python adapter computes them (the linear encoder on index 4)
  for (int k = 0; k < 8; k++)
  {
    if (k == 3)
      continue;
    msg.encVals[k] = static_cast<int>(k == 4 ? jpos[k] * LINEAR_JOINT_ENC_PER_M : jpos[k] * (ROTARY_JOINT_ENC_PER_REV / PI));
  }
  // grasp angle (grasper 1 + grasper 2) in radians
  msg.grasp_d[0] = setpoint_[5] + setpoint_[6];
  raven_state_pub_.publish(msg);
}
}  // namespace raven2_ambf_crtk

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AMBF_raven2_controller", ros::init_options::AnonymousName);
  ros::NodeHandle pnh("~");
  double rate;
  pnh.param("rate", rate, 1000.0);

  raven2_ambf_crtk::Adapter adapter(rate);
  if (!adapter.connect())
    return 1;
  adapter.home();
  adapter.run();
  return 0;
}
