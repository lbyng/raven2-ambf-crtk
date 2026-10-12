// CRTK <-> AMBF adapter for the RAVEN-II left (gold) arm.
//
// Serves the CRTK interface of the real RAVEN-II software (the raven2 crtk branch, which recorded the real RAVEN
// data this simulation is compared with) for arm1 / grasp1, and its /ravenstate topic, on top of the AMBF arm
// (base_link_L State / Command):
//   published:  /arm1/{measured,setpoint}_{js,cp}, /grasp1/measured_js, /arm1/operating_state, /operating_state,
//               /ravenstate
//   subscribed: /arm1/servo_{jr,jp,jv,cr,cp,cv}, /grasp1/servo_{jr,jp,jv}, /arm1/state_command, /state_command
// Joint values are the real RAVEN's (insertion 0.40 m and tool roll 0 at HOME); CRTK poses are in the real CRTK
// base frame, raven_state poses in frame 0. Like the real RAVEN, the arm homes at start-up and then waits paused
// for a "resume" command; servo commands move it only while it is enabled. Known bugs of the real software are
// not reproduced.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <ambf_msgs/RigidBodyCmd.h>
#include <ambf_msgs/RigidBodyState.h>
#include <crtk_msgs/StringStamped.h>
#include <crtk_msgs/operating_state.h>
#include <geometry_msgs/TransformStamped.h>
#include <raven_2/raven_state.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "raven2_ambf_crtk_adapter/kinematics.h"

namespace raven2_ambf_crtk
{
using namespace raven_def;

const char *const FRAME0_ID = "frame0_L";      // raven_state poses: the real RAVEN's frame 0 of the left arm
const char *const CRTK_BASE_ID = "arm1_base";  // CRTK poses: the real RAVEN's CRTK base frame of arm1
const char *const JOINT_NAMES[7] = {"gold_shoulder", "gold_elbow", "gold_ins",  "gold_roll",
                                    "gold_wrist",    "gold_jaw1",  "gold_jaw2"};
const char *const GRASP_NAMES[3] = {"gold_grasp", "gold_jaw1", "gold_jaw2"};

Eigen::Matrix3d crtkBaseRotation()
{
  Eigen::Matrix3d R;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      R(r, c) = CRTK_BASE_ROTATION[r][c];
  return R;
}
const Eigen::Matrix3d R_CRTK = crtkBaseRotation();  // CRTK base <- frame 0

// frame 0 pose -> CRTK base pose
Eigen::Matrix4d toCrtk(const Eigen::Matrix4d &pose)
{
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = R_CRTK * pose.topLeftCorner<3, 3>();
  out.block<3, 1>(0, 3) = R_CRTK * pose.block<3, 1>(0, 3);
  return out;
}

// a rotation message, with an all-zero quaternion read as no rotation
Eigen::Matrix3d rotationOf(const geometry_msgs::Quaternion &q)
{
  const double v[4] = {q.x, q.y, q.z, q.w};
  return quaternionMatrix(v).topLeftCorner<3, 3>();
}

double angleBetween(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b)
{
  return std::acos(std::max(-1.0, std::min(1.0, ((a.transpose() * b).trace() - 1) / 2)));
}

Joints toReal(const Joints &joints)
{
  Joints out;
  for (int i = 0; i < 7; i++)
    out[i] = joints[i] + REAL_OFFSET[i];
  return out;
}

class Adapter
{
public:
  explicit Adapter(double rate) : rate_(rate), cycle_ms_(1000.0 / rate) {}

  bool connect();
  void run();

private:
  // Servo commands received since the last cycle, combined like the real RAVEN's network thread combines them:
  // relative commands add up, the others keep the latest one. A cycle applies only the first of jr, cr, jp, cp,
  // jv, cv (the real RAVEN's priority) and drops the rest; grasp commands apply when no joint command does.
  struct Pending
  {
    bool jr = false, jp = false, jv = false, cr = false, cp = false, cv = false;
    Joints jr_sum{}, jp_target{}, jv_velocity{};  // real RAVEN joint values for jp
    Eigen::Vector3d cr_translation = Eigen::Vector3d::Zero();
    Eigen::Matrix3d cr_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d cp_position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d cp_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d cv_linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d cv_angular = Eigen::Vector3d::Zero();
    bool grasp_jr = false, grasp_jp = false, grasp_jv = false;
    double grasp_jr_sum = 0, grasp_jp_target = 0, grasp_jv_velocity = 0;

    bool any() const { return jr || jp || jv || cr || cp || cv || grasp_jr || grasp_jp || grasp_jv; }
  };
  enum class Servo { NONE, JR, CR, JP, CP, JV, CV };

  // safety limit of the simulation: the setpoint sent to AMBF follows the goal at most this fast, so that an IK
  // jump does not become a step (joint servo commands are slower than this anyway)
  const double setpoint_rad_per_s_ = 2.5;
  const double setpoint_m_per_s_ = 0.5;

  void onState(const ambf_msgs::RigidBodyState::ConstPtr &msg);
  void onServoJr(const sensor_msgs::JointState::ConstPtr &msg);
  void onServoJp(const sensor_msgs::JointState::ConstPtr &msg);
  void onServoJv(const sensor_msgs::JointState::ConstPtr &msg);
  void onServoCr(const geometry_msgs::TransformStamped::ConstPtr &msg);
  void onServoCp(const geometry_msgs::TransformStamped::ConstPtr &msg);
  void onServoCv(const geometry_msgs::TransformStamped::ConstPtr &msg);
  void onGraspServoJr(const sensor_msgs::JointState::ConstPtr &msg);
  void onGraspServoJp(const sensor_msgs::JointState::ConstPtr &msg);
  void onGraspServoJv(const sensor_msgs::JointState::ConstPtr &msg);
  void onStateCommand(const crtk_msgs::StringStamped::ConstPtr &msg);

  std::string stateName() const;
  void startHoming();
  void enterRunlevel(int runlevel);
  void controlCycle();
  void stepHoming();
  bool applyServoCommands();
  void applyJointCommand(Servo type);
  void applyCartesianCommand(Servo type);
  void applyGraspCommand();
  void limitGoal();
  void setDesiredToMeasured();
  void setDesiredToGoal();
  void solveDesiredPose();
  void stepTowardGoal();
  void sendCommand();
  void publish();
  void publishRavenState(const ros::Time &stamp);

  double rate_, cycle_ms_;
  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;
  std::vector<ros::Subscriber> command_subs_;
  ros::Publisher command_pub_, measured_js_pub_, measured_cp_pub_, setpoint_js_pub_, setpoint_cp_pub_,
      grasp_measured_js_pub_, arm_state_pub_, state_pub_, raven_state_pub_;

  // measured state (AMBF joint values)
  bool have_state_ = false;
  Joints joint_positions_{}, joint_velocities_{};
  Eigen::Matrix4d measured_pose_ = Eigen::Matrix4d::Identity();  // frame 0

  // control state, as the real RAVEN keeps it
  int runlevel_ = RL_E_STOP;
  bool homed_ = false;
  bool joint_control_ = false;  // joint servo commands arrived this cycle
  int countdown_ = JOINT_CONTROL_COUNTDOWN;
  Eigen::Matrix4d desired_pose_ = Eigen::Matrix4d::Identity();  // target of Cartesian control, frame 0
  double desired_grasp_ = 0;                                     // its grasp angle, rad
  Eigen::Matrix4d reported_pose_ = Eigen::Matrix4d::Identity();  // raven_state pos_d / ori_d
  double reported_grasp_ = 0;                                    // raven_state grasp_d
  Joints goal_{};      // desired joints (jpos_d of the real RAVEN)
  Joints setpoint_{};  // sent to AMBF: the goal, rate limited
  Joints setpoint_velocity_{};
  Joints previous_setpoint_{};
  Pending pending_;
  long cycle_ = 0, last_command_cycle_ = -BUSY_CYCLES;
  int homing_cycle_ = 0;
  Joints homing_start_{};
  long clamp_count_ = 0;
  std::chrono::steady_clock::time_point last_step_;
  bool stepped_ = false;

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
  setpoint_ = goal_ = joint_positions_;
  measured_pose_ = forwardKinematics(joint_positions_);
  setDesiredToMeasured();

  measured_js_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm1/measured_js", 1, true);
  measured_cp_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/arm1/measured_cp", 1, true);
  setpoint_js_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm1/setpoint_js", 1, true);
  setpoint_cp_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/arm1/setpoint_cp", 1, true);
  grasp_measured_js_pub_ = nh_.advertise<sensor_msgs::JointState>("/grasp1/measured_js", 1, true);
  arm_state_pub_ = nh_.advertise<crtk_msgs::operating_state>("/arm1/operating_state", 1, true);
  state_pub_ = nh_.advertise<crtk_msgs::operating_state>("/operating_state", 1, true);
  raven_state_pub_ = nh_.advertise<raven_2::raven_state>("/ravenstate", 1, true);

  command_subs_ = {
      nh_.subscribe("/arm1/servo_jr", 100, &Adapter::onServoJr, this, nodelay),
      nh_.subscribe("/arm1/servo_jp", 100, &Adapter::onServoJp, this, nodelay),
      nh_.subscribe("/arm1/servo_jv", 100, &Adapter::onServoJv, this, nodelay),
      nh_.subscribe("/arm1/servo_cr", 100, &Adapter::onServoCr, this, nodelay),
      nh_.subscribe("/arm1/servo_cp", 100, &Adapter::onServoCp, this, nodelay),
      nh_.subscribe("/arm1/servo_cv", 100, &Adapter::onServoCv, this, nodelay),
      nh_.subscribe("/grasp1/servo_jr", 100, &Adapter::onGraspServoJr, this, nodelay),
      nh_.subscribe("/grasp1/servo_jp", 100, &Adapter::onGraspServoJp, this, nodelay),
      nh_.subscribe("/grasp1/servo_jv", 100, &Adapter::onGraspServoJv, this, nodelay),
      nh_.subscribe("/arm1/state_command", 10, &Adapter::onStateCommand, this, nodelay),
      nh_.subscribe("/state_command", 10, &Adapter::onStateCommand, this, nodelay)};

  ROS_INFO("CRTK to AMBF Adapter initialized and connected to AMBF (%s), running at %.0f Hz.", ns.c_str(), rate_);
  return true;
}

void Adapter::run()
{
  ros::Rate rate(rate_);
  startHoming();  // like pressing the real RAVEN's start button
  while (ros::ok())
  {
    ros::spinOnce();  // AMBF state, servo and state commands
    controlCycle();
    stepTowardGoal();
    sendCommand();
    publish();
    rate.sleep();
  }
}

// ---- incoming commands ------------------------------------------------------------------------------------------

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
    ROS_WARN_THROTTLE(1, "servo_jr needs 7 joint increments, got %zu. Command ignored.", msg->position.size());
    return;
  }
  for (int i = 0; i < 7; i++)
    pending_.jr_sum[i] += msg->position[i];
  pending_.jr = true;
}

void Adapter::onServoJp(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (msg->position.size() < 7)
  {
    ROS_WARN_THROTTLE(1, "servo_jp needs 7 joint positions, got %zu. Command ignored.", msg->position.size());
    return;
  }
  for (int i = 0; i < 7; i++)
    pending_.jp_target[i] = msg->position[i];
  pending_.jp = true;
}

void Adapter::onServoJv(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (msg->velocity.size() < 7)
  {
    ROS_WARN_THROTTLE(1, "servo_jv needs 7 joint velocities, got %zu. Command ignored.", msg->velocity.size());
    return;
  }
  for (int i = 0; i < 7; i++)
    pending_.jv_velocity[i] = msg->velocity[i];
  pending_.jv = true;
}

void Adapter::onServoCr(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  const auto &t = msg->transform.translation;
  pending_.cr_translation += Eigen::Vector3d(t.x, t.y, t.z);
  pending_.cr_rotation = pending_.cr_rotation * rotationOf(msg->transform.rotation);
  pending_.cr = true;
}

void Adapter::onServoCp(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  const auto &t = msg->transform.translation;
  Eigen::Vector3d target(t.x, t.y, t.z);
  for (int k = 0; k < 3; k++)
  {
    if (target[k] < WORKSPACE_LOW[k] || target[k] > WORKSPACE_HIGH[k])
    {
      ROS_ERROR_THROTTLE(1, "servo_cp target (%.3f, %.3f, %.3f) is outside the workspace. Command ignored.", target[0],
                         target[1], target[2]);
      return;
    }
  }
  // a target more than 15 mm from the tool is clipped to 15 mm, one more than 75 mm away is refused
  const Eigen::Vector3d tool = R_CRTK * measured_pose_.block<3, 1>(0, 3);
  const double step = (target - tool).norm();
  if (step > SERVO_CP_MAX_STEP)
  {
    ROS_ERROR_THROTTLE(1, "servo_cp target is %.3f m from the tool (more than %.3f m). Command ignored.", step,
                       SERVO_CP_MAX_STEP);
    return;
  }
  if (step >= SERVO_CP_STEP)
  {
    target = tool + SERVO_CP_STEP / step * (target - tool);
    ROS_INFO_THROTTLE(5, "servo_cp target clipped to %.0f mm from the tool.", SERVO_CP_STEP * 1000);
  }
  pending_.cp_position = target;
  pending_.cp_rotation = rotationOf(msg->transform.rotation);
  pending_.cp = true;
}

void Adapter::onServoCv(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  // like the real RAVEN: a TransformStamped whose translation is the linear velocity (m/s) and whose rotation, as
  // an axis and an angle, is the angular velocity (rad/s)
  const auto &t = msg->transform.translation;
  const auto &q = msg->transform.rotation;
  pending_.cv_linear = Eigen::Vector3d(t.x, t.y, t.z);
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  pending_.cv_angular.setZero();
  if (n > 1e-9)
  {
    const double w = std::max(-1.0, std::min(1.0, q.w / n));
    const double s = std::sqrt(1 - w * w);
    if (s > 1e-9)
      pending_.cv_angular = Eigen::Vector3d(q.x, q.y, q.z) / n / s * (2 * std::acos(w));
  }
  pending_.cv = true;
}

void Adapter::onGraspServoJr(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (msg->position.empty())
    return;
  pending_.grasp_jr_sum += msg->position[0];
  pending_.grasp_jr = true;
}

void Adapter::onGraspServoJp(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (msg->position.empty())
    return;
  pending_.grasp_jp_target = msg->position[0];
  pending_.grasp_jp = true;
}

void Adapter::onGraspServoJv(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (msg->velocity.empty())
    return;
  pending_.grasp_jv_velocity = msg->velocity[0];
  pending_.grasp_jv = true;
}

// ---- state machine ----------------------------------------------------------------------------------------------

// The real RAVEN's runlevels and CRTK states: E-stop = DISABLED, init (homing) = ENABLED and busy,
// pedal up = PAUSED, pedal down = ENABLED. The simulation has no E-stop, start button or pedal; the state commands
// stand in for them.
std::string Adapter::stateName() const
{
  if (runlevel_ == RL_E_STOP)
    return "DISABLED";
  return runlevel_ == RL_PEDAL_UP ? "PAUSED" : "ENABLED";
}

void Adapter::onStateCommand(const crtk_msgs::StringStamped::ConstPtr &msg)
{
  const std::string &command = msg->string;
  ROS_INFO("Received %s", command.c_str());
  if (command == "enable")
  {
    // from DISABLED, the start button: home if needed (then PAUSED, like the real RAVEN), else go ENABLED
    if (runlevel_ != RL_E_STOP)
      ROS_WARN("\"enable\" only works when the arm is DISABLED (now %s).", stateName().c_str());
    else if (homed_)
      enterRunlevel(RL_PEDAL_DN);
    else
      startHoming();
  }
  else if (command == "disable")
  {
    if (runlevel_ != RL_E_STOP)
      enterRunlevel(RL_E_STOP);
  }
  else if (command == "pause")
  {
    if (runlevel_ == RL_INIT)
      enterRunlevel(RL_E_STOP);  // stops homing
    else if (runlevel_ == RL_PEDAL_DN)
      enterRunlevel(RL_PEDAL_UP);
    else
      ROS_WARN("\"pause\" only works when the arm is ENABLED (now %s).", stateName().c_str());
  }
  else if (command == "resume")
  {
    if (runlevel_ == RL_PEDAL_UP)
      enterRunlevel(RL_PEDAL_DN);
    else if (runlevel_ == RL_E_STOP)
      ROS_WARN("The arm is DISABLED: send \"enable\" (on the real RAVEN, press the start button).");
    else
      ROS_WARN("\"resume\" only works when the arm is PAUSED (now %s).", stateName().c_str());
  }
  else if (command == "unhome")
  {
    homed_ = false;
    enterRunlevel(RL_E_STOP);
  }
  else if (command == "home")
  {
    startHoming();
  }
  else
  {
    ROS_WARN("Unknown state command \"%s\".", command.c_str());
  }
}

void Adapter::startHoming()
{
  homed_ = false;
  homing_start_ = joint_positions_;
  homing_cycle_ = 0;
  enterRunlevel(RL_INIT);
}

void Adapter::enterRunlevel(int runlevel)
{
  if (runlevel_ == RL_INIT && runlevel != RL_INIT && !homed_)
    ROS_WARN("Homing stopped before it finished: the arm is not homed.");
  runlevel_ = runlevel;
  joint_control_ = false;
  // the real RAVEN turns the motors off below pedal down; the simulated arm holds where it is
  if (runlevel == RL_E_STOP || runlevel == RL_PEDAL_UP)
    goal_ = setpoint_;
  ROS_INFO("Entered runlevel %d: %s%s.", runlevel, stateName().c_str(), runlevel == RL_INIT ? " (homing)" : "");
}

// ---- control ----------------------------------------------------------------------------------------------------

void Adapter::controlCycle()
{
  cycle_++;
  previous_setpoint_ = setpoint_;
  measured_pose_ = forwardKinematics(joint_positions_);

  if (runlevel_ == RL_INIT)
  {
    if (pending_.any())
      ROS_WARN_THROTTLE(1, "Homing: servo commands are ignored.");
    stepHoming();
  }
  else if (runlevel_ == RL_PEDAL_DN)
  {
    if (applyServoCommands())
      last_command_cycle_ = cycle_;
  }
  else if (pending_.any())
  {
    ROS_WARN_THROTTLE(1, "The arm is %s: servo commands are ignored (send \"%s\").", stateName().c_str(),
                      runlevel_ == RL_PEDAL_UP ? "resume" : "enable");
  }
  pending_ = Pending();

  // Cartesian control (inverse kinematics every cycle) takes over 10 cycles after the last joint command. Like the
  // real RAVEN, raven_state reports the measured pose as the desired one under joint control and keeps it during
  // those 10 cycles; but Cartesian control starts from the pose of the joint goal: starting from the measured
  // pose, as the real RAVEN does, would stop the arm short of where the joint commands sent it.
  if (runlevel_ == RL_PEDAL_DN)
  {
    if (joint_control_)
    {
      countdown_ = JOINT_CONTROL_COUNTDOWN;
      setDesiredToGoal();
      reported_pose_ = measured_pose_;
      reported_grasp_ = joint_positions_[5] + joint_positions_[6];
    }
    else if (countdown_ > 0)
    {
      countdown_--;
    }
    else
    {
      solveDesiredPose();
      reported_pose_ = desired_pose_;
      reported_grasp_ = desired_grasp_;
    }
  }
  else
  {
    // homing, E-stop and pedal up: the desired pose is the measured one, so that the arm continues from where
    // it is when it gets enabled
    setDesiredToMeasured();
    countdown_ = 0;
  }
}

void Adapter::stepHoming()
{
  // A 2 s minimum-jerk path to HOME (the real RAVEN finds its joint limits first; the simulation knows its joints).
  const int n = std::max(1, static_cast<int>(2.0 * rate_));
  homing_cycle_++;
  double s = std::min(1.0, static_cast<double>(homing_cycle_) / n);
  s = 10 * std::pow(s, 3) - 15 * std::pow(s, 4) + 6 * std::pow(s, 5);
  for (int i = 0; i < 7; i++)
    setpoint_[i] = goal_[i] = homing_start_[i] + s * (HOME[i] - homing_start_[i]);
  if (homing_cycle_ >= n)
  {
    homed_ = true;
    std::cout << "Set Home" << std::endl;
    enterRunlevel(RL_PEDAL_UP);  // like the real RAVEN: homed, waiting for "resume"
  }
}

bool Adapter::applyServoCommands()
{
  const Pending &p = pending_;
  const Servo type = p.jr ? Servo::JR : p.cr ? Servo::CR : p.jp ? Servo::JP : p.cp ? Servo::CP
                   : p.jv ? Servo::JV : p.cv ? Servo::CV : Servo::NONE;
  const bool joint = type == Servo::JR || type == Servo::JP || type == Servo::JV;
  if (joint)
    applyJointCommand(type);
  else if (type != Servo::NONE)
    applyCartesianCommand(type);
  if (!joint)
    applyGraspCommand();
  joint_control_ = joint;
  return type != Servo::NONE;
}

void Adapter::applyJointCommand(Servo type)
{
  const Pending &p = pending_;
  for (int i = 0; i < 7; i++)
  {
    const double limit = (i == 2 ? SERVO_M_PER_MS : SERVO_RAD_PER_MS) * cycle_ms_;  // per cycle
    double step;
    if (type == Servo::JR)
      step = p.jr_sum[i];
    else if (type == Servo::JP)
      step = p.jp_target[i] - REAL_OFFSET[i] - goal_[i];
    else
      step = p.jv_velocity[i] * cycle_ms_ / 1000;
    if (std::abs(step) > limit)
    {
      // servo_jp always moves at most at the limit; for jr / jv a step beyond it is a client sending too much
      step = step > 0 ? limit : -limit;
      if (type != Servo::JP && ++clamp_count_ % 500 == 0)
        ROS_WARN("servo_j%s: %ld joint steps limited to the joint speed limit (%.4f rad or %.5f m per ms).",
                 type == Servo::JR ? "r" : "v", clamp_count_, SERVO_RAD_PER_MS, SERVO_M_PER_MS);
    }
    goal_[i] += step;
  }
  limitGoal();
}

void Adapter::limitGoal()
{
  // the joint limits act like the hard stops: a joint servoed past one stays there
  for (int i = 0; i < 7; i++)
  {
    const double clamped = std::max(JOINT_LIMIT_LOW[i], std::min(JOINT_LIMIT_HIGH[i], goal_[i]));
    if (clamped != goal_[i])
      ROS_WARN_THROTTLE(1, "Joint %d (%s) is at its limit (%.3f).", i + 1, JOINT_NAMES[i], clamped + REAL_OFFSET[i]);
    goal_[i] = clamped;
  }
}

void Adapter::applyCartesianCommand(Servo type)
{
  const Pending &p = pending_;
  const double dt = cycle_ms_ / 1000;
  const double max_translation = SERVO_CR_M_PER_MS * cycle_ms_;
  if (type == Servo::CR || type == Servo::CV)
  {
    // increments in the CRTK base frame: the translation is added to the desired position, the rotation turns
    // the desired orientation about the base axes (pre-multiplied), as on the real RAVEN
    Eigen::Vector3d translation =
        R_CRTK.transpose() * (type == Servo::CR ? p.cr_translation : Eigen::Vector3d(p.cv_linear * dt));
    if (translation.norm() > max_translation)
    {
      ROS_INFO_THROTTLE(1, "Relative Cartesian translation scaled to the maximum speed (%.3f m per ms).",
                        SERVO_CR_M_PER_MS);
      translation *= max_translation / translation.norm();
    }
    Eigen::Matrix3d rotation = p.cr_rotation;
    if (type == Servo::CV)
    {
      const double angle = p.cv_angular.norm() * dt;
      rotation = angle > 0 ? Eigen::AngleAxisd(angle, p.cv_angular.normalized()).toRotationMatrix()
                           : Eigen::Matrix3d::Identity();
    }
    desired_pose_.block<3, 1>(0, 3) += translation;
    desired_pose_.topLeftCorner<3, 3>() =
        R_CRTK.transpose() * rotation * R_CRTK * desired_pose_.topLeftCorner<3, 3>();
  }
  else
  {
    // absolute pose in the CRTK base frame
    const Eigen::Vector3d position = R_CRTK.transpose() * p.cp_position;
    if ((position - measured_pose_.block<3, 1>(0, 3)).norm() <= SERVO_CP_MAX_JUMP)
      desired_pose_.block<3, 1>(0, 3) = position;
    else
      ROS_ERROR_THROTTLE(1, "servo_cp: absolute Cartesian step too large. Position ignored.");
    const Eigen::Matrix3d rotation = R_CRTK.transpose() * p.cp_rotation;
    if (angleBetween(rotation, measured_pose_.topLeftCorner<3, 3>()) <= SERVO_CP_MAX_ROTATION)
      desired_pose_.topLeftCorner<3, 3>() = rotation;
    else
      ROS_ERROR_THROTTLE(1, "servo_cp: absolute Cartesian rotation too large. Rotation ignored.");
  }
}

void Adapter::applyGraspCommand()
{
  const Pending &p = pending_;
  if (p.grasp_jr)
    desired_grasp_ += p.grasp_jr_sum;
  else if (p.grasp_jp)
    desired_grasp_ = p.grasp_jp_target;
  else if (p.grasp_jv)
    desired_grasp_ += p.grasp_jv_velocity * cycle_ms_ / 1000;
}

void Adapter::setDesiredToMeasured()
{
  reported_pose_ = desired_pose_ = measured_pose_;
  reported_grasp_ = desired_grasp_ = joint_positions_[5] + joint_positions_[6];
}

void Adapter::setDesiredToGoal()
{
  desired_pose_ = forwardKinematics(goal_);
  desired_grasp_ = goal_[5] + goal_[6];
}

void Adapter::solveDesiredPose()
{
  // the real RAVEN's Cartesian control: inverse kinematics of the desired pose every cycle, taking the solution
  // closest to the measured joints, keeping the joints if there is none and saturating at the joint limits
  Joints joints;
  bool limited = false;
  if (inverseKinematics(desired_pose_, desired_grasp_, joint_positions_, joints, limited) != IkResult::OK)
  {
    ROS_ERROR_THROTTLE(1, "No IK solution for the desired pose (unreachable or at the RCM). Joints kept.");
    return;
  }
  // the tool roll is continuous: the turn closest to the measured roll
  joints[3] += 2 * PI * std::nearbyint((joint_positions_[3] - joints[3]) / (2 * PI));
  limited = applyJointLimits(joints) || limited;
  if (limited)
  {
    ROS_WARN_THROTTLE(1, "The desired pose is outside the joint limits: saturated at the limits.");
    desired_pose_ = forwardKinematics(joints);
    desired_grasp_ = joints[5] + joints[6];
  }
  goal_ = joints;
}

void Adapter::stepTowardGoal()
{
  const auto now = std::chrono::steady_clock::now();
  // a late cycle may step further, but never more than 20 ms worth
  const double dt_ms =
      stepped_ ? std::min(std::max(std::chrono::duration<double, std::milli>(now - last_step_).count(), cycle_ms_), 20.0)
               : cycle_ms_;
  last_step_ = now;
  stepped_ = true;

  for (int i = 0; i < 7; i++)
  {
    const double limit = (i == 2 ? setpoint_m_per_s_ : setpoint_rad_per_s_) * dt_ms / 1000;
    const double diff = goal_[i] - setpoint_[i];
    setpoint_[i] = std::abs(diff) <= limit ? goal_[i] : setpoint_[i] + (diff > 0 ? limit : -limit);
    setpoint_velocity_[i] = (setpoint_[i] - previous_setpoint_[i]) / (dt_ms / 1000);
  }
}

void Adapter::sendCommand()
{
  for (int i = 0; i < 7; i++)
    command_.joint_cmds[i] = setpoint_[i];
  command_.header.stamp = ros::Time::now();
  command_pub_.publish(command_);
}

// ---- outgoing state ---------------------------------------------------------------------------------------------

void fillTransform(const Eigen::Matrix4d &pose, geometry_msgs::Transform &out)
{
  out.translation.x = pose(0, 3);
  out.translation.y = pose(1, 3);
  out.translation.z = pose(2, 3);
  double q[4];
  quaternionFromMatrix(pose, q);
  out.rotation.x = q[0];
  out.rotation.y = q[1];
  out.rotation.z = q[2];
  out.rotation.w = q[3];
}

void Adapter::publish()
{
  const ros::Time now = ros::Time::now();
  const Joints measured = toReal(joint_positions_), setpoint = toReal(setpoint_);

  sensor_msgs::JointState js;
  js.header.stamp = now;
  js.name.assign(JOINT_NAMES, JOINT_NAMES + 7);
  js.position.assign(measured.begin(), measured.end());
  js.velocity.assign(joint_velocities_.begin(), joint_velocities_.end());
  js.effort.assign(7, 0.0);
  measured_js_pub_.publish(js);
  js.position.assign(setpoint.begin(), setpoint.end());
  js.velocity.assign(setpoint_velocity_.begin(), setpoint_velocity_.end());
  js.effort.clear();
  setpoint_js_pub_.publish(js);

  geometry_msgs::TransformStamped cp;
  cp.header.stamp = now;
  cp.header.frame_id = CRTK_BASE_ID;
  cp.child_frame_id = "end_effector";
  fillTransform(toCrtk(measured_pose_), cp.transform);
  measured_cp_pub_.publish(cp);
  fillTransform(toCrtk(forwardKinematics(setpoint_)), cp.transform);
  setpoint_cp_pub_.publish(cp);

  sensor_msgs::JointState grasp;
  grasp.header.stamp = now;
  grasp.name.assign(GRASP_NAMES, GRASP_NAMES + 3);
  grasp.position = {joint_positions_[5] + joint_positions_[6], joint_positions_[5], joint_positions_[6]};
  grasp.velocity = {joint_velocities_[5] + joint_velocities_[6], joint_velocities_[5], joint_velocities_[6]};
  grasp.effort.assign(3, 0.0);
  grasp_measured_js_pub_.publish(grasp);

  crtk_msgs::operating_state state;
  state.header.stamp = now;
  state.state = stateName();
  state.is_homed = homed_;
  state.is_busy = runlevel_ == RL_INIT || (runlevel_ == RL_PEDAL_DN && cycle_ - last_command_cycle_ < BUSY_CYCLES);
  arm_state_pub_.publish(state);
  state_pub_.publish(state);

  publishRavenState(now);
}

void Adapter::publishRavenState(const ros::Time &stamp)
{
  // raven_state layout: 0-2 shoulder/elbow/insertion, 3 unused, 4-7 tool roll/wrist/graspers; the real RAVEN's
  // joint values, angles in degrees and velocities in degrees/s (the insertion too, as the real RAVEN does)
  const Joints measured = toReal(joint_positions_), setpoint = toReal(setpoint_);
  double jpos[8] = {0}, jpos_d[8] = {0}, jvel[8] = {0}, mpos[8] = {0}, mpos_d[8] = {0};
  for (int i = 0; i < 7; i++)
  {
    const int k = i < 3 ? i : i + 1;
    jpos[k] = measured[i] * 180.0 / PI;
    jpos_d[k] = setpoint[i] * 180.0 / PI;
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
  msg.hdr.stamp = stamp;
  msg.hdr.frame_id = FRAME0_ID;
  msg.runlevel = runlevel_;
  msg.sublevel = homed_ ? 1 : 0;  // the crtk branch reports whether the arm is homed here
  msg.last_seq = 0;               // sequence of teleoperation console packets: there is no console
  msg.type[0] = GOLD_ARM;
  msg.dt = ros::Duration(dt);

  // pos / ori: the measured tool pose; pos_d / ori_d / grasp_d: the desired one, which is the measured one
  // except under Cartesian control
  for (int i = 0; i < 3; i++)
  {
    msg.pos[i] = static_cast<int>(measured_pose_(i, 3) * 1e6);
    msg.pos_d[i] = static_cast<int>(reported_pose_(i, 3) * 1e6);
    for (int j = 0; j < 3; j++)
    {
      msg.ori[3 * i + j] = measured_pose_(i, j);
      msg.ori_d[3 * i + j] = reported_pose_(i, j);
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
  msg.grasp_d[0] = reported_grasp_;
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
  adapter.run();
  return 0;
}
