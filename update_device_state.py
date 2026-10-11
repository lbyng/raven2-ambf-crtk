import rospy
import time
import numpy as np
import tf.transformations as tft
from utils import raven_fk as fk
from utils import raven_ik as ik
from utils import ambf_raven_def as ard

class update_device_state:
    def __init__(self, robot_state, arm):
        self.max_radian_per_ms = 0.0025   # rad/ms
        # self.max_meters_per_ms = 0.00005  # meters/ms
        self.max_meters_per_ms = 0.0005  # meters/ms

        self.MICRON_PER_M = 1e6
        # largest joint change accepted for a single servo_cr command; anything bigger is not a small move
        self.max_ik_step_m = 0.1
        self.max_ik_step_rad = np.pi

        self.robot_state = robot_state
        self.arm = arm

        # Commands move the joint goal. Every cycle the setpoint sent to AMBF (robot_state.joint_targets)
        # steps toward it at the speed limits above over the real elapsed time, so the motion does not
        # depend on how fast this loop actually runs.
        self.joint_goal = None
        self.last_update = None

        self.js_limit_count = 0
        return None


    def update_crtk_motion(self):
        if self.joint_goal is None:
            self.joint_goal = self.robot_state.joint_targets.copy()

        # apply every command queued since the last cycle, so none is dropped or left waiting
        while self.robot_state.commands:
            command = self.robot_state.commands.popleft()
            self.robot_state.control_mode = command[0]

            if command[0] == "JOINT":
                self.robot_state.joint_delta[:] = command[1]
                self.update_crtk_motion_js()
            else:
                self.robot_state.delta_pos[:] = command[1]
                self.robot_state.delta_ori[:] = command[2]
                self.update_crtk_motion_tf()

        self.step_toward_goal()
        return None


    def step_toward_goal(self):
        now = time.monotonic()
        # cap the time step so that a stalled cycle does not turn into a jump
        dt_ms = 1.0 if self.last_update is None else min((now - self.last_update) * 1000.0, 20.0)
        self.last_update = now

        targets = self.robot_state.joint_targets
        if np.array_equal(targets, self.joint_goal):
            return None

        limit = np.full(7, self.max_radian_per_ms * dt_ms)
        limit[2] = self.max_meters_per_ms * dt_ms
        diff = self.joint_goal - targets
        joints = np.where(np.abs(diff) <= limit, self.joint_goal, targets + np.sign(diff) * limit)
        for i in range(7):
            targets[i] = joints[i]
            self.arm.set_joint_pos(i, joints[i])
        return None


    def update_crtk_motion_js(self):
        out = 0
        if(self.check_joint_limits()):

            # Deltas accumulate on the goal, not the measured position, so a joint that gets
            # knocked off by the motion of another one is driven back instead of staying off.
            for i in range(7):
                limit = self.max_meters_per_ms if i == 2 else self.max_radian_per_ms

                if abs(self.robot_state.joint_delta[i]) <= limit:
                    self.joint_goal[i] += self.robot_state.joint_delta[i]
                    out = 1

                else:
                    sign = 1 if self.robot_state.joint_delta[i] > 0 else -1
                    self.joint_goal[i] += sign * limit

                    self.js_limit_count += 1
                    out = 1

                    if self.js_limit_count % 500 == 0:
                        rospy.logerr("THAT WAS TOO FAR! joint {} jpos_d - {}, in {} ".format(i, self.robot_state.joint_delta[i] + self.joint_goal[i], self.robot_state.joint_delta[i]))
                        out = -1
        return out


    def update_crtk_motion_tf(self):
        goal = self.joint_goal

        curr_tm = fk.fwd_kinematics(0, goal, ard)
        delta_tm = tft.quaternion_matrix(self.robot_state.delta_ori)
        delta_tm[0:3, 3] = self.robot_state.delta_pos

        target_tm = np.matmul(curr_tm, delta_tm)

        # Keep the current grasp angle and take the IK branch closest to the current goal.
        jpl = ik.inv_kinematics(0, target_tm, goal[5] + goal[6], ard, goal)
        if jpl is None:
            rospy.logerr("No IK solution for the desired cartesian pose (unreachable or at the RCM). Command ignored.")
            return 0
        joints, limited = jpl

        # The tool roll is continuous: use the turn closest to the current goal.
        joints[3] += 2 * np.pi * np.round((goal[3] - joints[3]) / (2 * np.pi))
        joints, limited_roll = ik.apply_joint_limits(joints, ard)

        step = np.abs(joints - goal)
        if step[2] > self.max_ik_step_m or np.delete(step, 2).max() > self.max_ik_step_rad:
            rospy.logerr("Desired cartesian pose needs a joint jump of {:.3f}. Command ignored.".format(step.max()))
            return 0

        # Saturating the joints instead would put the tool somewhere other than where it was sent.
        if limited or limited_roll:
            rospy.logerr("Desired cartesian pose is outside the Raven2 joint limits. Command ignored.")
            return 0

        # The setpoint follows at the joint speed limits (step_toward_goal): near the wrist
        # singularity (HOME has wrist = 0) a 1 cm move can need ~0.7 rad of tool roll.
        goal[:] = joints
        return 1

    def check_joint_limits(self):
        joint_goal = np.array(self.joint_goal, dtype='float')
        joint_deltas = np.array(self.robot_state.joint_delta, dtype='float')
        min_limits = ard.RAVEN_JOINT_LIMITS[0]
        max_limits = ard.RAVEN_JOINT_LIMITS[1]

        future_positions = joint_goal + joint_deltas
        within_limits = True

        for i, (future_pos, min_val, max_val) in enumerate(zip(future_positions, min_limits, max_limits)):
            if future_pos < min_val or future_pos > max_val:
                rospy.logerr(f"Joint {i} will exceed limits: {future_pos:.3f} (min: {min_val:.3f}, max: {max_val:.3f})")
                within_limits = False

        return within_limits


