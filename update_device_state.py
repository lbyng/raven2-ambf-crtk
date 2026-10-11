import rospy
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
        self.MAX_DIST_PER_MS = 0.1
        self.MAX_RADIAN_PER_MS = 0.5
        
        self.robot_state = robot_state
        self.arm = arm
        
        self.js_limit_count = 0
        return None
    
    
    def update_crtk_motion(self):
        out = 0
        busy_cycle = 0
        busy_flag = 0
        
        type = self.robot_state.control_mode
        
        if type == "JOINT":
            out = self.update_crtk_motion_js()
        else:
            out = self.update_crtk_motion_tf()

        if out or busy_flag:
            self.robot_state.is_busy = True
            busy_flag = 1
            busy_cycle += 1
            if (not out) and (busy_cycle > 1000):
                busy_flag = 0
        else:
            self.robot_state.is_busy = False
            busy_cycle = 0
        
        return None
    

    def update_crtk_motion_js(self):
        out = 0
        if self.robot_state.motion_state == "MOVING":
            self.robot_state.motion_state = "IDLE"
            
            if(self.check_joint_limits()):

                # Deltas accumulate on the setpoint, not the measured position, so a joint that gets
                # knocked off by the motion of another one is driven back instead of staying off.
                for i in range(7):
                    limit = self.max_meters_per_ms if i == 2 else self.max_radian_per_ms

                    if abs(self.robot_state.joint_delta[i]) <= limit:
                        target = self.robot_state.joint_targets[i] + self.robot_state.joint_delta[i]
                        self.robot_state.joint_targets[i] = target
                        self.arm.set_joint_pos(i, target)
                        out = 1
                        
                    else:
                        sign = 1 if self.robot_state.joint_delta[i] > 0 else -1
                        target = self.robot_state.joint_targets[i] + sign * limit
                        self.robot_state.joint_targets[i] = target
                        self.arm.set_joint_pos(i, target)

                        self.js_limit_count += 1
                        out = 1

                        if self.js_limit_count % 500 == 0:
                            rospy.logerr("THAT WAS TOO FAR! joint {} jpos_d - {}, in {} ".format(i, self.robot_state.joint_delta[i] + self.robot_state.joint_targets[i], self.robot_state.joint_delta[i]))
                            out = -1
        return out
    
    
    def update_crtk_motion_tf(self):
        out = 0
        if self.robot_state.motion_state == "MOVING":
            self.robot_state.motion_state = "IDLE"
            
            curr_tm = fk.fwd_kinematics(0, self.robot_state.joint_targets, ard)
            delta_tm = tft.quaternion_matrix(self.robot_state.delta_ori)
            delta_tm[0:3, 3] = self.robot_state.delta_pos
            
            target_tm = np.matmul(curr_tm, delta_tm)
            
            jpl = ik.inv_kinematics(0, target_tm, 0, ard)
            limited = jpl[1]
            if limited:
                rospy.logerr("Desired cartesian position is out of bounds for Raven2. Will move to max pos.")
            
            for i in range(7):
                self.robot_state.joint_targets[i] = jpl[0][i]
                self.arm.set_joint_pos(i, jpl[0][i])
                
            out = 1

        return out
    
    def check_joint_limits(self):
        joint_targets = np.array(self.robot_state.joint_targets, dtype='float')
        joint_deltas = np.array(self.robot_state.joint_delta, dtype='float')
        min_limits = ard.RAVEN_JOINT_LIMITS[0]
        max_limits = ard.RAVEN_JOINT_LIMITS[1]

        future_positions = joint_targets + joint_deltas
        within_limits = True

        for i, (future_pos, min_val, max_val) in enumerate(zip(future_positions, min_limits, max_limits)):
            if future_pos < min_val or future_pos > max_val:
                rospy.logerr(f"Joint {i} will exceed limits: {future_pos:.3f} (min: {min_val:.3f}, max: {max_val:.3f})")
                within_limits = False

        return within_limits
        
    