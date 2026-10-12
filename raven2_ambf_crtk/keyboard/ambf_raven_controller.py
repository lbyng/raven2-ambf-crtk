import math
import time
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import crtk_msgs.msg

# The real RAVEN moves a joint at most this much per 1 ms cycle on servo_jr (rad, and m for the insertion); larger
# increments are cut to it, so a move is sent as small steps, half this size so that two landing in one cycle are
# still applied in full.
SERVO_STEP_RAD = 0.0025 / 2
SERVO_STEP_M = 0.00005 / 2

class ambf_raven_controller:
    def __init__(self):
        self.__init_pub_sub()

        self.max_rate_move = 20
        self.min_interval_move = 1.0/self.max_rate_move
        self.time_last_pub_move = -1.0
        return None

    def __init_pub_sub(self):
        self.__publisher_servo_jr = rospy.Publisher('/arm1/servo_jr', sensor_msgs.msg.JointState, queue_size = 500)
        self.__publisher_servo_cr = rospy.Publisher('/arm1/servo_cr', geometry_msgs.msg.TransformStamped, queue_size = 1)
        self.__publisher_state_command = rospy.Publisher('/arm1/state_command', crtk_msgs.msg.StringStamped, queue_size = 1)
        self.state = None
        rospy.Subscriber('/arm1/operating_state', crtk_msgs.msg.operating_state, self.__state_callback)
        return None

    def __state_callback(self, msg):
        self.state = msg

    def enable(self, timeout = 20.0):
        """Bring the arm to ENABLED, as a client of the real RAVEN does: "enable" when it is DISABLED (it homes if
        needed), "resume" when it is PAUSED."""
        end = time.time() + timeout
        last_sent = 0.0
        while time.time() < end and not rospy.is_shutdown():
            state = self.state
            if state is not None and state.state == 'ENABLED' and state.is_homed:
                return True
            command = {'PAUSED': 'resume', 'DISABLED': 'enable'}.get(state.state if state else None)
            if (command and time.time() - last_sent > 1.0
                    and self.__publisher_state_command.get_num_connections() > 0):
                self.__publisher_state_command.publish(crtk_msgs.msg.StringStamped(string = command))
                last_sent = time.time()
            time.sleep(0.05)
        return False

    def pub_servo_jr_command(self, joint_command):

        joint_command = joint_command[1:8]

        interval_pub = time.time() - self.time_last_pub_move

        if self.time_last_pub_move != -1.0:
            interval_pub = time.time() - self.time_last_pub_move
            if interval_pub < self.min_interval_move:
                time.sleep(self.min_interval_move - interval_pub)

        # one step per millisecond, so that the whole increment is applied
        steps = max(1, int(math.ceil(max(abs(x) / (SERVO_STEP_M if i == 2 else SERVO_STEP_RAD)
                                         for i, x in enumerate(joint_command)))))
        msg = sensor_msgs.msg.JointState()
        msg.position = [float(x) / steps for x in joint_command]
        next_time = time.time()
        for _ in range(steps):
            self.__publisher_servo_jr.publish(msg)
            next_time += 0.0011
            time.sleep(max(0.0, next_time - time.time()))
        self.time_last_pub_move = time.time()
        return 1

    def pub_servo_cr_command(self, position, rotation):
        msg = geometry_msgs.msg.TransformStamped()

        msg.transform.translation.x = position[0]
        msg.transform.translation.y = position[1]
        msg.transform.translation.z = position[2]

        if not any(rotation):
            rotation = [0, 0, 0, 1]  # no rotation
        msg.transform.rotation.x = rotation[0]
        msg.transform.rotation.y = rotation[1]
        msg.transform.rotation.z = rotation[2]
        msg.transform.rotation.w = rotation[3]

        self.__publisher_servo_cr.publish(msg)
        return 1
