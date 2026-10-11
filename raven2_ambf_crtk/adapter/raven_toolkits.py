import rospy
import numpy as np

from ..kinematics import ambf_raven_def as ard

class raven_toolkits:
    def __init__(self, arm):
        self.arm = arm
        return None

    def set_home(self, duration=2.0, rate=100):
        # Ease into HOME along a minimum-jerk path instead of jumping there: a sudden
        # shoulder/elbow step swings the lightly damped wrist and graspers for seconds.
        start = [self.arm.get_joint_pos(i) for i in range(7)]
        if None in start:   # no joint state from AMBF yet, nothing to interpolate from
            start = ard.HOME_JOINTS
        start = np.array(start, dtype='float')

        r = rospy.Rate(rate)
        n = max(1, int(duration * rate))
        for k in range(1, n + 1):
            s = k / n
            s = 10 * s**3 - 15 * s**4 + 6 * s**5
            q = start + s * (ard.HOME_JOINTS - start)
            for i in range(7):
                self.arm.set_joint_pos(i, q[i])
            r.sleep()
        return None

