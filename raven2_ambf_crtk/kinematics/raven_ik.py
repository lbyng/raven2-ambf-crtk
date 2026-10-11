from .raven_fk import fwd_kinematics, fwd_trans, joint_to_dhvalue
import math as m
import numpy as np
from . import utilities as u


# alpha, theta, a, d --> 3 out of 4 are constant, for R --> theta, for P --> ?
# input_cp --> T_C_6, where the tip of the robot is with respect to the global frame, C
# T_CB --> here the B frame is with respect to C, in sim, universal frame called C in between two arms, the B frames
# T_B0 --> where the base of the robot is with respect to the B frame (middle of the robot arms)

# this method returns (joints, limited) for the valid solution closest to curr_jp (HOME_JOINTS if not given),
# or None when there is no valid solution (RCM singularity or unreachable pose)

def inv_kinematics(arm, input_cp, input_gangle, raven_def, curr_jp=None):

    # contextualizing the location of the tip of raven with respect to the outer plane of existence

    # xf = T_0_6, one per arm

    # T_0_6 = np.matmul(np.linalg.inv(np.matmul(raven_def.RAVEN_T_CB, raven_def.RAVEN_T_B0[arm])), input_cp)
    
    #########
    # T_0_6 = np.matmul(np.linalg.inv(np.matmul(raven_def.X_ROT, raven_def.RAVEN_T_B0[arm])), input_cp)
    # T_0_6 = input_cp
    
    T_0_6 = np.matmul(np.linalg.inv(raven_def.RAVEN_T_B0[arm]), input_cp)
    #########
    
    iksol = np.zeros((raven_def.RAVEN_IKSOLS, raven_def.RAVEN_JOINTS))
    ikcheck = np.zeros(raven_def.RAVEN_IKSOLS)

    dh_alpha = np.zeros(6) # known
    dh_theta = np.zeros(6) # trying to solve, except for joint 3 (index 2)
    dh_d = np.zeros(6) # known
    dh_a = np.zeros(6) # known, except for joint 3 (index 2)

    for i in range(raven_def.RAVEN_JOINTS - 1):
        # in ambf_def "V" is the placeholder
        dh_alpha[i] = raven_def.RAVEN_DH_ALPHA[arm][i]
        dh_theta[i] = raven_def.RAVEN_DH_THETA[arm][i]
        dh_d[i]     = raven_def.RAVEN_DH_D[arm][i]
        dh_a[i]     = raven_def.RAVEN_DH_A[arm][i]

    for i in range(raven_def.RAVEN_IKSOLS):
        iksol[i]   = np.zeros(raven_def.RAVEN_JOINTS) # 6 length cannot accomodate length 7 vector??? if problem, create zero_joints length 6
        ikcheck[i] = True # flag whether particular set of joints are legal or checking eventually the closest one

    # STEP 1: Comput P5
    T_6_0 = np.linalg.inv(T_0_6) # T60 --> the 0th frame in terms of the 6th frame

    p6rcm = np.zeros((4,1), dtype = 'float')
    p6rcm[:3] = u.get_Origin(T_6_0) # x, y, z  rcm stands for remote center of motion

    p05   = np.ones((8, 4))
    p6rcm[2] = 0 # takes projection on x,y plane

    for i in range(2):
        p65 = (-1 + 2 * i) * raven_def.RAVEN_IKIN_PARAM[5] * (p6rcm / np.linalg.norm(p6rcm)) # finds the position of the 5th joint with respect to the 6th joint


        p65[-1] = 1

        p05[4 * i][:3] = p05[4 * i + 1][:3] = p05[4 * i + 2][:3] = p05[4 * i + 3][:3] = np.matmul(T_0_6, p65)[:3].squeeze()
    # now we have two unique solutions

    # STEP 2: Computing the prismatic joint j3
    for i in range(int(raven_def.RAVEN_IKSOLS / 4)):
        insertion = float(0)
        insertion += np.linalg.norm(p05[4 * i][:3])
        # print("insertion = ")
        # print(insertion)

        # checking the physical boundary of how much it can insert
        if insertion <= raven_def.RAVEN_IKIN_PARAM[5]:
            # like the C++ RAVEN IK (r2_kinematics.cpp), the whole IK fails here
            print("WARNING: Raven mechanism at RCM singularity. IK failing.")
            return None

        # sets prismatic joint as higher or lower depending on how high or low the 5th frame is relative to the 0th frame
        iksol[4 * i + 0][2] = iksol[4 * i + 1][2] = - raven_def.RAVEN_IKIN_PARAM[4] - insertion
        iksol[4 * i + 2][2] = iksol[4 * i + 3][2] = - raven_def.RAVEN_IKIN_PARAM[4] + insertion
        # now we have 4 unique solutions

    # STEP 3: Evaluate Theta 2
    for i in np.arange(0, raven_def.RAVEN_IKSOLS, 2): # now we have to look at 4 unique solutions
        z0p5 = float(p05[i][2])  # <-- zth position of the 5th joint with respect to the 0th joint
        d = float(iksol[i][2] + raven_def.RAVEN_IKIN_PARAM[4])
        cth2_nom = float(( z0p5 / d) + raven_def.RAVEN_IKIN_PARAM[1] * raven_def.RAVEN_IKIN_PARAM[3])
        cth2_den = float(raven_def.RAVEN_IKIN_PARAM[0] * raven_def.RAVEN_IKIN_PARAM[2])
        cth2 = float(-cth2_nom / cth2_den) # cosine(theta2)


        # Smooth roundoff errors at +/- 1. <-- exceeding one or less than negative 1 by a little bit, still valid
        if cth2 > 1 and cth2 < 1 + raven_def.Eps:
            cth2 = 1
        elif cth2 < -1 and cth2 > -1 - raven_def.Eps:
            cth2 = -1
        if cth2 > 1 or cth2 < - 1:
            ikcheck[i] = ikcheck[i + 1] = False
        else:
            iksol[i][1] = m.acos(cth2)
            iksol[i + 1][1] = - m.acos(cth2)
        i += 1
        # now we have 8 unique solutions

    # STEP 4: Compute Theta 1
    for i in range(raven_def.RAVEN_IKSOLS):
        if ikcheck[i] == False:
            continue
        cth2 = float(m.cos(iksol[i][1]))
        sth2 = float(m.sin(iksol[i][1]))
        d = float(iksol[i][2] + raven_def.RAVEN_IKIN_PARAM[4])
        BB1 = sth2 * raven_def.RAVEN_IKIN_PARAM[2]
        xyp05 = p05[i]
        xyp05[2] = 0
        BB2 = cth2 * raven_def.RAVEN_IKIN_PARAM[1] * raven_def.RAVEN_IKIN_PARAM[2] - raven_def.RAVEN_IKIN_PARAM[0] * raven_def.RAVEN_IKIN_PARAM[3]
        if arm == 0:
            Bmx = np.matrix([[ BB1, BB2, 0],
                             [-BB2, BB1, 0],
                             [   0,   0, 1]])
        else:
            Bmx = np.matrix([[BB1,  BB2, 0],
                             [BB2, -BB1, 0],
                             [  0,    0, 1]])
        scth1 = np.ones(4, dtype = 'float')
        scth1[:3] = np.matmul(np.linalg.inv(Bmx), xyp05[:3]) * (1 / d)
        iksol[i][0] = m.atan2(scth1[1], scth1[0])

    # STEP 5: Compute Theta 4, 5, 6

    for i in range(raven_def.RAVEN_IKSOLS):
        if ikcheck[i] == False:
            continue
        # compute T_0_3
        dh_theta[0] = iksol[i][0]
        dh_theta[1] = iksol[i][1]
        dh_d[2]     = iksol[i][2]

        T_0_3 = fwd_trans(0, 3, dh_alpha, dh_theta, dh_a, dh_d)

        T_3_6 = np.matmul(np.linalg.inv(T_0_3), T_0_6)
        T_3_6_B = u.get_Basis(T_3_6)
        T_3_6_O = u.get_Origin(T_3_6)
        c5 = -float(T_3_6_B[2, 2])
        s5 = float(T_3_6_O[2] - raven_def.RAVEN_IKIN_PARAM[4]) / float(raven_def.RAVEN_IKIN_PARAM[5])

        if m.fabs(c5) > raven_def.Eps:
            c4 = float(T_3_6_O[0]) / float(raven_def.RAVEN_IKIN_PARAM[5] * c5)
            s4 = float(T_3_6_O[1]) / float(raven_def.RAVEN_IKIN_PARAM[5] * c5)
        else:
            c4 = T_3_6_B[0, 2] / s5
            s4 = T_3_6_B[1, 2] / s5
        iksol[i][3] = m.atan2(s4, c4)
        iksol[i][4] = m.atan2(s5, c5)
        if m.fabs(s5) > raven_def.Eps:
            c6 = T_3_6_B[2, 0] / s5
            s6 = -T_3_6_B[2, 1] / s5
        else:
            dh_theta[3] = iksol[i][3]
            dh_theta[4] = iksol[i][4]
            T_0_5 = np.matmul(T_0_3, fwd_trans(3, 5, dh_alpha, dh_theta, dh_a, dh_d))
            T_5_6 = np.matmul(np.linalg.inv(T_0_5), T_0_6)
            c6 = u.get_Basis(T_5_6)[0,0]
            s6 = u.get_Basis(T_5_6)[2,0]
        iksol[i][5] = m.atan2(s6, c6)

    # pick the solution closest to the current joints, compared in DH space (as the C++ RAVEN IK does)
    if curr_jp is None:
        curr_jp = raven_def.HOME_JOINTS
    curr_dh = joint_to_dhvalue(np.asarray(curr_jp, dtype = 'float'), arm, raven_def)[1]
    best_err, best_idx = find_best_solution(curr_dh, iksol, ikcheck, raven_def)
    if best_idx < 0:
        return None
    return dhvalue_to_joint(iksol[best_idx], input_gangle, arm, raven_def)

def dhvalue_to_joint(dhvalue, gangle, arm, raven_def):
    joint = np.zeros(raven_def.RAVEN_JOINTS, dtype = 'float')
    for i in range(raven_def.RAVEN_JOINTS - 1):
        if i != 2:
            if i == 5:
                if arm == 0:
                    joint[i + 1] = (-dhvalue[i] + gangle) / 2
                    joint[i]     = (dhvalue[i] + gangle) / 2
                else:
                    joint[i] = (-dhvalue[i] + gangle) / 2
                    joint[i + 1] = (dhvalue[i] + gangle) / 2
            else:
                joint[i] = dhvalue[i]
            while joint[i] > m.pi:
                joint[i] -= 2 * m.pi
            while joint[i] < -m.pi:
                joint[i] += 2 * m.pi
        else:
            joint[i] = dhvalue[i]
    # print(joint)
    return apply_joint_limits(joint, raven_def)

def apply_joint_limits(joint, raven_def):
    limited = False
    for i in range(raven_def.RAVEN_JOINTS):
        # if i != 2:
        #     while joint[i] > m.pi:
        #         joint[i] -= 2 * m.pi
        #     while joint[i] < -m.pi:
        #         joint[i] += 2 * m.pi
        if joint[i] < raven_def.RAVEN_JOINT_LIMITS[0][i]:
            joint[i] = raven_def.RAVEN_JOINT_LIMITS[0][i]
            limited = True
        elif joint[i] > raven_def.RAVEN_JOINT_LIMITS[1][i]:
            joint[i] = raven_def.RAVEN_JOINT_LIMITS[1][i]
            limited = True
    return joint, limited


def find_best_solution(curr_jp, iksol, ikcheck, raven_def):
    best_err = float(1E10)
    # is this supposed to be best_idx?
    best_idx = -1

    for i in range(len(iksol)):
        error = 0
        if ikcheck[i] == True:
            for j in range(raven_def.RAVEN_JOINTS - 1):
                if j == 2:
                    error += 100 * (iksol[i][j] - curr_jp[j]) ** 2
                else:
                    diff = float(iksol[i][j] - curr_jp[j])
                    while diff > m.pi:
                        diff -= 2 * m.pi
                    while diff < -m.pi:
                        diff += 2 * m.pi
                    error += diff ** 2
            if error < best_err:
                best_err = error
                best_idx = i

    return best_err, best_idx