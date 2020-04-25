import math
import random

'''
IMPORTANT:
6 axis robot.
Right handed coordinate systems.

When all joints are at zero, the upper arm and the lower arm are at 90 degrees from each other and all 
frames t1-t6 are orientated as the world coordinate system.

This script is a prototyping script for the forward and inverse kinematics and will not be part of the final robot.
'''

L1 = 0.0
L2 = 0.3
L3 = 0.2
L4 = 0.0
L5 = 0.1
EPS3 = 1.0e-3
EPS4 = 1.0e-4
EPS5 = 1.0e-5
EPS6 = 1.0e-6
EPS10 = 1.0e-10

THETA_1_RANGE_U = math.pi/2 - EPS5
THETA_1_RANGE_L = -math.pi/2 + EPS5
THETA_2_RANGE_U = math.pi/2
THETA_2_RANGE_L = -math.pi/2
THETA_3_RANGE_U = math.pi/2
THETA_3_RANGE_L = -math.pi/2
THETA_4_RANGE_U = math.pi - EPS5
THETA_4_RANGE_L = -math.pi + EPS5
THETA_5_RANGE_U = math.pi/2
THETA_5_RANGE_L = 0
THETA_6_RANGE_U = math.pi - EPS5
THETA_6_RANGE_L = -math.pi + EPS5

def inverseKin123(p):
    Lp = math.sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2])
    theta1 = math.atan2(p[1], p[0])

    theta1_flipped = False
    # If theta1 is above pi/2 or below -pi/2, then we try to reach the point by bending backwards, so theta1 is rotated pi rads.
    if (theta1 > THETA_1_RANGE_U + EPS6):
        theta1 -= math.pi
        theta1_flipped = True
    elif (theta1 < THETA_1_RANGE_L - EPS6):
        theta1 += math.pi
        theta1_flipped = True

    # Might be a few decimal points under or over 1 due to double precision error. Make that excaclty +-1 to ensure acos works.
    l = almost_one_to_one(math.sqrt(p[0]*p[0] + p[1]*p[1])/Lp) # acos(l) is angle between p and xy plane
    m = almost_one_to_one((L2*L2 + Lp*Lp - L3*L3) / (2*L2*Lp)) # L2, Lp, L3 form triangle of known sides. Look up triangle SSS for reference.
    p_xy_theta = 0

    if (p[2] < 0): # We are under the xy-plane.
        p_xy_theta = -math.acos(l)
    elif (theta1_flipped): # Bending backwards.
        p_xy_theta = math.pi - math.acos(l)
    else:
        p_xy_theta = math.acos(l)

    theta2 = math.pi/2 - p_xy_theta - math.acos(m)

    # Might be a few decimal points under or over 1 due to double precision error. Make that excaclty +-1 to ensure acos works.
    n = almost_one_to_one((L2*L2 + L3*L3 - Lp*Lp) / (2*L2*L3)) # L2, Lp, L3 form triangle of known sides. Look up triangle SSS for reference.
    theta3 = math.pi/2 - math.acos(n)

    return [theta1, theta2, theta3]

def almost_one_to_one(a):
    if (abs(abs(a) - 1) < EPS10):
        return a / abs(a)
    return a

def inverseKinFull(tcp_transform):
    m_temp = getUnityMatrix()
    m_temp[0][3] = -L5
    # p is pos of global t4
    p = get_pos(mat_mul(tcp_transform, m_temp))

    # Get theta1-3 and t1-3 
    theta1_3 = inverseKin123(p)
    t1, t2, t3 = forwardKinematics123(theta1_3)

    t4_unrot_global = mat_mul_list([t1, t2, t3, getT4(0)])  # Pos and rot x axis are already correct.

    # p_tcp is the vector from p to tcp
    p_tcp = vec_sub(get_pos(tcp_transform), p)

    # p_t3t4 is the vecor from t3 to t4 in global coordinates
    p_t3t4 = normalize(get_rot_x(t4_unrot_global))

    # Axis 4:
    t4ry = vec_cross(p_t3t4, p_tcp)
    if (get_len(t4ry) < EPS6): #In case p_t3t4 and p_tcp are parallel (theta5 zero), then 4ry is same ias 3ry, or what 4ry_unrot is already.
        t4ry = get_rot_y(t4_unrot_global)

    t4ry = normalize(t4ry)
    t4rz = vec_cross(get_rot_x(t4_unrot_global), t4ry)
    t4rz = normalize(t4rz)
    
    t4_global = mat_copy(t4_unrot_global) #rot x is already correct, just apply rot y and rot z
    set_rot_y(t4_global, t4ry)
    set_rot_z(t4_global, t4rz)

    s4 = vec_scalar_prod(get_rot_y(t4_global), get_rot_z(t4_unrot_global))
    if (is_close_to_zero(s4)): # In case rot4y and unrot4_z are perpendicular, use rot4y and unrot4_y to decide the sign.
        s4 = vec_scalar_prod(get_rot_y(t4_global), get_rot_y(t4_unrot_global))
    theta4_sign = get_sign(s4)
    theta4 = theta4_sign * get_angle(get_rot_y(t4_unrot_global), get_rot_y(t4_global))

    # Axis 5:
    t5_global = mat_mul(t4_global, getT5(0)) #pos and rot y is correct.

    set_rot_x(t5_global, normalize(get_rot_x(tcp_transform)))
    set_rot_z(t5_global, normalize(vec_cross(get_rot_x(t5_global), get_rot_y(t5_global))))

    theta5 = get_angle(get_rot_x(t4_global), get_rot_x(t5_global))

    # Axis 6:
    t6_global = tcp_transform # t6 global is by definition same as tcp_transform 

    s6 = vec_scalar_prod(get_rot_y(t6_global), get_rot_z(t5_global))
    if (is_close_to_zero(s6)): # In case rot6y and rot5z are perpendicular, use rot6y and rot5_y to decide the sign.
        s6 = vec_scalar_prod(get_rot_y(t6_global), get_rot_y(t5_global))
    theta6_sign = get_sign(s6)

    theta6 = theta6_sign * get_angle(get_rot_y(t5_global), get_rot_y(t6_global))

    theta1_3 = inverseKin123(p)
    return [theta1_3[0], theta1_3[1], theta1_3[2], theta4, theta5, theta6] 

def mat_copy(s):
    r = getUnityMatrix()
    for i in range(0,4):
        for j in range(0,4):
            r[i][j] = s[i][j]
    return r

def normalize(vec):
    len = get_len(vec)
    return [vec[0]/ len, vec[1]/ len, vec[2]/ len]

def get_len(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

def get_sign(a):
    if (a > 0):
        return 1
    return -1

def get_angle(v1, v2):
    return math.acos(vec_scalar_prod(v1, v2))

def get_rot_x(m):
    return [m[0][0], m[1][0], m[2][0]]

def get_rot_y(m):
    return [m[0][1], m[1][1], m[2][1]]

def get_rot_z(m):
    return [m[0][2], m[1][2], m[2][2]]

def get_pos(m):
    return [m[0][3], m[1][3], m[2][3]] 

def set_rot_x(m, x_vec):
    m[0][0] = x_vec[0]
    m[1][0] = x_vec[1]
    m[2][0] = x_vec[2]

def set_rot_y(m, y_vec):
    m[0][1] = y_vec[0]
    m[1][1] = y_vec[1]
    m[2][1] = y_vec[2]

def set_rot_z(m, z_vec):
    m[0][2] = z_vec[0]
    m[1][2] = z_vec[1]
    m[2][2] = z_vec[2]

def set_pos(m, pos):
    m[0][3] = pos[0]
    m[1][3] = pos[1]
    m[2][3] = pos[2]

def get_pos(mat):
    return [mat[0][3], mat[1][3], mat[2][3]]

def vec_sub(v1, v2):
    return [v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]]

def vec_cross(v1, v2):
    return [v1[1] * v2[2] - v1[2] * v2[1],
            -(v1[0] * v2[2] - v1[2] * v2[0]),
            v1[0] * v2[1] - v1[1] * v2[0]]

def vec_scalar_prod(v1, v2):
    return almost_one_to_one(v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2])

def printMatrix(m):
    print(m[0])
    print(m[1])
    print(m[2])
    print(m[3])
    print("\n")

def getUnityMatrix():
    return [[1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]

def getT1(theta1):
    return [[math.cos(theta1), -math.sin(theta1), 0, 0],
            [math.sin(theta1), math.cos(theta1), 0, 0],
            [0,                0,                1, 0],
            [0,                0,                0, 1]]

def getT2(theta2):
    return [[math.cos(theta2),  0, math.sin(theta2), 0],
            [0,                 1, 0,                0],
            [-math.sin(theta2), 0, math.cos(theta2), L1],
            [0,                 0, 0,                1]]

def getT3(theta3):
    return [[math.cos(theta3),  0, math.sin(theta3), 0],
            [0,                 1, 0,                0],
            [-math.sin(theta3), 0, math.cos(theta3), L2],
            [0,                 0, 0,                1]]

def getT4(theta4):
    return [[1, 0,                0,                 L3],
            [0, math.cos(theta4), -math.sin(theta4), 0],
            [0, math.sin(theta4), math.cos(theta4),  0],
            [0, 0,                0,                 1]]

def getT5(theta5):
    return [[math.cos(theta5),  0, math.sin(theta5), L4],
            [0,                 1, 0,                0],
            [-math.sin(theta5), 0, math.cos(theta5), 0],
            [0,                 0, 0,                1]]

def getT6(theta6):
    return [[1, 0,                0,                 L5],
            [0, math.cos(theta6), -math.sin(theta6), 0],
            [0, math.sin(theta6), math.cos(theta6),  0],
            [0, 0,                0,                 1]]

def mat_mul(m1, m2):
    m00 = m1[0][0] * m2[0][0] + m1[0][1] * m2[1][0] + m1[0][2] * m2[2][0] + m1[0][3] * m2[3][0]
    m01 = m1[0][0] * m2[0][1] + m1[0][1] * m2[1][1] + m1[0][2] * m2[2][1] + m1[0][3] * m2[3][1]
    m02 = m1[0][0] * m2[0][2] + m1[0][1] * m2[1][2] + m1[0][2] * m2[2][2] + m1[0][3] * m2[3][2]
    m03 = m1[0][0] * m2[0][3] + m1[0][1] * m2[1][3] + m1[0][2] * m2[2][3] + m1[0][3] * m2[3][3]

    m10 = m1[1][0] * m2[0][0] + m1[1][1] * m2[1][0] + m1[1][2] * m2[2][0] + m1[1][3] * m2[3][0]
    m11 = m1[1][0] * m2[0][1] + m1[1][1] * m2[1][1] + m1[1][2] * m2[2][1] + m1[1][3] * m2[3][1]
    m12 = m1[1][0] * m2[0][2] + m1[1][1] * m2[1][2] + m1[1][2] * m2[2][2] + m1[1][3] * m2[3][2]
    m13 = m1[1][0] * m2[0][3] + m1[1][1] * m2[1][3] + m1[1][2] * m2[2][3] + m1[1][3] * m2[3][3]

    m20 = m1[2][0] * m2[0][0] + m1[2][1] * m2[1][0] + m1[2][2] * m2[2][0] + m1[2][3] * m2[3][0]
    m21 = m1[2][0] * m2[0][1] + m1[2][1] * m2[1][1] + m1[2][2] * m2[2][1] + m1[2][3] * m2[3][1]
    m22 = m1[2][0] * m2[0][2] + m1[2][1] * m2[1][2] + m1[2][2] * m2[2][2] + m1[2][3] * m2[3][2]
    m23 = m1[2][0] * m2[0][3] + m1[2][1] * m2[1][3] + m1[2][2] * m2[2][3] + m1[2][3] * m2[3][3]

    m30 = m1[3][0] * m2[0][0] + m1[3][1] * m2[1][0] + m1[3][2] * m2[2][0] + m1[3][3] * m2[3][0]
    m31 = m1[3][0] * m2[0][1] + m1[3][1] * m2[1][1] + m1[3][2] * m2[2][1] + m1[3][3] * m2[3][1]
    m32 = m1[3][0] * m2[0][2] + m1[3][1] * m2[1][2] + m1[3][2] * m2[2][2] + m1[3][3] * m2[3][2]
    m33 = m1[3][0] * m2[0][3] + m1[3][1] * m2[1][3] + m1[3][2] * m2[2][3] + m1[3][3] * m2[3][3]

    return [[m00, m01, m02, m03],
            [m10, m11, m12, m13],
            [m20, m21, m22, m23],
            [m30, m31, m32, m33]]

def forwardKinematics123(theta1_3):
    t1 = getT1(theta1_3[0])
    t2 = getT2(theta1_3[1])
    t3 = getT3(theta1_3[2])

    return t1, t2, t3

def mat_mul_list(m_l):
    if (len(m_l) == 1):
        return m_l[0]
    return mat_mul(m_l[0], mat_mul_list(m_l[1 : len(m_l)]))

def forwardKinematicsFull(theta1_6):
    return mat_mul_list([getT1(theta1_6[0]), getT2(theta1_6[1]), getT3(theta1_6[2]), getT4(theta1_6[3]), getT5(theta1_6[4]), getT6(theta1_6[5])])

def forwardKinematicsFull_debug(theta1_6):
    t1g = getT1(theta1_6[0])
    t2g = mat_mul(t1g, getT2(theta1_6[1]))
    t3g = mat_mul(t2g, getT3(theta1_6[2]))
    t4g = mat_mul(t3g, getT4(theta1_6[3]))
    t5g = mat_mul(t4g, getT5(theta1_6[4]))
    t6g = mat_mul(t5g, getT6(theta1_6[5]))
    print("thetas; ", theta1_6)
    print("t1g")
    printMatrix(t1g)
    print("t2g")
    printMatrix(t2g)
    print("t3g")
    printMatrix(t3g)
    print("t4g")
    printMatrix(t4g)
    print("t5g")
    printMatrix(t5g)
    print("t6g")
    printMatrix(t6g)

def is_mat_same(m1, m2):
    for i in range(0,4):
        for j in range(0,4):
            if (abs(m1[i][j] - m2[i][j]) > EPS3):
                return False
    return True

def is_ang_same(a1, a2):
    for i in range(0,6):
        if (abs(a1[i] - a2[i]) > EPS3):
            return False
    return True


def is_angs_within_limits(ang):
    if (ang[0] < THETA_1_RANGE_L - EPS6 or ang[0] > THETA_1_RANGE_U + EPS6):
        return False
    if (ang[1] < THETA_2_RANGE_L - EPS6 or ang[1] > THETA_2_RANGE_U + EPS6):
        return False
    if (ang[2] < THETA_3_RANGE_L - EPS6 or ang[2] > THETA_3_RANGE_U + EPS6):
        return False
    if (ang[3] < THETA_4_RANGE_L - EPS6 or ang[3] > THETA_4_RANGE_U + EPS6):
        return False
    if (ang[4] < THETA_5_RANGE_L - EPS6 or ang[4] > THETA_5_RANGE_U + EPS6):
        return False
    if (ang[5] < THETA_6_RANGE_L - EPS6 or ang[5] > THETA_6_RANGE_U + EPS6):
        return False
    return True

# Get input angles, do forward kinematics for those, then do inverse kinematics
# for the result of the forward kinematics and match up the input to the output.
def runKinTest(ang):
    tcp = forwardKinematicsFull(ang)

    inv_kin_thetas = inverseKinFull(tcp)

    ang_result = is_angs_within_limits(inv_kin_thetas) and is_ang_same(inv_kin_thetas, ang)
    if (ang_result):
        print("Test passed! Thetas: ", ang)
        return True
    else:
        print("Test failed.")
        print("Test theta in was: ", ang)
        print("Test theta out was: ", inv_kin_thetas)
        print("Test Tcp in was: ")
        printMatrix(tcp)

        return False

def is_close_to_zero(a):
    return abs(a) < EPS6

def is_singularity(ang):
    if (is_close_to_zero(ang[4])):
        return True
    return False

def filter(ang):
    theta1Allow = True#is_close_to_zero(ang[0])
    theta2Allow = True#is_close_to_zero(ang[1])
    theta3Allow = True#is_close_to_zero(ang[2])
    theta4Allow = True#is_close_to_zero(ang[3])
    theta5Allow = not (is_close_to_zero(ang[4]) and not is_close_to_zero(ang[3])) # If 5 is zero (singularity) and 4 is non-zero, skip!
    theta6Allow = True#is_close_to_zero(ang[5])
    if (theta1Allow and theta2Allow and theta3Allow and theta4Allow and theta5Allow and theta6Allow):
        return True
    return False

def run_tests_right_angles():
    for t1 in range(0, 1):
        for t2 in range(-1, 2):
            for t3 in range(-1, 2):
                for t4 in range(-1, 2):
                    for t5 in range(0, 2):
                        for t6 in range(-1, 2):
                            t1_ = t1 * math.pi/2
                            t2_ = t2 * math.pi/2
                            t3_ = t3 * math.pi/2
                            t4_ = t4 * math.pi/2
                            t5_ = t5 * math.pi/2
                            t6_ = t6 * math.pi/2
                            if (filter([t1_, t2_, t3_, t4_, t5_, t6_])):
                                    if (not runKinTest([t1_, t2_, t3_, t4_, t5_, t6_])):
                                        return

def test_random_angles(num_runs):
    for i in range(0, num_runs):
        theta1 = random.uniform(THETA_1_RANGE_L + EPS5, THETA_1_RANGE_U - EPS5)
        theta2 = random.uniform(THETA_2_RANGE_L + EPS5, THETA_2_RANGE_U - EPS5)
        theta3 = random.uniform(THETA_3_RANGE_L + EPS5, THETA_3_RANGE_U - EPS5)
        theta4 = random.uniform(THETA_4_RANGE_L + EPS5, THETA_4_RANGE_U - EPS5)
        theta5 = random.uniform(THETA_5_RANGE_L + EPS5, THETA_5_RANGE_U - EPS5)
        theta6 = random.uniform(THETA_6_RANGE_L + EPS5, THETA_6_RANGE_U - EPS5)
        
        if (filter([theta1, theta2, theta3, theta4, theta5, theta6])):
            print("Test starting with angles: ", [theta1, theta2, theta3, theta4, theta5, theta6])
            if (not runKinTest([theta1, theta2, theta3, theta4, theta5, theta6])):
                return
        

def main():
    #run_tests_right_angles()
    test_random_angles(10000)
if __name__ == "__main__":
    main()