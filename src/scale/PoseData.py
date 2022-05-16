import numpy as np
import math


def eulerAnglesToRotationMatrix(x, y, z):
    theta = np.zeros((3, 1), dtype=np.float64)  #
    theta[0] = x
    theta[1] = y
    theta[2] = z
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])
    R = np.dot(R_z, np.dot(R_y, R_x))
    # rvecs = np.zeros((1, 1, 3), dtype=np.float64)
    # rvecs,_ = cv2.Rodrigues(R, rvecstmp)
    return R


#### read the key frame trajectory ############
def LoadPoseData(keyframefile, timefile):
    with open(keyframefile, 'r') as f:
        list_read = f.readlines()

    v_x_pos = []
    v_y_pos = []
    v_z_pos = []
    v_roll = []
    v_pitch = []
    v_yaw = []
    kf_time_list = []
    kf_idx_list = []
    kf_pose_matrix_list = []

    for i in range(len(list_read)):
        j = i
        a6 = list_read[j].split(' ')  # 8个数

        v_x_pos.append(float(a6[1]))
        v_y_pos.append(float(a6[2]))
        v_z_pos.append(float(a6[3]))
        kf_time_list.append(float(a6[0]))
        w = float(a6[7])  # q0
        x = float(a6[4])  # q1
        y = float(a6[5])  # q2
        z = float(a6[6])  # q3
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if (sinp > 1.0):
            pitch = 1
        elif (sinp < -1.0):
            pitch = -1
        else:
            pitch = math.asin(sinp)
        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # roll=np.rad2deg(roll)
        # pitch=np.rad2deg(pitch)
        # yaw=np.rad2deg(yaw)
        v_roll.append(roll)
        v_pitch.append(pitch)
        v_yaw.append(yaw)

    total_time_list = []
    with open(timefile, 'r') as f:
        list_read = f.readlines()

    for i in range(len(list_read)):
        a6 = list_read[i].split(' ')
        total_time_list.append(float("{:.{}f}".format(float(a6[0]), 6)))
        # print(float(a6[0]))

    for i in range(len(kf_time_list)):
        idx = total_time_list.index(float(kf_time_list[i]))
        #idx = '%06d' % idx
        kf_idx_list.append(idx)
        x = v_roll[i]
        y = v_pitch[i]
        z = v_yaw[i]
        kf_R = eulerAnglesToRotationMatrix(x, y, z)
        kf_T = np.array((v_x_pos[i], v_y_pos[i], v_z_pos[i])).reshape(3, 1)
        kf_pose = np.concatenate((kf_R, kf_T), axis=1)
        kf_pose_matrix_list.append(kf_pose)

    return kf_idx_list, kf_pose_matrix_list