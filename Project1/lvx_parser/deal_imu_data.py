#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author： 11360
# datetime： 2021/5/27 19:06 
import pickle
import numpy as np
import matplotlib.pyplot as plt

# Gyro unit: rad/s
# acc unit: g

with open("./output/acc_x.pkl", "rb") as f:
    acc_x_array = np.array([pickle.load(f) * 9.8])


with open("./output/acc_y.pkl", "rb") as f:
    acc_y_array = np.array([pickle.load(f) * 9.8])

with open("./output/acc_z.pkl", "rb") as f:
    acc_z_array = np.array([pickle.load(f) * 9.8])

acc_array = np.concatenate([acc_x_array, acc_y_array, acc_z_array], axis=0)
print(acc_array.shape)
with open("./output/gyro_x.pkl", "rb") as f:
    gyro_x_array = np.array([pickle.load(f)])

with open("./output/gyro_y.pkl", "rb") as f:
    gyro_y_array = np.array([pickle.load(f)])

with open("./output/gyro_z.pkl", "rb") as f:
    gyro_z_array = np.array([pickle.load(f)])

gyro_array = np.concatenate([gyro_x_array, gyro_y_array, gyro_z_array], axis=0)
print(gyro_array.shape)


def cal_camera_pos_rotation(acc_array, gyro_array):
    dt = 0.005
    R_t = np.eye(3) # 初始位姿
    length = acc_array.shape[1]
    print(length)
    g_array = np.ones([3, 1]) * 9.8
    g_array[0, 0] = 0
    g_array[1, 0] = 0

    V_t = np.zeros([3, 1])
    pos_t = np.zeros([3, 1])
    # ##################################################
    R_lst = [R_t]
    # t_lst = np.array([pos_t])
    t_lst = [pos_t]

    # for t in range(5):
    for t in range(length - 1):
        a_local_coord_t = np.reshape(acc_array[:, t], [3, 1])
        # print("a_local_coord_t.shape = ", a_local_coord_t.shape)

        a_world_coord_t = R_t @ a_local_coord_t - g_array
        # print("a_world_coord_t.shape = ", a_world_coord_t.shape)
        # ######################
        # calculate averate radii velocity
        # ######################
        angle_v_t = gyro_array[:, t]
        angle_v_t_1 = gyro_array[:, t + 1]
        average_angle_v = (angle_v_t + angle_v_t_1) / 2

        # ##############################
        # next time R
        # angle_t = average_angle_v * dt
        # rotate_t = cal_rotate_matrix(angle_array=angle_t)
        rotate_t = cal_rotate_matrix(average_angle_v, dt=dt)
        R_t_1 = R_t @ rotate_t
        # R_t_1 = rotate_t @ R_t


        R_lst.append(R_t_1)
        a_local_coord_t_1 = np.reshape(acc_array[:, t + 1], [3, 1])
        # a_world_coord_t_1 = R_t_1 @ a_local_coord_t_1 - g_array
        # R * a = I * A_W
        a_world_coord_t_1 = R_t_1 @ a_local_coord_t_1 - g_array

        average_a_world_coord = (a_world_coord_t_1 + a_world_coord_t) / 2
        # print("average_a_world_coord.shape = ", average_a_world_coord.shape)
        # #######################################
        pos_t_1 = pos_t + V_t * dt + 0.5 * average_a_world_coord * dt ** 2
        # print("pos_t_1.shape = ", pos_t_1.shape)
        V_t_1 = V_t + average_a_world_coord * dt
        t_lst.append(pos_t_1)
        # #####
        V_t = V_t_1
        R_t = R_t_1
    return R_lst, t_lst

def cal_rotate_matrix(angle_array, dt):
    """
    :param angle_array: convert angle_array to rotate matrix
    angle velocity array (cross product)
    :return: rotate matrix
    """
    alpha = angle_array[0]
    betta = angle_array[1]
    gamma = angle_array[2]
    matrix = np.array([[0, -gamma, betta],
                       [gamma, 0, -alpha],
                       [-betta, alpha, 0]])
    rotate_matrix = np.eye(3) + matrix * dt
    return rotate_matrix


def mean_filter(n, acc_x_array):
    acc_x_array = np.convolve(acc_x_array, np.ones((n,)) / n, mode="same")
    return acc_x_array


length = acc_x_array.shape[1]
R_lst, pos_lst = cal_camera_pos_rotation(acc_array, gyro_array)
# x_lst = np.array(pos_lst)
pos_x = np.array([pos[0] for i, pos in enumerate(pos_lst)]).reshape(-1, )
pos_y = np.array([pos[1] for i, pos in enumerate(pos_lst)]).reshape(-1, )
pos_z = np.array([pos[2] for i, pos in enumerate(pos_lst)]).reshape(-1, )

# print(x_lst.shape)
# x_lst = x_lst.tolist()
# print("translate = ", pos_lst[699])
# print("rotate = ", R_lst[699])

print("rotate = ", R_lst[1390])
print("translate = ", pos_lst[1390])

# print(R_lst[699] @ np.transpose(R_lst[699]))
# 200 Hz, t = 1 / 200
# length = 6
time_step = [i * 0.005 for i in range(length)]
print(time_step)
plt.plot(time_step, pos_x, label="pos_x")
plt.plot(time_step, pos_y, label="pos_y")
plt.plot(time_step, pos_z, label="pos_z")

plt.legend()
plt.show()




# print(cal_rotate_matrix([np.pi / 2, 0, 0]))
