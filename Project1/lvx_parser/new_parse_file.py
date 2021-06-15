#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author： 11360
# datetime： 2021/5/27 21:14 
import os
import sys
import numpy
import struct
import time


def parse_version_1_1(fd, verbose = 0):
    bytes_counter = 0
    DATA = fd.read()
    PRIVATE_HEADER_BLOCK = DATA[:5]
    bytes_counter += 5
    FRAME_DURATION, DEVICE_COUNT = struct.unpack("=IB", PRIVATE_HEADER_BLOCK)

    if verbose == 1:
        print(f"FRAME_DURATION is {FRAME_DURATION}")
        print(f"DEVICE COUNT is {DEVICE_COUNT}")

    devices_info = []
    for _ in range(DEVICE_COUNT):
        device_info_block = DATA[bytes_counter:bytes_counter+59]
        bytes_counter += 59

        LIDAR_SN, HUB_SN, DEVICE_IDX, DEVICE_TYPE, EXTRINSIC_ENABLE, ROLL, PITCH, YAW, X, Y, Z = struct.unpack(
            "=16s16sBBBffffff", device_info_block)
        devices_info.append([LIDAR_SN, HUB_SN, DEVICE_IDX, DEVICE_TYPE, EXTRINSIC_ENABLE, ROLL, PITCH, YAW, X, Y, Z])
        if verbose == 1:
            print(_, LIDAR_SN, HUB_SN, DEVICE_IDX, DEVICE_TYPE, EXTRINSIC_ENABLE, ROLL, PITCH, YAW, X, Y, Z)

    points = []

    while True:
        frame_header = DATA[bytes_counter:bytes_counter+24]
        if len(frame_header) != 24:
            break
        current_offset, next_offset, frame_index = struct.unpack("<qqq",frame_header )
        bytes_counter += 24

        frame_points = []
        while True:
            PACKAGE_BEGIN_BLOCK = DATA[bytes_counter:bytes_counter+19]

            if len(PACKAGE_BEGIN_BLOCK) != 19:
                points.extend(frame_points)
                break
            device_idx, version, slot_id, lidar_id, reserved, status_code, timestamp_type, data_type, timestamp = struct.unpack(
                "<BBBBBIBBQ", PACKAGE_BEGIN_BLOCK)

            if version != 5:
                points.extend(frame_points)
                break

            bytes_counter += 19

            if data_type == 0:
                #read 100 points * 13
                for _ in range(100):
                    point_block = DATA[bytes_counter: bytes_counter + 13]
                    if len(point_block) != 13:
                        points.extend(frame_points)
                        break
                    x,y,z,r = struct.unpack("<iiiB",point_block)
                    frame_points.append([x,y,z,r])
                    bytes_counter += 13
            elif data_type == 1:
                #read 100 points * 9
                for _ in range(100):
                    point_block = DATA[bytes_counter: bytes_counter + 9]
                    if len(point_block) != 9:
                        points.extend(frame_points)
                        break
                    depth, theta, phi, r = struct.unpack("<iHHB", point_block)
                    frame_points.append([depth,theta,phi,r])
                    bytes_counter += 13
            elif data_type == 2:
                #read 96 points * 14
                # point_block = fd.read(96 * 14)
                for _ in range(96):
                    point_block = DATA[bytes_counter: bytes_counter + 14]
                    if len(point_block) != 14:
                        points.extend(frame_points)
                        break
                    x, y, z, r, tag = struct.unpack("<iiiBB", point_block)
                    frame_points.append([x,y,z,r,tag])
                    bytes_counter += 14
            elif data_type == 3:
                #read 96 points * 10
                # point_block = fd.read(96 * 10)
                for _ in range(96):
                    point_block = DATA[bytes_counter: bytes_counter + 10]
                    if len(point_block) != 10:
                        points.extend(frame_points)
                        break
                    depth, theta, phi, r, tag = struct.unpack("<iHHBB", point_block)
                    frame_points.append([depth, theta, phi, r, tag])
                    bytes_counter += 10
            elif data_type == 4:
                #read 48 points * 28
                # point_block = fd.read(48 * 28)
                for _ in range(48):
                    point_block = DATA[bytes_counter: bytes_counter + 28]
                    if len(point_block) != 28:
                        points.extend(frame_points)
                        break
                    x1, y1, z1, r1,tag1,x2,y2,z2,r2,tag2 = struct.unpack("<iiiBBiiiBB", point_block)
                    frame_points.append([x1, y1, z1, r1,tag1,x2,y2,z2,r2,tag2])
                    bytes_counter += 28
            elif data_type == 5:
                #read 48 points * 24
                # point_block = fd.read(48 * 24)
                for _ in range(48):
                    point_block = DATA[bytes_counter: bytes_counter + 24]
                    if len(point_block) != 24:
                        points.append(frame_points)
                        break
                    theta, phi, depth1, r, tag1, depth2, r2, tag2, gyro_x = struct.unpack("<iiiBBiBBf", point_block)
                    frame_points.append([theta, phi, depth1, r, tag1, depth2, r2, tag2, gyro_x])
                    bytes_counter += 24
            elif data_type == 6:
                #read 1 point * 48
                point_block = DATA[bytes_counter: bytes_counter + 24]
                if len(point_block) != 24:
                    points.extend(frame_points)
                    break
                gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z = struct.unpack("<ffffff", point_block)
                frame_points.append([gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z])
                bytes_counter += 48
                print(acc_x)
                print(acc_y)
                print(acc_z)
                print(gyro_x)
                print(gyro_y)
                print(gyro_z)
            # else:
            #     print("unknown data type", data_type)

    fd.close()
    return points

def parse_version_1_0(fd, verbose = 0):
    PRIVATE_HEADER_BLOCK = fd.read(1)
    DEVICE_COUNT = struct.unpack("=B", PRIVATE_HEADER_BLOCK)[0]

    if verbose:
        print("DEVICE_COUNT:", DEVICE_COUNT)

    DEVICES_INFO = []

    for device_idx in range(DEVICE_COUNT):
        DEVICE_INFO_BLOCK = fd.read(58)

        LIDAR_SN, HUB_SN, DEVICE_IDX, DEVICE_TYPE, ROLL, PITCH, YAW, X, Y, Z = struct.unpack("=16s16sBBffffff",
                                                                                             DEVICE_INFO_BLOCK)

        # print("LIDAR_SN ", LIDAR_SN)
        # print("HUB_SN ", HUB_SN)
        # print("DEVICE_IDX ", DEVICE_IDX)
        # print("DEVICE TYPE ", DEVICE_TYPE)
        # print("ROLL ", ROLL)
        # print("PITCH ", PITCH)
        # print("YAW ", YAW)
        # print("X ", X)
        # print("Y ", Y)
        # print("Z ", Z)

        DEVICES_INFO.append([LIDAR_SN, HUB_SN, DEVICE_IDX, ROLL, PITCH, YAW, X, Y, Z])

    FRAMES = {}
    current_timestamp = None

    POINTS = []

    start_time = time.time()

    while True:
        FRAME_HEADER_BLOCK = fd.read(32)

        if FRAME_HEADER_BLOCK == b'':
            if verbose:
                print("end of file")
            break

        Current_offset, Next_offset, Frame_index, Package_count = struct.unpack("<qqqq", FRAME_HEADER_BLOCK)
        if verbose:
            print("Frame header info:", Current_offset, Next_offset, Frame_index, Package_count)

        for package_idx in range(Package_count):
            Package_block = fd.read(1319)

            device_idx, Version, Slot_id, LIDAR_id, Reserved, Status_code, Timestamp_type, Data_type, Timestamp = struct.unpack(
                "=BBBBBIBBQ", Package_block[:19])

            if Timestamp != current_timestamp:
                if current_timestamp:
                    FRAMES[current_timestamp] = POINTS
                    POINTS.clear()

                    if verbose:
                        print(f"Считывание одного таймштампа заняло {time.time() - start_time}")
                    start_time = time.time()

                current_timestamp = Timestamp

            points_block = Package_block[19:]
            for idx in range(100):
                point = struct.unpack("<fffB", points_block[idx * 13:(idx + 1) * 13])
                POINTS.append(point)
    if verbose:
        print(f"Всего насчитано: {len(POINTS)} points")

    fd.close()

# FILE_PATH = 'Livox LiDAR - Mid-100 Point Cloud Data %231.lvx'
FILE_PATH = 'short.lvx'

if __name__ == "__main__":

    fd = open(FILE_PATH,"rb")
    # data = fd.read()

    PUBLIC_HEADER_DATA =fd.read(24)
    PUBLIC_HEADER, VERSION_A, VERSION_B, VERSION_C, VERSION_D, MAGIC_CODE = struct.unpack("=16sccccI",PUBLIC_HEADER_DATA)
    # print(PUBLIC_HEADER)
    # print(hex(MAGIC_CODE))

    VERSION_A = int.from_bytes(VERSION_A, "little")
    VERSION_B = int.from_bytes(VERSION_B, "little")
    VERSION_C = int.from_bytes(VERSION_C,"little")
    VERSION_D = int.from_bytes(VERSION_D,"little")

    print(f"version {VERSION_A}_{VERSION_B}_{VERSION_C}_{VERSION_D}")

    assert VERSION_A == 1, "VERSION_A"
    assert VERSION_B in [0,1], "VERSION_B"
    assert VERSION_C == 0, "VERSION_C"
    assert VERSION_D == 0, "VERSION_D"
    assert hex(MAGIC_CODE) == "0xAC0EA767".lower(), "MAGIC_CODE"

    if VERSION_A == 1 and VERSION_B == 0:
        parse_version_1_0(fd)
    elif VERSION_A == 1 and VERSION_B == 1:
        parse_version_1_1(fd)
