from _frame import FrameHeader, Frame, DataType, Point0, Point1, Point2, Point3, Point4, Point5, Point6, Package
import os
from datetime import datetime
import numpy as np
import pickle


class PublicHeader:
    def __init__(self, bs):
        self.bs = bs

    @property
    def file_signature(self):
        return self.bs[:16].decode()

    @property
    def version_a(self):
        return int.from_bytes(self.bs[16:17], 'little')

    @property
    def version_b(self):
        return int.from_bytes(self.bs[17:18], 'little')

    @property
    def version_c(self):
        return int.from_bytes(self.bs[18:19], 'little')

    @property
    def version_d(self):
        return int.from_bytes(self.bs[19:20], 'little')

    @property
    def magic_code(self):
        return int.from_bytes(self.bs[20:24], 'little')


class PrivateHeader:
    def __init__(self, bs):
        self.bs = bs

    @property
    def frame_duration(self):
        return int.from_bytes(self.bs[:4], 'little')

    @property
    def device_count(self):
        return int.from_bytes(self.bs[4:5], 'little')


class DeivceInfo:
    def __init__(self, bs):
        self.bs: bytes = bs

    @property
    def lidar_sn_code(self):
        return self.bs[:16].decode()

    @property
    def hub_sn_code(self):
        return self.bs[16:32].decode()

    @property
    def device_index(self):
        return int.from_bytes(self.bs[32:33], 'little')

    @property
    def device_type(self):
        return int.from_bytes(self.bs[33:34], 'little')

    @property
    def extrinsic_enable(self):
        return int.from_bytes(self.bs[34:35], 'little')

    @property
    def roll(self):
        bs = self.bs[35:39]
        hs = ''.join(['%02X' % x for x in bs])
        return float.fromhex(hs)

    @property
    def pitch(self):
        bs = self.bs[39:43]
        hs = ''.join(['%02X' % x for x in bs])
        return float.fromhex(hs)

    @property
    def yaw(self):
        bs = self.bs[43:47]
        hs = ''.join(['%02X' % x for x in bs])
        return float.fromhex(hs)

    @property
    def x(self):
        bs = self.bs[47:51]
        hs = ''.join(['%02X' % x for x in bs])
        return float.fromhex(hs)

    @property
    def y(self):
        bs = self.bs[51:55]
        hs = ''.join(['%02X' % x for x in bs])
        return float.fromhex(hs)

    @property
    def z(self):
        bs = self.bs[55:59]
        hs = ''.join(['%02X' % x for x in bs])
        return float.fromhex(hs)


class LvxFile:
    def __init__(self, fp):
        self.fp = open(fp, 'rb')

    @property
    def public_header_block(self):
        # seek(offset),也就是代表需要移动偏移的字节数
        self.fp.seek(0)
        return PublicHeader(self.fp.read(24))

    @property
    def private_header_block(self):
        self.fp.seek(24)
        return PrivateHeader(self.fp.read(5))

    @property
    def device_info_block(self):
        self.fp.seek(29)
        for _ in range(self.private_header_block.device_count):
            yield DeivceInfo(self.fp.read(59))

    @property
    def point_data_block(self):
        current_offset = 29 + 59 * int(self.private_header_block.device_count)
        self.fp.seek(current_offset)
        frame_header = FrameHeader(self.fp.read(24))
        assert frame_header.current_offset == current_offset

        while frame_header.next_offset:
            self.fp.seek(current_offset)
            yield Frame(self.fp.read(frame_header.next_offset - current_offset))
            current_offset = frame_header.next_offset
            frame_header = FrameHeader(self.fp.read(24))


def asdict(obj):
    d = {}
    for attr in dir(obj):
        if not attr.startswith('__') and not attr.startswith('_'):
            d[attr] = getattr(obj, attr)
    return d


def get_imu(lvxfile, outdir):
    if not os.path.exists(outdir):
        os.makedirs(outdir)
    lf = LvxFile(lvxfile)
    data_type = DataType.IMU_INFO
    # ACC_X: 加速度计X轴分量
    # GYR_X：绕X轴旋转角速度
    attributes = ['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']
    acc_x_lst = []
    acc_y_lst = []
    acc_z_lst = []
    gyro_x_lst = []
    gyro_y_lst = []
    gyro_z_lst = []
    for frame_index, frame in enumerate(lf.point_data_block):
        # timestamp = 0
        points = []
        for package in frame.packages:
            package: Package
            # if not timestamp:
            for point in package.points:
                if package.data_type == data_type:
                    points.append(point)
                    # print("frame = ", frame_index, "timestamp = ", timestamp, [str(getattr(point, attr)) for attr in attributes])
                    # print(getattr(point, 'acc_x'))
                    # print(type(getattr(point, 'acc_x')))
                    gyro_x_lst.append(point.gyro_x)
                    gyro_y_lst.append(point.gyro_y)
                    gyro_z_lst.append(point.gyro_z)
                    acc_x_lst.append(point.acc_x)
                    acc_y_lst.append(point.acc_y)
                    acc_z_lst.append(point.acc_z)

    address_acc_x = outdir + "/acc_x.pkl"
    address_acc_y = outdir + "/acc_y.pkl"
    address_acc_z = outdir + "/acc_z.pkl"
    address_gyro_x = outdir + "/gyro_x.pkl"
    address_gyro_y = outdir + "/gyro_y.pkl"
    address_gyro_z = outdir + "/gyro_z.pkl"

    with open(address_acc_x, "wb") as f:
        pickle.dump(np.array(acc_x_lst), f)
    with open(address_acc_y, "wb") as f:
        pickle.dump(np.array(acc_y_lst), f)
    with open(address_acc_z, "wb") as f:
        pickle.dump(np.array(acc_z_lst), f)
    with open(address_gyro_x, "wb") as f:
        pickle.dump(np.array(gyro_x_lst), f)
    with open(address_gyro_y, "wb") as f:
        pickle.dump(np.array(gyro_y_lst), f)
    with open(address_gyro_z, "wb") as f:
        pickle.dump(np.array(gyro_z_lst), f)


def topcds(lvxfile, outdir):
    if not os.path.exists(outdir):
        os.makedirs(outdir)

    lf = LvxFile(lvxfile)
    print(lf.private_header_block.frame_duration)

    index = 0
    for frame in lf.point_data_block:
        timestamp = 0
        data_type = None
        points = []
        for package in frame.packages:
            package: Package
            if not timestamp:
                timestamp = package.timestamp
            if data_type is None and package.data_type != DataType.IMU_INFO:
                data_type = package.data_type
            for point in package.points:
                if package.data_type == data_type:
                    points.append(point)
        f = open(os.path.join(outdir, '{}.pcd'.format(datetime.fromtimestamp(timestamp/
                                                10 ** 9).strftime('%Y%m%d%H%M%S%f'))), 'w')

        f.write('VERSION 0.7\n')
        if data_type == DataType.CARTESIAN_MID:
            field_line = "FIELDS x y z reflectivity"
            type_line = "TYPE F F F U"
            size_line = "SIZE 4 4 4 1"
            count_line = "COUNT 1 1 1 1"
        elif data_type == DataType.SPHERICAL_MID:
            field_line = "FIELDS theta phi depth reflectivity"
            type_line = "TYPE U U F U"
            size_line = "SIZE 2 2 4 1"
            count_line = "COUNT 1 1 1 1"
        elif data_type == DataType.CARTESIAN_SINGLE:
            field_line = "FIELDS x y z reflectivity tag"
            type_line = "TYPE F F F U U"
            size_line = "SIZE 4 4 4 1 1"
            count_line = "COUNT 1 1 1 1 1"
        elif data_type == DataType.SPHERAICAL_SINGLE:
            field_line = "FIELDS theta phi depth reflectivity tag"
            type_line = "TYPE U U F U U"
            size_line = "SIZE 2 2 4 1 1"
            count_line = "COUNT 1 1 1 1 1"
        elif data_type == DataType.CARTESIAN_DOUBLE:
            field_line = "FIELDS x1 y1 z1 reflectivity1 tag1 x2 y2 z2 reflectivity2 tag2"
            type_line = "TYPE F F F U U F F F U U"
            size_line = "SIZE 4 4 4 1 1 4 4 4 1 1"
            count_line = "COUNT 1 1 1 1 1 1 1 1 1 1"
        elif data_type == DataType.SPHERAICAL_DOUBLE:
            field_line = "FIELDS theta phi depth1 reflectivity1 tag1 depth2 reflectivity2 tag2"
            type_line = "TYPE U U F U U F U U"
            size_line = "SIZE 2 2 4 1 1 4 1 1"
            count_line = "COUNT 1 1 1 1 1 1 1 1"
        else:
            raise

        f.write(field_line + '\n')
        f.write(size_line + '\n')
        f.write(type_line + '\n')
        f.write(count_line + '\n')
        f.write('WIDTH {}\n'.format(len(points)))
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write('POINTS {}\n'.format(len(points)))
        f.write('DATA ascii\n')

        for p in points:
            fields = field_line.split(' ')[1:]
            values = [str(getattr(p, field)) for field in fields]
            f.write(' '.join(values) + '\n')
        f.close()
        index += 1


if __name__ == "__main__":
    get_imu("long.lvx", "./output")

