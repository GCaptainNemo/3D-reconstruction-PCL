def _floatfrombytes(bs):
    hs = ''.join(['%02X' % x for x in bs])
    return float.fromhex(hs)


class DataType:
    CARTESIAN_MID = 0
    SPHERICAL_MID = 1
    CARTESIAN_SINGLE = 2
    SPHERAICAL_SINGLE = 3
    CARTESIAN_DOUBLE = 4
    SPHERAICAL_DOUBLE = 5
    IMU_INFO = 6


class Point0:
    def __init__(self, bs):
        self.bs = bs

    @property
    def x(self):
        return int.from_bytes(self.bs[:4], 'little', signed=True) / 1000

    @property
    def y(self):
        return int.from_bytes(self.bs[4:8], 'little', signed=True) / 1000

    @property
    def z(self):
        return int.from_bytes(self.bs[8:12], 'little', signed=True) / 1000

    @property
    def reflectivity(self):
        return int.from_bytes(self.bs[12:13], 'little')


class Point1:
    def __init__(self, bs):
        self.bs = bs

    @property
    def depth(self):
        return int.from_bytes(self.bs[:4], 'little', signed=True) / 1000

    @property
    def theta(self):
        return int.from_bytes(self.bs[4:6], 'little')

    @property
    def phi(self):
        return int.from_bytes(self.bs[6:8], 'little')

    @property
    def reflectivity(self):
        return int.from_bytes(self.bs[8:9], 'little')


class Point2:
    def __init__(self, bs):
        self.bs = bs

    @property
    def x(self):
        return int.from_bytes(self.bs[:4], 'little', signed=True) / 1000

    @property
    def y(self):
        return int.from_bytes(self.bs[4:8], 'little', signed=True) / 1000

    @property
    def z(self):
        return int.from_bytes(self.bs[8:12], 'little', signed=True) / 1000

    @property
    def reflectivity(self):
        return int.from_bytes(self.bs[12:13], 'little')

    @property
    def tag(self):
        return int.from_bytes(self.bs[13:14], 'little')


class Point3:
    def __init__(self, bs):
        self.bs = bs

    @property
    def depth(self):
        return int.from_bytes(self.bs[:4], 'little', signed=True) / 1000

    @property
    def theta(self):
        return int.from_bytes(self.bs[4:6], 'little')

    @property
    def phi(self):
        return int.from_bytes(self.bs[6:8], 'little')

    @property
    def reflectivity(self):
        return int.from_bytes(self.bs[8:9], 'little')

    @property
    def tag(self):
        return int.from_bytes(self.bs[9:10], 'little')


class Point4:
    def __init__(self, bs):
        self.bs = bs

    @property
    def x1(self):
        return int.from_bytes(self.bs[:4], 'little', signed=True) / 1000

    @property
    def y1(self):
        return int.from_bytes(self.bs[:8], 'little', signed=True) / 1000

    @property
    def z1(self):
        return int.from_bytes(self.bs[:12], 'little', signed=True) / 1000

    @property
    def reflectivity1(self):
        return int.from_bytes(self.bs[:13], 'little')

    @property
    def tag1(self):
        return int.from_bytes(self.bs[:14], 'little')

    @property
    def x2(self):
        return int.from_bytes(self.bs[:18], 'little', signed=True) / 1000

    @property
    def y2(self):
        return int.from_bytes(self.bs[:22], 'little', signed=True) / 1000

    @property
    def z2(self):
        return int.from_bytes(self.bs[:26], 'little', signed=True) / 1000

    @property
    def reflectivity2(self):
        return int.from_bytes(self.bs[:27], 'little')

    @property
    def tag2(self):
        return int.from_bytes(self.bs[:28], 'little')


class Point5:
    def __init__(self, bs):
        self.bs = bs

    @property
    def theta(self):
        return int.from_bytes(self.bs[:2], 'little')

    @property
    def phi(self):
        return int.from_bytes(self.bs[2:4], 'little')

    @property
    def depth1(self):
        return int.from_bytes(self.bs[4:8], 'little', signed=True) / 1000

    @property
    def reflectivity1(self):
        return int.from_bytes(self.bs[8:9], 'little')

    @property
    def tag1(self):
        return int.from_bytes(self.bs[9:10], 'little')

    @property
    def depth2(self):
        return int.from_bytes(self.bs[10:14], 'little', signed=True) / 1000

    @property
    def reflectivity2(self):
        return int.from_bytes(self.bs[14:15], 'little')

    @property
    def tag2(self):
        return int.from_bytes(self.bs[15:16], 'little')


class Point6:
    def __init__(self, bs):
        self.bs = bs

    @property
    def gyro_x(self):
        return _floatfrombytes(self.bs[:4])

    @property
    def gyro_y(self):
        return _floatfrombytes(self.bs[4:8])

    @property
    def gyro_z(self):
        return _floatfrombytes(self.bs[8:12])

    @property
    def acc_x(self):
        return _floatfrombytes(self.bs[12:16])

    @property
    def acc_y(self):
        return _floatfrombytes(self.bs[16:20])

    @property
    def acc_z(self):
        return _floatfrombytes(self.bs[20:24])


class Package:
    def __init__(self, bs):
        self.bs = bs

    @property
    def device_index(self):
        return int.from_bytes(self.bs[:1], 'little')

    @property
    def version(self):
        return int.from_bytes(self.bs[1:2], 'little')

    @property
    def slot_id(self):
        return int.from_bytes(self.bs[2:3], 'little')

    @property
    def lidar_id(self):
        return int.from_bytes(self.bs[3:4], 'little')

    @property
    def reserved(self):
        return int.from_bytes(self.bs[4:5], 'little')

    @property
    def status_code(self):
        return int.from_bytes(self.bs[5:9], 'little')

    @property
    def timestamp_type(self):
        return int.from_bytes(self.bs[9:10], 'little')

    @property
    def data_type(self):
        return int.from_bytes(self.bs[10:11], 'little')

    @property
    def timestamp(self):
        return int.from_bytes(self.bs[11:19], 'little')

    @property
    def points(self):
        if self.data_type == DataType.CARTESIAN_MID:
            point_size = 13
            point_count = 100
            point_class = Point0
        elif self.data_type == DataType.SPHERICAL_MID:
            point_size = 9
            point_count = 100
            point_class = Point1
        elif self.data_type == DataType.CARTESIAN_SINGLE:
            point_size = 14
            point_count = 96
            point_class = Point2
        elif self.data_type == DataType.SPHERAICAL_SINGLE:
            point_size = 10
            point_count = 96
            point_class = Point3
        elif self.data_type == DataType.CARTESIAN_DOUBLE:
            point_size = 28
            point_count = 48
            point_class = Point4
        elif self.data_type == DataType.SPHERAICAL_DOUBLE:
            point_size = 16
            point_count = 48
            point_class = Point5
        elif self.data_type == DataType.IMU_INFO:
            point_size = 24
            point_count = 1
            point_class = Point6
        else:
            raise Exception
        return [point_class(self.bs[19 + i * point_size: 19 + point_size * (i + 1)]) for i in range(point_count)]


class FrameHeader:
    def __init__(self, bs):
        self.bs = bs

    @property
    def current_offset(self):
        return int.from_bytes(self.bs[:8], 'little')

    @property
    def next_offset(self):
        return int.from_bytes(self.bs[8:16], 'little')

    @property
    def frame_index(self):
        return int.from_bytes(self.bs[16:24], 'little')


class Frame:
    def __init__(self, bs):
        self.bs = bs

    @property
    def frame_header(self):
        return FrameHeader(self.bs[:24])

    @property
    def packages(self):
        current_offset = 24
        while current_offset < len(self.bs):
            pakcage_header = Package(self.bs[current_offset:current_offset + 19])
            if pakcage_header.data_type == DataType.CARTESIAN_MID:
                point_size = 13
                point_count = 100
            elif pakcage_header.data_type == DataType.SPHERICAL_MID:
                point_size = 9
                point_count = 100
            elif pakcage_header.data_type == DataType.CARTESIAN_SINGLE:
                point_size = 14
                point_count = 96
            elif pakcage_header.data_type == DataType.SPHERAICAL_SINGLE:
                point_size = 10
                point_count = 96
            elif pakcage_header.data_type == DataType.CARTESIAN_DOUBLE:
                point_size = 28
                point_count = 48
            elif pakcage_header.data_type == DataType.SPHERAICAL_DOUBLE:
                point_size = 16
                point_count = 48
            elif pakcage_header.data_type == DataType.IMU_INFO:
                point_size = 24
                point_count = 1
            else:
                raise Exception(pakcage_header.data_type)
            yield Package(self.bs[current_offset:current_offset + 19 + point_size * point_count])
            current_offset += 19 + point_size * point_count
