import ctypes
import random

MADGWICK = ctypes.CDLL(name="./madgwick.so")


class Madgwick(ctypes.Structure):
    _fields_ = [
        ("gain", ctypes.c_float),
        ("rate", ctypes.c_float),
        ("q0", ctypes.c_float),
        ("q1", ctypes.c_float),
        ("q2", ctypes.c_float),
        ("q3", ctypes.c_float),
        ("roll", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("yaw", ctypes.c_float),
    ]


MADGWICK.madgwick_create.restype = ctypes.POINTER(Madgwick)
MADGWICK.madgwick_create.argtypes = [ctypes.c_float, ctypes.c_float]

MADGWICK.madgwick_update.restype = ctypes.c_bool
MADGWICK.madgwick_update.argtypes = [
    ctypes.POINTER(Madgwick),
    ctypes.c_float,  # ax
    ctypes.c_float,  # ay
    ctypes.c_float,  # az
    ctypes.c_float,  # gx
    ctypes.c_float,  # gy
    ctypes.c_float,  # gz
    ctypes.c_float,  # mx
    ctypes.c_float,  # my
    ctypes.c_float,  # mz
]


def main():
    frequency = 4.0
    gain = 0.1

    filter = MADGWICK.madgwick_create(frequency, gain)
    if not filter:
        print("error: filter is NULL")
        return

    filtered_data = []

    n = 5_000_000
    for _ in range(n):
        mock_data = [random.random() for _ in range(9)]
        MADGWICK.madgwick_update(
            filter,
            mock_data[0],
            mock_data[1],
            mock_data[2],
            mock_data[3],
            mock_data[4],
            mock_data[5],
            mock_data[6],
            mock_data[7],
            mock_data[8],
        )
        filtered_data.append(
            (
                filter.contents.roll,
                filter.contents.pitch,
                filter.contents.yaw,
            )
        )

    for data in filtered_data:
        print(data)


main()
