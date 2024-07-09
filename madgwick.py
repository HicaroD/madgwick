import ctypes

MADGWICK = ctypes.CDLL(name="madgwick.so")


class Madgwick(ctypes.Structure):
    _fields_ = [
        ("beta", ctypes.c_float),
        ("q0", ctypes.c_float),
        ("q1", ctypes.c_float),
        ("q2", ctypes.c_float),
        ("q3", ctypes.c_float),
        ("freq", ctypes.c_float),
    ]


def main():
    filter = Madgwick()
    # TODO
