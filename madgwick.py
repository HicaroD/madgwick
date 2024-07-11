import ctypes
import numpy as np


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


MADGWICK = ctypes.CDLL(name="./madgwick.so")

MADGWICK.madgwick_create.restype = ctypes.POINTER(Madgwick)
MADGWICK.madgwick_create.argtypes = [ctypes.c_float, ctypes.c_float]

MADGWICK.madgwick_filter.restype = ctypes.POINTER(ctypes.POINTER(ctypes.c_float))
MADGWICK.madgwick_filter.argtypes = [
    ctypes.POINTER(Madgwick),  # *struct madgwick *filter
    ctypes.POINTER(ctypes.POINTER(ctypes.c_float)),  # float** acc
    ctypes.POINTER(ctypes.POINTER(ctypes.c_float)),  # float** gyro
    ctypes.POINTER(ctypes.POINTER(ctypes.c_float)),  # float** mag
    ctypes.c_size_t,  # size_t size
]

MADGWICK.madgwick_destroy.restype = ctypes.c_bool
MADGWICK.madgwick_destroy.argtypes = [ctypes.POINTER(ctypes.POINTER(Madgwick))]


def numpy_matrix_to_ctypes(matrix: np.ndarray):
    FloatArrayType = ctypes.c_float * len(matrix[0])
    float_arrays = [FloatArrayType(*row) for row in matrix]
    rows = [ctypes.cast(row, ctypes.POINTER(ctypes.c_float)) for row in float_arrays]
    float_pointers = (ctypes.POINTER(ctypes.c_float) * len(matrix))(*rows)
    return ctypes.cast(float_pointers, ctypes.POINTER(ctypes.POINTER(ctypes.c_float)))


def main():
    frequency = 4.0
    gain = 0.1

    filter = MADGWICK.madgwick_create(frequency, gain)
    if not filter:
        print("error: filter is NULL")
        return

    acc = numpy_matrix_to_ctypes(np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]))
    gyro = numpy_matrix_to_ctypes(np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]))
    mag = numpy_matrix_to_ctypes(np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]))

    okay = MADGWICK.madgwick_filter(filter, acc, gyro, mag, 2)
    if okay is None:
        print("error: unable to filter data")
        return

    okay = MADGWICK.madgwick_destroy(filter)
    if not okay:
        print("error: unable to clean up memory from filter")
        return

    print("Done")


if __name__ == "__main__":
    main()
