import ctypes
import numpy as np
import pyinstrument


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

MADGWICK.madgwick_filter.restype = ctypes.POINTER(ctypes.c_float)
MADGWICK.madgwick_filter.argtypes = [
    ctypes.POINTER(Madgwick),  # struct madgwick *filter
    ctypes.POINTER(ctypes.c_float),  # float* acc
    ctypes.POINTER(ctypes.c_float),  # float* gyro
    ctypes.POINTER(ctypes.c_float),  # float* mag
    ctypes.c_size_t,  # size_t rows
]

MADGWICK.madgwick_destroy.restype = ctypes.c_bool
MADGWICK.madgwick_destroy.argtypes = [ctypes.POINTER(ctypes.POINTER(Madgwick))]


def from_numpy_matrix_to_float_p(matrix: np.ndarray):
    arr = np.ascontiguousarray(matrix, dtype=np.float32)
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float))


def from_ctypes_float_p_to_numpy_matrix(matrix_ptr, rows, cols):
    # memory shared here, no copy should be done!!
    return np.ctypeslib.as_array(matrix_ptr, shape=(rows, cols))


def generate_random_matrix(n, low=0.0, high=1.0):
    return np.random.uniform(low, high, size=(n, 3))


def main():
    frequency = 4.0
    gain = 0.1

    filter = MADGWICK.madgwick_create(frequency, gain)
    if not filter:
        print("error: filter is NULL")
        return

    n = 10_000_000
    # n = 10

    acc = generate_random_matrix(n, high=10.0)
    gyro = generate_random_matrix(n, high=10.0)
    mag = generate_random_matrix(n, high=10.0)

    profiler = pyinstrument.Profiler()
    with profiler:
        p_acc = from_numpy_matrix_to_float_p(acc)
        p_gyro = from_numpy_matrix_to_float_p(gyro)
        p_mag = from_numpy_matrix_to_float_p(mag)

        raw_buffer_data = MADGWICK.madgwick_filter(
            filter,
            p_acc,
            p_gyro,
            p_mag,
            len(acc),
        )

        if raw_buffer_data is None:
            print("error: unable to filter data")
            return

        filtered_data = from_ctypes_float_p_to_numpy_matrix(
            raw_buffer_data,
            len(acc),
            len(acc[0]),
        )

    print(filtered_data)
    okay = MADGWICK.madgwick_destroy(filter)
    if not okay:
        print("error: unable to clean up memory from filter")
        return
    print("Done")

    profiler.open_in_browser(timeline=True)


if __name__ == "__main__":
    main()
