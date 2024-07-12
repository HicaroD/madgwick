import ctypes
import numpy as np
import pyinstrument

Madgwick = ctypes.CDLL(name="./madgwick.so")


class Madgwick_CStruct(ctypes.Structure):
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


Madgwick.madgwick_create.restype = ctypes.POINTER(Madgwick_CStruct)
Madgwick.madgwick_create.argtypes = [
    ctypes.c_float,  # float freq
    ctypes.c_float,  # float gain
]

Madgwick.madgwick_filter.restype = ctypes.POINTER(ctypes.c_float)
Madgwick.madgwick_filter.argtypes = [
    ctypes.POINTER(Madgwick_CStruct),  # struct madgwick *filter
    ctypes.POINTER(ctypes.c_float),  # float* acc
    ctypes.POINTER(ctypes.c_float),  # float* gyro
    ctypes.POINTER(ctypes.c_float),  # float* mag
    ctypes.c_size_t,  # size_t rows
]

Madgwick.madgwick_destroy.restype = ctypes.c_bool
Madgwick.madgwick_destroy.argtypes = [
    ctypes.POINTER(ctypes.POINTER(Madgwick_CStruct)),  # struct madgwick **filter
]


def from_numpy_matrix_to_float_p(matrix: np.ndarray):
    arr = np.ascontiguousarray(matrix, dtype=np.float32)
    if arr.flags["C_CONTIGUOUS"]:
        print("matrix IS C-contiguous, which means NO COPY IS BEING DONE")
    else:
        print("matrix IS NOT C-contiguous, which means A COPY IS BEING DONE")
    print()
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float))


def from_ctypes_float_p_to_numpy_matrix(matrix_ptr, rows, cols):
    # memory is shared here, no copy should be done!!
    return np.ctypeslib.as_array(matrix_ptr, shape=(rows, cols))


def generate_random_matrix(n, low=0.0, high=1.0):
    return np.random.uniform(low, high, size=(n, 3))


def main():
    frequency = 4.0
    gain = 0.1

    filter = Madgwick.madgwick_create(frequency, gain)
    if not filter:
        print("error: filter is NULL")
        return

    n = 40_000_000
    # n = 10

    acc = generate_random_matrix(n, high=10.0)
    gyro = generate_random_matrix(n, high=10.0)
    mag = generate_random_matrix(n, high=10.0)

    profiler = pyinstrument.Profiler()
    with profiler:
        p_acc = from_numpy_matrix_to_float_p(acc)
        p_gyro = from_numpy_matrix_to_float_p(gyro)
        p_mag = from_numpy_matrix_to_float_p(mag)

        raw_buffer_data = Madgwick.madgwick_filter(
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
    okay = Madgwick.madgwick_destroy(filter)
    if not okay:
        print("error: unable to clean up memory from filter")
        return
    print("Done")

    profiler.open_in_browser(timeline=True)


if __name__ == "__main__":
    main()
