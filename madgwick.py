import ctypes
import numpy as np
from time import time


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


def from_numpy_matrix_to_float_pp_ctypes(matrix: np.ndarray):
    FloatArrayType = ctypes.c_float * len(matrix[0])
    float_arrays = [FloatArrayType(*row) for row in matrix]
    rows = [ctypes.cast(row, ctypes.POINTER(ctypes.c_float)) for row in float_arrays]
    float_pointers = (ctypes.POINTER(ctypes.c_float) * len(matrix))(*rows)
    return ctypes.cast(float_pointers, ctypes.POINTER(ctypes.POINTER(ctypes.c_float)))


def from_ctypes_float_pp_to_numpy_matrix(matrix_ptr, rows, cols):
    array = np.empty((rows, cols), dtype=np.float32)
    for row in range(rows):
        row_ptr = matrix_ptr[row]
        array[row, :] = np.ctypeslib.as_array(row_ptr, shape=(cols,))
    return array


def generate_random_matrix(n, low=0.0, high=1.0):
    return np.random.uniform(low, high, size=(n, 3))


def main():
    frequency = 4.0
    gain = 0.1

    filter = MADGWICK.madgwick_create(frequency, gain)
    if not filter:
        print("error: filter is NULL")
        return

    n = 2_000_000

    acc = generate_random_matrix(n, high=10.0)
    gyro = generate_random_matrix(n, high=10.0)
    mag = generate_random_matrix(n, high=10.0)

    p_acc = from_numpy_matrix_to_float_pp_ctypes(acc)
    p_gyro = from_numpy_matrix_to_float_pp_ctypes(gyro)
    p_mag = from_numpy_matrix_to_float_pp_ctypes(mag)

    start = time()

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

    filtered_data = from_ctypes_float_pp_to_numpy_matrix(
        raw_buffer_data,
        len(acc),
        len(acc[0]),
    )

    end = time()
    print("Time taken: ", end - start)

    print(filtered_data)

    okay = MADGWICK.madgwick_destroy(filter)
    if not okay:
        print("error: unable to clean up memory from filter")
        return

    print("Done")


if __name__ == "__main__":
    main()
