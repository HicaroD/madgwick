import ctypes

import numpy as np


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


class TypesConverter:
    @staticmethod
    def from_numpy_matrix_to_float_p(matrix: np.ndarray):
        arr = np.ascontiguousarray(matrix, dtype=np.float32)
        if arr.flags["C_CONTIGUOUS"]:
            print("matrix IS C-contiguous, which means NO COPY IS BEING DONE")
        else:
            print("matrix IS NOT C-contiguous, which means A COPY IS BEING DONE")
        return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

    @staticmethod
    def from_numpy_arr_to_float_p(arr: np.ndarray):
        return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

    @staticmethod
    def from_ctypes_float_p_to_numpy_matrix(matrix_ptr, rows: int, cols: int):
        # memory is shared here, no copy should be done with this operation!!
        if matrix_ptr is None:
            raise ValueError("matrix ptr is NULL")
        arr = np.ctypeslib.as_array(matrix_ptr, shape=(rows, cols))
        return arr


class MadgwickC:
    def __init__(
        self,
        frequency: float,
        gain: float = 0.1,
    ) -> None:
        self._lib = self._setup_madgwick_lib()
        self._filter = self._lib.madgwick_create(frequency, gain)
        if not filter:
            raise ValueError("Cannot create Madgwick filter object")

    def filter(
        self,
        acc: np.ndarray,
        gyro: np.ndarray,
        mag: np.ndarray | None = None,
    ) -> np.ndarray:
        p_acc = TypesConverter.from_numpy_matrix_to_float_p(acc)
        p_gyro = TypesConverter.from_numpy_matrix_to_float_p(gyro)
        p_mag = (
            TypesConverter.from_numpy_matrix_to_float_p(mag)
            if mag is not None
            else None
        )

        rows = ctypes.c_size_t(len(acc))

        raw_buffer_data = self._lib.madgwick_filter(
            self._filter,
            p_acc,
            p_gyro,
            p_mag,
            rows,
        )

        if raw_buffer_data is None:
            raise ValueError("unable to filter data")

        filtered_data = TypesConverter.from_ctypes_float_p_to_numpy_matrix(
            raw_buffer_data,
            len(acc),
            len(acc[0]),
        )

        return filtered_data

    def clean(self):
        okay = self._lib.madgwick_destroy(self._filter)
        if not okay:
            raise ValueError("unable to clean up memory from filter")

    def _setup_madgwick_lib(self) -> ctypes.CDLL:
        madgwick_lib = ctypes.CDLL(name="./madgwick.so")

        madgwick_lib.madgwick_create.restype = ctypes.POINTER(Madgwick_CStruct)
        madgwick_lib.madgwick_create.argtypes = [
            ctypes.c_float,  # float freq
            ctypes.c_float,  # float gain
        ]

        madgwick_lib.madgwick_filter.restype = ctypes.POINTER(ctypes.c_float)
        madgwick_lib.madgwick_filter.argtypes = [
            ctypes.POINTER(Madgwick_CStruct),  # struct madgwick *filter
            ctypes.POINTER(ctypes.c_float),  # float* acc
            ctypes.POINTER(ctypes.c_float),  # float* gyro
            ctypes.POINTER(ctypes.c_float),  # float* mag
            ctypes.c_size_t,  # size_t rows
        ]

        madgwick_lib.madgwick_destroy.restype = ctypes.c_bool
        madgwick_lib.madgwick_destroy.argtypes = [
            ctypes.POINTER(
                ctypes.POINTER(Madgwick_CStruct)
            ),  # struct madgwick **filter
        ]

        return madgwick_lib
