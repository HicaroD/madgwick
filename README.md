# Madgwick

Implementation of the Madgwick algorithm in C, with Python serving as the
caller interface via [ctypes](https://docs.python.org/3/library/ctypes.html).

## Algorithm

The algorithm itself was not implemented by me, the hard work was done by
people from the [Mongoose OS](https://github.com/mongoose-os-libs/imu/tree/master). 

In order to make things work and access the core implementation of the
algorithm via `ctypes`, I've made some changes, removed some unnecessary
attributes and methods in order to make the code even more efficient.
