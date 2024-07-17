#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define RAD2DEG 57.29577951308232
#define COLUMNS 3
#define INDEX(i, j) (i * COLUMNS + j)

/* Madgwick filter structure. */
struct madgwick {
  float gain;
  float rate;

  float q0;
  float q1;
  float q2;
  float q3;

  float roll;
  float pitch;
  float yaw;
};

/* Create a new filter and initialize it by resetting the Quaternion and setting
 * the rate and gain variables.
 * Returns a pointer to a `struct madgwick`, or NULL in case of error.
 */
struct madgwick *madgwick_create(float freq, float gain);

/* Clean up memory from allocated filter.
 */
bool madgwick_destroy(struct madgwick **filter);

/* Filter the received information from accelerometer, gyroscope and
 * magnetometer using the Madgwick algorithm.
 * Returns a Nx3 float matrix, or NULL in case of error.
 */
float *madgwick_filter(struct madgwick *filter, float *acc, float *gyro,
                       float *mag, size_t rows);

