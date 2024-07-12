//
// NOTE: The original code was modified by Hícaro. A copy of the original
// license is located at FILTER_LICENSE file.
//

#include "cmadgwick.h"

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

static float inverse_square_root(float x) {
  union {
    float f;
    uint32_t i;
  } conv;

  float x2;
  const float threehalfs = 1.5F;

  x2 = x * 0.5F;
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  conv.f = conv.f * (threehalfs - (x2 * conv.f * conv.f));
  return conv.f;
}

/* Resets the filter Quaternion to an initial state (of {1,0,0,0}).
 */
static bool madgwick_reset(struct madgwick *filter) {
  if (!filter) {
    return false;
  }

  filter->q0 = 1.0f;
  filter->q1 = 0.0f;
  filter->q2 = 0.0f;
  filter->q3 = 0.0f;

  filter->roll = 0;
  filter->pitch = 0;
  filter->yaw = 0;

  return true;
}

struct madgwick *madgwick_create(float freq, float gain) {
  struct madgwick *filter;

  filter = (struct madgwick *)malloc(sizeof(struct madgwick));
  if (!filter) {
    return NULL;
  }

  filter->rate = 1.0f / freq;
  filter->gain = gain;

  madgwick_reset(filter);
  return filter;
}

bool madgwick_destroy(struct madgwick **filter) {
  if (!*filter) {
    return false;
  }
  free(*filter);
  *filter = NULL;
  return true;
}

static bool madgwick_updateIMU(struct madgwick *filter, float ax, float ay,
                               float az, float gx, float gy, float gz) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
      q3q3;

  // No need to check filter pointer -- it's checked by _update()

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-filter->q1 * gx - filter->q2 * gy - filter->q3 * gz);
  qDot2 = 0.5f * (filter->q0 * gx + filter->q2 * gz - filter->q3 * gy);
  qDot3 = 0.5f * (filter->q0 * gy - filter->q1 * gz + filter->q3 * gx);
  qDot4 = 0.5f * (filter->q0 * gz + filter->q1 * gy - filter->q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = inverse_square_root(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * filter->q0;
    _2q1 = 2.0f * filter->q1;
    _2q2 = 2.0f * filter->q2;
    _2q3 = 2.0f * filter->q3;
    _4q0 = 4.0f * filter->q0;
    _4q1 = 4.0f * filter->q1;
    _4q2 = 4.0f * filter->q2;
    _8q1 = 8.0f * filter->q1;
    _8q2 = 8.0f * filter->q2;
    q0q0 = filter->q0 * filter->q0;
    q1q1 = filter->q1 * filter->q1;
    q2q2 = filter->q2 * filter->q2;
    q3q3 = filter->q3 * filter->q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * filter->q1 - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * filter->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * filter->q3 - _2q1 * ax + 4.0f * q2q2 * filter->q3 -
         _2q2 * ay;
    recipNorm = inverse_square_root(s0 * s0 + s1 * s1 + s2 * s2 +
                                    s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= filter->gain * s0;
    qDot2 -= filter->gain * s1;
    qDot3 -= filter->gain * s2;
    qDot4 -= filter->gain * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  filter->q0 += qDot1 * filter->rate;
  filter->q1 += qDot2 * filter->rate;
  filter->q2 += qDot3 * filter->rate;
  filter->q3 += qDot4 * filter->rate;

  // Normalise quaternion
  recipNorm =
      inverse_square_root(filter->q0 * filter->q0 + filter->q1 * filter->q1 +
                          filter->q2 * filter->q2 + filter->q3 * filter->q3);
  filter->q0 *= recipNorm;
  filter->q1 *= recipNorm;
  filter->q2 *= recipNorm;
  filter->q3 *= recipNorm;

  return true;
}

/*
 * Set AHRS angles of roll, pitch and yaw, in degrees, from the resultant
 * quaternion. Returns true on success, false in case of error.
 */
static bool madgwick_set_angles(struct madgwick *filter) {
  if (!filter) {
    return false;
  }

  // TODO: make sure they are properly set
  filter->roll =
      asinf(-2.0f * (filter->q1 * filter->q3 - filter->q0 * filter->q2));
  filter->pitch =
      atan2f(filter->q0 * filter->q1 + filter->q2 * filter->q3,
             0.5f - filter->q1 * filter->q1 - filter->q2 * filter->q2);
  filter->yaw =
      atan2f(filter->q1 * filter->q2 + filter->q0 * filter->q3,
             0.5f - filter->q2 * filter->q2 - filter->q3 * filter->q3);

  filter->roll *= RAD2DEG;
  filter->pitch *= RAD2DEG;
  filter->yaw *= RAD2DEG;

  return true;
}

/* Run an update cycle on the filter. Inputs ax/ay/az are in any calibrated
 * input (for example, m/s/s or G), inputs of gx/gy/gz are in Rads/sec, inputs
 * of mx/my/mz are in any calibrated input (for example, uTesla or Gauss). The
 * inputs of mx/my/mz can be passed as 0.0, in which case the magnetometer
 * fusion will not occur. Returns true on success, false on failure.
 */
static bool madgwick_update(struct madgwick *filter, float ax, float ay,
                            float az, float gx, float gy, float gz, float mx,
                            float my, float mz) {
  if (!filter) {
    return false;
  }
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
      _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3,
      q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in
  // magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    bool okay = madgwick_updateIMU(filter, ax, ay, az, gx, gy, gz);
    return okay;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-filter->q1 * gx - filter->q2 * gy - filter->q3 * gz);
  qDot2 = 0.5f * (filter->q0 * gx + filter->q2 * gz - filter->q3 * gy);
  qDot3 = 0.5f * (filter->q0 * gy - filter->q1 * gz + filter->q3 * gx);
  qDot4 = 0.5f * (filter->q0 * gz + filter->q1 * gy - filter->q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = inverse_square_root(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = inverse_square_root(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * filter->q0 * mx;
    _2q0my = 2.0f * filter->q0 * my;
    _2q0mz = 2.0f * filter->q0 * mz;
    _2q1mx = 2.0f * filter->q1 * mx;
    _2q0 = 2.0f * filter->q0;
    _2q1 = 2.0f * filter->q1;
    _2q2 = 2.0f * filter->q2;
    _2q3 = 2.0f * filter->q3;
    _2q0q2 = 2.0f * filter->q0 * filter->q2;
    _2q2q3 = 2.0f * filter->q2 * filter->q3;
    q0q0 = filter->q0 * filter->q0;
    q0q1 = filter->q0 * filter->q1;
    q0q2 = filter->q0 * filter->q2;
    q0q3 = filter->q0 * filter->q3;
    q1q1 = filter->q1 * filter->q1;
    q1q2 = filter->q1 * filter->q2;
    q1q3 = filter->q1 * filter->q3;
    q2q2 = filter->q2 * filter->q2;
    q2q3 = filter->q2 * filter->q3;
    q3q3 = filter->q3 * filter->q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * filter->q3 + _2q0mz * filter->q2 + mx * q1q1 +
         _2q1 * my * filter->q2 + _2q1 * mz * filter->q3 - mx * q2q2 -
         mx * q3q3;
    hy = _2q0mx * filter->q3 + my * q0q0 - _2q0mz * filter->q1 +
         _2q1mx * filter->q2 - my * q1q1 + my * q2q2 + _2q2 * mz * filter->q3 -
         my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * filter->q2 + _2q0my * filter->q1 + mz * q0q0 +
           _2q1mx * filter->q3 - mz * q1q1 + _2q2 * my * filter->q3 -
           mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
         _2bz * filter->q2 *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * filter->q3 + _2bz * filter->q1) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * filter->q2 *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * filter->q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         _2bz * filter->q3 *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * filter->q2 + _2bz * filter->q0) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * filter->q3 - _4bz * filter->q1) *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * filter->q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         (-_4bx * filter->q2 - _2bz * filter->q0) *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * filter->q1 + _2bz * filter->q3) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * filter->q0 - _4bz * filter->q2) *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
         (-_4bx * filter->q3 + _2bz * filter->q1) *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * filter->q0 + _2bz * filter->q2) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * filter->q1 *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = inverse_square_root(s0 * s0 + s1 * s1 + s2 * s2 +
                                    s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= filter->gain * s0;
    qDot2 -= filter->gain * s1;
    qDot3 -= filter->gain * s2;
    qDot4 -= filter->gain * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  filter->q0 += qDot1 * filter->rate;
  filter->q1 += qDot2 * filter->rate;
  filter->q2 += qDot3 * filter->rate;
  filter->q3 += qDot4 * filter->rate;

  // Normalise quaternion
  recipNorm =
      inverse_square_root(filter->q0 * filter->q0 + filter->q1 * filter->q1 +
                          filter->q2 * filter->q2 + filter->q3 * filter->q3);
  filter->q0 *= recipNorm;
  filter->q1 *= recipNorm;
  filter->q2 *= recipNorm;
  filter->q3 *= recipNorm;

  return madgwick_set_angles(filter);
}

float *madgwick_filter(struct madgwick *filter, float *acc, float *gyro,
                       float *mag, size_t rows) {
  float *matrix = malloc(rows * COLUMNS * sizeof(float));
  // TODO: improve error handling, maybe store a string in the struct and set
  // the message in case of error
  if (!matrix) {
    fprintf(stderr,
            "unable to allocate matrix for storing filtered data in memory\n");
    return NULL;
  }

  for (size_t i = 0; i < rows; ++i) {
    int x = INDEX(i, 0);
    int y = INDEX(i, 1);
    int z = INDEX(i, 2);

    float ax = acc[x];
    float ay = acc[y];
    float az = acc[z];

    float gx = gyro[x];
    float gy = gyro[y];
    float gz = gyro[z];

    float mx = mag != NULL ? mag[x] : 0;
    float my = mag != NULL ? mag[y] : 0;
    float mz = mag != NULL ? mag[z] : 0;

    bool okay = madgwick_update(filter, ax, ay, az, gx, gy, gz, mx, my, mz);
    if (!okay) {
      fprintf(stderr, "unable to filter data\n");
      return NULL;
    }

    matrix[x] = filter->roll;
    matrix[y] = filter->pitch;
    matrix[z] = filter->yaw;
  }

  return matrix;
}
