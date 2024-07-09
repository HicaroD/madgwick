//=============================================================================================
// madgwick.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 12/11/2018	Pim van Pelt    Rewritten to C for Mongoose OS
//
//=============================================================================================
#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* Madgwick filter structure. */
struct madgwick {
  float gain;
  float q0;
  float q1;
  float q2;
  float q3;
  float freq;
};

/* Create a new filter and initialize it by resetting the Quaternion and setting
 * the update rate to rate variable and the gain to gain variable
 * Returns a pointer to a `struct madgwick`, or NULL otherwise.
 */
struct madgwick *madgwick_create(float rate, float gain);

/* Clean up and return memory for the filter
 */
bool madgwick_destroy(struct madgwick **filter);

/* Resets the filter Quaternion to an initial state (of {1,0,0,0}).
 */
bool madgwick_reset(struct madgwick *filter);

/* Run an update cycle on the filter. Inputs gx/gy/gz are in any calibrated
 * input (for example, m/s/s or G), inputs of ax/ay/az are in Rads/sec, inputs
 * of mx/my/mz are in any calibrated input (for example, uTesla or Gauss). The
 * inputs of mx/my/mz can be passed as 0.0, in which case the magnetometer
 * fusion will not occur. Returns true on success, false on failure.
 */
bool madgwick_update(struct madgwick *filter, float gx, float gy, float gz,
                         float ax, float ay, float az, float mx, float my,
                         float mz);

/*
 * Returns AHRS Quaternion, as values between -1.0 and +1.0.
 * Each of q0, q1, q2, q3 pointers may be NULL, in which case they will not be
 * filled in.
 * Returns true on success, false in case of error, in which case the values of
 * q0, q1, q2 and q3 are undetermined.
 */
bool madgwick_get_quaternion(struct madgwick *filter, float *q0, float *q1,
                                 float *q2, float *q3);

/*
 * Returns AHRS angles of roll, pitch and yaw, in Radians between -Pi and +Pi.
 * Each of the roll, pitch and yaw pointers may be NULL, in which case they will
 * not be filled in.
 * Returns true on success, false in case of error, in which case the values of
 * roll, pitch and yaw are undetermined.
 */
bool madgwick_get_angles(struct madgwick *filter, float *roll, float *pitch,
                             float *yaw);
