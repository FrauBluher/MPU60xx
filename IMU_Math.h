#ifndef IMU_MATH_H
#define IMU_MATH_H

// Returns the Euler angles in radians defined with the Aerospace sequence.
// See Sebastian O.H. Madwick report
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
/**
 * Converts a quaternion representation to Euler angles.
 *
 * @param q The quaternion.
 * @param angles Radians as defined with the Aerospace sequence.
 */
void QuaternionToEuler(const float q[4], float angles[3]);

/**
 * Converts from yaw-pitch-roll angles to a quaternion using the standard
 * aerospace/Tait-Bryan angles.
 * 
 * @see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * 
 * @param ypr Yaw/pitch/roll in radians
 * @param q[out] The output quaternion in [w x y z] order.
 */
void YawPitchRollToQuaternion(const float ypr[3], float q[4]);

/**
 * Converts a quaternion to yaw-pitch-roll values.
 *
 * @param q The quaternion to convert.
 * @param ypr A 3-element array where the yaw-pitch-roll values will be written to.
 */
void QuaternionToYawPitchRoll(const float q[4], float ypr[3]);

/**
 * Converts a quaternion into a direction cosine matrix.
 *
 * @param q A quaternion in [w x y z] order (w is the angle)
 * @param dcm[out] A 3x3 array to store the DCM (row-major order).
 */
void QuaternionToDCM(const float q[4], float dcm[3][3]);

/**
 * Stringifies a quaternion into the output format (sprintf-syntax) "%04X,%04X,%04X,%04X,\n".
 * And note that the numbers are output in little-endian format. This is used for integration
 * with the visualization scripts provided.
 * @param q The input quaternion in [w x y z] format.
 * @param out[out] The output character array the 37 bytes will be written to. No NULL-
 *                 terminating character will be added.
 */
void QuaternionToString(const float q[4], char out[37]);

/**
 Function
    MovingAvg8
 Parameters
    NewValue int,the new vale to enter into the moving average
 Returns
    int, the value of the moving average after entering the NewValue
 Description
    Implements an 8-point moving average using an 8-entry buffer and an
 * alogithm that kees a sum ans subracts the oldest value from the sum,
 * followed by adding the new value before diving by 8.
 * A very literal implementation.
 Notes
    While the buffer is initially filling, it is hard to really call the
 * average accurate, since we force the initals values to 0
 Author
    J. Edward Carryer, 12/06/09 15:09
 */
int MoveAvg8(int NewValue, int bufferNum);

#endif // IMU_MATH_H