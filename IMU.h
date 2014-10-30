/* 
 * File:   IMU.h
 * Author: Jonathan
 *
 * Created on October 23, 2014, 11:27 AM
 *
 */

#ifndef IMU_H
#define	IMU_H

#include "I2CdsPIC.h"
#include "MPU60xx.h"
#include "MAG3110.h"
#include <math.h>

#include "MadgwickAHRS/MadgwickAHRS.h"

typedef struct {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float magX;
    float magY;
    float magZ;
} IMU_Data;

#define G_FORCE 9.80665
#define MOVING_AVG_BUFFER 32

#define M_PI 3.1415927f
#define DEG2RAD(d)   (((d)*M_PI)/180.0f)

/**
 * This function will setup the MPU60x0 and the MAG3110 as a single i2c device.
 * The MPU will be set as an i2c master to the MAG3110. This allows the host device
 * to get all 9 DoFs from a single burst read to the MPU.
 *
 * @param i2cFreq The frequency of the i2c line. Should always be 400000 at the moment
 * @param sysFreq This is the system clock of the host device
 */
void IMU_Init(uint32_t i2cFreq, uint32_t sysFreq);

/**
 * Sets up the MPU60x0 device to read data registers for the MAG3110 in a burst fasion (hopefully)
 */
void IMU_SetMPUMaster(void);

/**
 * This function uses burst reading on i2c to read all IMU data and then stores it into their respective structs.
 *
 * @param mpuData MPU6050_Data struct pointer
 * @param magData MAG3110_Data struct pointer
 */
void IMU_GetData(MPU6050_Data *mpuData, MAG3110_Data *magData);

/**
 * Normalizes the raw bit data into their respective units
 * Accel: meters per second squared
 * Gyro: degrees per second
 * Mags: micro teslas
 * 
 * @param mpuData MPU6050_Data datatype which holds the Accel and Gyro data
 * @param magData MAG3110_Data datatype which holds the Mag data
 * @param normData IMU_Data this holds the normalized output data
 */
void IMU_normalizeData(MPU6050_Data mpuData, MAG3110_Data magData, IMU_Data *normData);

/**
 * Updates the internal IMU algorithm with data from a new timestep.
 * This function only uses Accels and Gyros
 *
 * @param newData The new IMU data. Should be the result of IMU_normalizeData().
 */
void IMU_UpdateIMU(const IMU_Data *newData);

/**
 * Updates the internal IMU algorithm with data from a new timestep.
 * This function uses MAG data as well as Accel and Gyros
 *
 * @param newData The new IMU data. Should be the result of IMU_normalizeData().
 */
void IMU_UpdateAHRS(const IMU_Data *newData);

/**
 * Call this function after IMU_Update() has been called for the current timestep.
 * Returns the quaternion representing the vehicle's attitude.
 * @param q The quaternion.
 */
void IMU_GetQuaternion(float q[4]);

// Returns the Euler angles in radians defined with the Aerospace sequence.
// See Sebastian O.H. Madwick report
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
/**
 * Converts a quaternion representation to Euler angles.
 *
 * @param q The quaternion.
 * @param angles Radians as defined with the Aerospace sequence.
 */
void IMU_QuaternionToEuler(const float q[4], float angles[3]);

/**
 * Converts a quaternion to yaw-pitch-roll values.
 *
 * @param q The quaternion to convert.
 * @param ypr A 3-element array where the yaw-pitch-roll values will be written to.
 */
void IMU_QuaternionToYawPitchRoll(const float q[4], float ypr[3]);

/**
 * Converts a quaternion into a direction cosine matrix.
 * 
 * @param q A quaternion in [w x y z] order (w is the angle)
 * @param dcm[out] A 3x3 array to store the DCM (row-major order).
 */
void IMU_QuaternionToDCM(const float q[4], float dcm[3][3]);

void IMU_QuaternionToString(const float q[4], char out[37]);

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

#endif	/* IMU_H */

