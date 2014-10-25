/* 
 * File:   IMU.h
 * Author: Jonathan
 *
 * Created on October 23, 2014, 11:27 AM
 */

#ifndef IMU_H
#define	IMU_H

#include "I2CdsPIC.h"
#include "MPU60xx.h"
#include "MAG3110.h"

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
#endif	/* IMU_H */

