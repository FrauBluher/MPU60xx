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
#endif	/* IMU_H */

