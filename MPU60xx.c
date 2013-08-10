/**
 * The MIT License (MIT)
 * 
 * Copyright (c) 2013 Pavlo Milo Manovi
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#include "MPU60xx.h"
#include "I2CdsPIC.h"
#include <stdint.h>
#include <xc.h>
#include <i2c.h>

uint8_t buffer[11];
uint16_t intBuffer;


void MPU60xx_Init() {
    MPU60xx_SetClockSource(CLOCK_PLL_XGYRO);
    MPU60xx_SetGyroRange(GYRO_FS_250);
    MPU60xx_SetAccelRange(ACCEL_FS_2);
    MPU60xx_Sleep(0);
}


void MPU60xx_Sleep(uint8_t enabled) {
    buffer[0] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1);
    if (enabled) {
        buffer[0] = (buffer[0] | 1 << PWR1_SLEEP_BIT);
    } else {
        buffer[0] = (buffer[0] & ~(1 << PWR1_SLEEP_BIT));
    }
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1, buffer[0]);
}


void MPU60xx_SetGyroRange(uint8_t range) {
    buffer[0] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_CONFIG);
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_GYRO_CONFIG, ((buffer[0] & 0xE0) | (range << 3)));
}


void MPU60xx_SetAccelRange(uint8_t range) {
    buffer[0] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_CONFIG);
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_ACCEL_CONFIG, ((buffer[0] & 0xE0) | (range << 3)));
}


void MPU60xx_Get6AxisData(MPU6050_Data *sensorData) {
    buffer[0] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_XOUT_H);
    buffer[1] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_XOUT_L);
    buffer[2] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_YOUT_H);
    buffer[3] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_YOUT_L);
    buffer[4] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_ZOUT_H);
    buffer[5] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_ZOUT_L);
    buffer[6] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_XOUT_H);
    buffer[7] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_XOUT_L);
    buffer[8] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_YOUT_H);
    buffer[9] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_YOUT_L);
    buffer[10] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_ZOUT_H);
    buffer[11] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_ZOUT_L);

    sensorData->accelX = (((int16_t) buffer[0]) << 8) | buffer[1];
    sensorData->accelY = (((int16_t) buffer[2]) << 8) | buffer[3];
    sensorData->accelZ = (((int16_t) buffer[4]) << 8) | buffer[5];
    sensorData->gyroX = (((int16_t) buffer[6]) << 8) | buffer[7];
    sensorData->gyroY = (((int16_t) buffer[8]) << 8) | buffer[9];
    sensorData->gyroZ = (((int16_t) buffer[10]) << 8) | buffer[11];
    sensorData->newData = 1;
}


void MPU60xx_GetTemperature(MPU6050_Data *sensorData) {
    //This may cause sampling issues, TODO see if non-burst reading will cause issues.
    buffer[0] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_TEMP_OUT_H);
    buffer[1] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_TEMP_OUT_L);
    sensorData->temperature = (((int16_t) buffer[0] << 8) | buffer[1]);
}


void MPU60xx_GetTempSensorEnabled(uint8_t enabled) {
    buffer[0] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1);
    if (!enabled) {
        buffer[0] = (buffer[0] | 1 << PWR1_TEMP_DIS_BIT);
    } else {
        buffer[0] = (buffer[0] & ~(1 << PWR1_TEMP_DIS_BIT));
    }
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1, buffer[0]);
}


uint8_t MPU60xx_GetDeviceID() {
    return (I2C_ReadFromReg(DEFAULT_ADDRESS, RA_WHO_AM_I));
}


void MPU60xx_SetClockSource(uint8_t source) {
    buffer[0] = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1);
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1, ((buffer[0] & 0xF8) | source));
}

