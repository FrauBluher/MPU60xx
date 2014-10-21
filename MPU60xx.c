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

// Standard headers
#include <stdint.h>

// Microchip headers
#include <i2c.h>
#include <xc.h>

// User headers
#include "I2CdsPIC.h"
#include "MPU60xx.h"

void MPU60xx_Init(bool enable_passthrough) {
    // Set the clock source to one of the gyros
    // (as recommended by the docs)
    MPU60xx_SetClockSource(CLOCK_PLL_XGYRO);

    // Set the gyro and accel sensitivity to its highest
    MPU60xx_SetGyroRange(GYRO_FS_250);
    MPU60xx_SetAccelRange(ACCEL_FS_2);

    MPU60xx_SetI2CAuxPassthrough(enable_passthrough);

    // And finally enable the sensor
    MPU60xx_SetEnabled(true);
}


void MPU60xx_SetEnabled(bool enabled) {
    uint8_t powerManagementReg = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1);
    if (enabled) {
        powerManagementReg &= ~(1 << PWR1_SLEEP_BIT);
    } else {
        powerManagementReg |= 1 << PWR1_SLEEP_BIT;
    }
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1, powerManagementReg);
}


void MPU60xx_SetI2CAuxPassthrough(bool enabled) {
    uint8_t tmp = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_INT_PIN_CONFIG);
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_INT_PIN_CONFIG, ((tmp & (enabled << 1))));
    tmp = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_USER_CONTROL);
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_USER_CONTROL, ((tmp & (!enabled << 1))));
}


void MPU60xx_SetGyroRange(uint8_t range) {
    uint8_t gyroConfigReg = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_CONFIG);
    gyroConfigReg = ((gyroConfigReg & 0xE0) | (range << 3));
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_GYRO_CONFIG, gyroConfigReg);
}


void MPU60xx_SetAccelRange(uint8_t range) {
    uint8_t accelConfigReg = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_CONFIG);
    accelConfigReg = ((accelConfigReg & 0xE0) | (range << 3));
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_ACCEL_CONFIG, accelConfigReg);
}


void MPU60xx_Get6AxisData(MPU6050_Data *sensorData) {
    // Use some temp variables for reading out the sensor data
    uint8_t tempLow, tempHigh;

    tempHigh = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_XOUT_H);
    tempLow = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_XOUT_L);
    sensorData->accelX = (((int16_t) tempHigh) << 8) | tempLow;

    tempHigh = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_YOUT_H);
    tempLow = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_YOUT_L);
    sensorData->accelY = (((int16_t) tempHigh) << 8) | tempLow;

    tempHigh = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_ZOUT_H);
    tempLow = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_ACCEL_ZOUT_L);
    sensorData->accelZ = (((int16_t) tempHigh) << 8) | tempLow;

    tempHigh = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_XOUT_H);
    tempLow = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_XOUT_L);
    sensorData->gyroX = (((int16_t) tempHigh) << 8) | tempLow;

    tempHigh = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_YOUT_H);
    tempLow = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_YOUT_L);
    sensorData->gyroY = (((int16_t) tempHigh) << 8) | tempLow;

    tempHigh = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_ZOUT_H);
    tempLow = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_GYRO_ZOUT_L);
    sensorData->gyroZ = (((int16_t) tempHigh) << 8) | tempLow;

    // And indicate we have new data
    sensorData->newData = true;
}


void MPU60xx_GetTemperature(MPU6050_Data *sensorData) {
    // This may cause sampling issues, TODO see if non-burst reading will cause issues.
    uint8_t tempOutRegHigh = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_TEMP_OUT_H);
    uint8_t tempOutRegLow = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_TEMP_OUT_L);
    sensorData->temperature = (((int16_t) tempOutRegHigh << 8) | tempOutRegLow);
}


void MPU60xx_SetTempSensorEnabled(bool enabled) {
    uint8_t powerManagementReg = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1);
    if (!enabled) {
        powerManagementReg |= 1 << PWR1_TEMP_DIS_BIT;
    } else {
        powerManagementReg &= ~(1 << PWR1_TEMP_DIS_BIT);
    }
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1, powerManagementReg);
}


uint8_t MPU60xx_GetDeviceID(void) {
    return (I2C_ReadFromReg(DEFAULT_ADDRESS, RA_WHO_AM_I));
}


void MPU60xx_SetClockSource(uint8_t source) {
    uint8_t powerManagementReg = I2C_ReadFromReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1);
    powerManagementReg = (powerManagementReg & 0xF8) | source;
    I2C_WriteToReg(DEFAULT_ADDRESS, RA_PWR_MGMT_1, powerManagementReg);
}

