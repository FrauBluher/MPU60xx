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

void MPU60xx_Init(uint8_t rangeOfAccel, uint8_t rangeOfGyro, bool enable_passthrough)
{
	// Set the clock source to one of the gyros
	// (as recommended by the docs)
	MPU60xx_SetClockSource(CLOCK_PLL_XGYRO);

	// Enable the chip. If this isn't done early enough in the initialization
        // process, some settings don't stick and we don't get output data.
        // Uncertain as to the specifics of it, but no harm in moving it up here.
	MPU60xx_SetEnabled(true);

	// Divide the sample rate by 5
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_SMPRT_DIV, 4);

	// Low-pass filter the gyro and accel data at ~188Hz. Note that this reduces
	// the gyro sample rate to 1kHz. With this and the above SMPRT_DIV setting,
	// we end up getting raw data out at 200Hz.
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_CONFIG, 0x01);

	// Set the gyro and accel sensitivity to its highest.
	MPU60xx_SetGyroRange(rangeOfAccel);
	MPU60xx_SetAccelRange(rangeOfGyro);

	// The interrupt status bits are cleared whenever a register is read. This
	// allows us to use the Data Ready INT pin without reading the INT_STATUS
	// register every time.
	uint8_t tmp = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_INT_PIN_CONFIG);
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_INT_PIN_CONFIG, ((tmp | (1 << 4))));

	// Allow for slave devices connected to the Aux I2C pins
	MPU60xx_SetI2CAuxPassthrough(enable_passthrough);

	// Enable FIFO for temperature, gyros, and accels
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_FIFO_EN, 0xF8);

	// Turn on the FIFO hardware
	tmp = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_USER_CONTROL);
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_USER_CONTROL, ((tmp | (1 << 6))));

	// Turn on data ready interrupts so the host processor can read data when it's ready.
	// We disable all other interrupt sources for the INT pin
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_INT_ENABLE, 0x01);
}

void MPU60xx_SetEnabled(bool enabled)
{
	uint8_t powerManagementReg = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_PWR_MGMT_1);
	if (enabled) {
		powerManagementReg &= ~(1 << PWR1_SLEEP_BIT);
	} else {
		powerManagementReg |= 1 << PWR1_SLEEP_BIT;
	}
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_PWR_MGMT_1, powerManagementReg);
}

void MPU60xx_SetI2CAuxPassthrough(bool enabled)
{
	uint8_t tmp = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_INT_PIN_CONFIG);
	if (enabled) {
		I2C_WriteToReg(MPU60XX_ADDRESS, RA_INT_PIN_CONFIG, ((tmp | (1 << 1))));

		// Disable master mode if we're using i2c aux passthrough
		tmp = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_USER_CONTROL);
		I2C_WriteToReg(MPU60XX_ADDRESS, RA_USER_CONTROL, ((tmp & ~(1 << 5))));
	} else {
		I2C_WriteToReg(MPU60XX_ADDRESS, RA_INT_PIN_CONFIG, ((tmp & ~(1 << 1))));

		// And if we are not using aux passthrough, let the MPU be a master
		tmp = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_USER_CONTROL);
		I2C_WriteToReg(MPU60XX_ADDRESS, RA_USER_CONTROL, ((tmp | (1 << 5))));
	}

}

void MPU60xx_SetGyroRange(uint8_t range)
{
	uint8_t gyroConfigReg = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_GYRO_CONFIG);
	gyroConfigReg = ((gyroConfigReg & 0xE0) | (range << 3));
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_GYRO_CONFIG, gyroConfigReg);
}

void MPU60xx_SetAccelRange(uint8_t range)
{
	uint8_t accelConfigReg = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_ACCEL_CONFIG);
	accelConfigReg = ((accelConfigReg & 0xE0) | (range << 3));
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_ACCEL_CONFIG, accelConfigReg);
}

void MPU60xx_GetData(MPU6050_Data *sensorData)
{
	uint8_t data[14]; // TODO: Set to proper byte size dependent on what sampling is being done.

	// Read how many bytes have been loaded into the FIFO
	I2C_ReadFromReg_Burst(MPU60XX_ADDRESS, RA_FIFO_COUNT_H, data, 2);
	uint16_t bytesInBuffer = (data[0] << 8) | data[1];

	// If there's data available, read a single timestamp
	if (bytesInBuffer >= sizeof(data)) {
		I2C_ReadFromReg_Burst(MPU60XX_ADDRESS, RA_FIFO_R_W, data, 14);

		// Now fit this data into our nice pretty struct
		sensorData->accelX = data[1];
		sensorData->accelX |= data[0] << 8;
		sensorData->accelY = data[3];
		sensorData->accelY |= data[2] << 8;
		sensorData->accelZ = data[5];
		sensorData->accelZ |= data[4] << 8;
		sensorData->temp = data[7];
		sensorData->temp |= data[6] << 8;
		sensorData->gyroX = data[9];
		sensorData->gyroX |= data[8] << 8;
		sensorData->gyroY = data[11];
		sensorData->gyroY |= data[10] << 8;
		sensorData->gyroZ = data[13];
		sensorData->gyroZ |= data[12] << 8;
	}
}

void MPU60xx_GetTemperature(MPU6050_Data *sensorData)
{
	uint8_t tempOutRegHigh = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_TEMP_OUT_H);
	uint8_t tempOutRegLow = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_TEMP_OUT_L);
	sensorData->temp = (((int16_t) tempOutRegHigh << 8) | tempOutRegLow);
}

void MPU60xx_SetTempSensorEnabled(bool enabled)
{
	uint8_t powerManagementReg = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_PWR_MGMT_1);
	if (!enabled) {
		powerManagementReg |= 1 << PWR1_TEMP_DIS_BIT;
	} else {
		powerManagementReg &= ~(1 << PWR1_TEMP_DIS_BIT);
	}
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_PWR_MGMT_1, powerManagementReg);
}

uint8_t MPU60xx_GetDeviceID(void)
{
	return(I2C_ReadFromReg(MPU60XX_ADDRESS, RA_WHO_AM_I));
}

void MPU60xx_SetClockSource(uint8_t source)
{
	uint8_t powerManagementReg = I2C_ReadFromReg(MPU60XX_ADDRESS, RA_PWR_MGMT_1);
	powerManagementReg = (powerManagementReg & 0xF8) | source;
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_PWR_MGMT_1, powerManagementReg);
}