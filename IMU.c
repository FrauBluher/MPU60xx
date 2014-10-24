/* 
 * File:   IMU.c
 * Author: Jonathan
 *
 * Created on October 23, 2014, 11:26 AM
 */

#include "IMU.h"

void IMU_Init(uint32_t i2cFreq, uint32_t sysFreq)
{
	// Initializes the i2c line, MPU60x0, and MAG3110
	I2C_Init(I2C_CALC_BRG(i2cFreq, sysFreq));
	MPU60xx_Init(true);
	MAG3110_Init();

	IMU_SetMPUMaster();
}

void IMU_SetMPUMaster()
{
	// Disables i2c aux passthrough and sets the MPU as a master on the aux line
	MPU60xx_SetI2CAuxPassthrough(false);

	// Writes configuration bits for the MPU i2c master
	// Enables wait for external data for sync, and read characteristics
	// Also sets the MPU master i2c clock at 400kHz
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_I2C_MST_CTRL, (0x40 | 0xD));

	// Write the MAG3110 address (0x0E) to the Slave 0 address register
	// Also sets the Slave 0 read/write bit as a read transaction
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_SLV0_ADDR, (1 << 7) | MAG3110_ADDRESS);

	// Writes the Address to start reading from as the MAG3110 X MSB register (0x01)
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_SLV0_REG, MAG_OUT_X_MSB);

	// This register controls how the MPU will retrieve data from Slave 0
	// This write will enable Slave 0, Set grouping of words to be
	// Odd then Even (since MAG_OUT_X_MSB = 0x01) and reads 6 bytes
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_SLV0_CTRL, (0x90 | 0x6));

	// Enables FIFO for Temp, Gyros, Accels, and Slave 0 data
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_FIFO_EN, 0xF9);
}

void IMU_GetData(MPU6050_Data *mpuData, MAG3110_Data *magData)
{
	uint8_t data[20]; //TODO: Set to proper byte size dependent on what sampling is being done.

	// Read how many bytes have been loaded into the FIFO
	I2C_ReadFromReg_Burst(MPU60XX_ADDRESS, RA_FIFO_COUNT_H, data, 2);
	uint16_t bytesInBuffer = (data[0] << 8) | data[1];

	// If there's data available, read a single timestamp
	if (bytesInBuffer >= sizeof(data)) {
		I2C_ReadFromReg_Burst(MPU60XX_ADDRESS, RA_FIFO_R_W, data, sizeof(data));

		// Now fit this data into our nice pretty structs
		mpuData->accelX = data[1];
		mpuData->accelX |= data[0] << 8;
		mpuData->accelY = data[3];
		mpuData->accelY |= data[2] << 8;
		mpuData->accelZ = data[5];
		mpuData->accelZ |= data[4] << 8;
		mpuData->temp = data[7];
		mpuData->temp |= data[6] << 8;
		mpuData->gyroX = data[9];
		mpuData->gyroX |= data[8] << 8;
		mpuData->gyroY = data[11];
		mpuData->gyroY |= data[10] << 8;
		mpuData->gyroZ = data[13];
		mpuData->gyroZ |= data[12] << 8;

		magData->mag_X_msb = data[14];
		magData->mag_X_lsb = data[15];
		magData->magX = (((int16_t) magData->mag_X_msb) << 8) | magData->mag_X_lsb;
		magData->mag_Y_msb = data[16];
		magData->mag_Y_lsb = data[17];
		magData->magY = (((int16_t) magData->mag_Y_msb) << 8) | magData->mag_Y_lsb;
		magData->mag_Z_msb = data[18];
		magData->mag_Z_lsb = data[19];
		magData->magZ = (((int16_t) magData->mag_Z_msb) << 8) | magData->mag_Z_lsb;
	}
}
