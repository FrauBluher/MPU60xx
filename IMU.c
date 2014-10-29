/* 
 * File:   IMU.c
 * Author: Jonathan
 *
 * Created on October 23, 2014, 11:26 AM
 *
 * Any code below which has the following comment around it is derived from the
 * PIC24 FreeIMU project:
 * ****************** FreeIMU ******************************
 * *********************************************************
 *
 */

#include "IMU.h"
#include "MadgwickAHRS/MadgwickAHRS.h"

uint8_t accelRange;
uint8_t gyroRange;


void IMU_Init(uint32_t i2cFreq, uint32_t sysFreq)
{
	// Initializes the i2c line, MPU60x0, and MAG3110
	I2C_Init(I2C_CALC_BRG(i2cFreq, sysFreq));

	accelRange = ACCEL_FS_2;
	gyroRange = GYRO_FS_250;
	MPU60xx_Init(accelRange, gyroRange, true);
	MAG3110_Init();

	IMU_SetMPUMaster();
}

void IMU_SetMPUMaster(void)
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

void IMU_normalizeData(MPU6050_Data mpuData, MAG3110_Data magData, IMU_Data *normData)
{
	// Derive Normalization Factor
	float accelNormalizer = 16384.0 / (accelRange + 1);
	float gyroNormalizer = 131.0 / (gyroRange + 1);
	const static float magNormalizer = 10.0; // TODO: add support for user defined mag offset

	// Normalize Accel, Gyro, and Mag data
	// Accels are in units of m/s^2
	normData->accelX = (mpuData.accelX / accelNormalizer) * G_FORCE;
	normData->accelY = (mpuData.accelY / accelNormalizer) * G_FORCE;
	normData->accelZ = (mpuData.accelZ / accelNormalizer) * G_FORCE;
	// Gyros are in Degrees/s
	normData->gyroX = (mpuData.gyroX / gyroNormalizer);
	normData->gyroY = (mpuData.gyroY / gyroNormalizer);
	normData->gyroZ = (mpuData.gyroZ / gyroNormalizer);
	// Mags are in micro Teslas
	normData->magX = (magData.magX / magNormalizer);
	normData->magY = (magData.magY / magNormalizer);
	normData->magZ = (magData.magZ / magNormalizer);
}

void IMU_UpdateIMU(const IMU_Data *newData)
{
	// gyro values are expressed in deg/sec, convert to radians/sec
	MadgwickAHRSupdateIMU(DEG2RAD(newData->gyroX), DEG2RAD(newData->gyroY), DEG2RAD(newData->gyroZ), newData->accelX, newData->accelY, newData->accelZ);
}

void IMU_UpdateAHRS(const IMU_Data *newData)
{
	// gyro values are expressed in deg/sec, convert to radians/sec
	MadgwickAHRSupdate(DEG2RAD(newData->gyroX), DEG2RAD(newData->gyroY), DEG2RAD(newData->gyroZ), newData->accelX, newData->accelY, newData->accelZ, newData->magX, newData->magY, newData->magZ);
}

void IMU_GetQuaternion(float q[4])
{
	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}


void IMU_QuaternionToEuler(const float q[4], float angles[3])
{
	angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1); // psi
	angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
	angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}


void IMU_QuaternionToYawPitchRoll(const float q[4], float ypr[3])
{
        // Estimate the gravity vector
	float gx, gy, gz;
	gx = 2 * (q[1] * q[3] - q[0] * q[2]);
	gy = 2 * (q[0] * q[1] + q[2] * q[3]);
	gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

        // Calculate yaw (rads)
	ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);

        // Calculate pitch (rads)
	ypr[1] = atan(gx / sqrt(gy * gy + gz * gz));

        // Calculate roll (rads)
	ypr[2] = atan(gy / sqrt(gx * gx + gz * gz));
}

// Based on code from: https://github.com/kieranwood85/rotlib/blob/master/r_q_to_dcm.m
void IMU_QuaternionToDCM(const float q[4], float dcm[3][3])
{
	dcm[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	dcm[0][1] = 2 * (q[1] * q[2] + q[0] * q[3]);
	dcm[0][2] = 2 * (q[1] * q[3] - q[0] * q[2]);
	dcm[1][0] = 2 * (q[1] * q[2] - q[0] * q[3]);
	dcm[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
	dcm[1][2] = 2 * (q[2] * q[3] + q[0] * q[1]);
	dcm[2][0] = 2 * (q[1] * q[3] + q[0] * q[2]);
	dcm[2][1] = 2 * (q[2] * q[3] - q[0] * q[1]);
	dcm[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

typedef union {
	float r32;
	uint32_t u32;
} conv_union;

/**
 * Converts from a byte to a 2-character ASCII hex representation (2nd character is the LSB)
 * @param data
 * @param out
 */
void Byte2Hex(uint8_t data, char out[2])
{
    uint8_t lowerHalf = data & 0xF;
    uint8_t upperHalf = (data >> 4) & 0xF;

    if (lowerHalf <= 9) {
        out[1] = '0' + lowerHalf;
    } else if (lowerHalf > 9) {
        out[1] = 'A' + (lowerHalf - 10);
    } else {
        out[1] = '0';
    }

    if (upperHalf <= 9) {
        out[0] = '0' + upperHalf;
    } else if (upperHalf > 9) {
        out[0] = 'A' + (upperHalf - 10);
    } else {
        out[0] = '0';
    }
}

void Real32ToLEBytes(float data, uint8_t bytes[4])
{
    conv_union tmp;
    tmp.r32 = data;
    bytes[0] = (uint8_t)tmp.u32;
    bytes[1] = (uint8_t)(tmp.u32 >> 8);
    bytes[2] = (uint8_t)(tmp.u32 >> 16);
    bytes[3] = (uint8_t)(tmp.u32 >> 24);
}

void IMU_QuaternionToString(const float q[4], char out[38])
{
    uint8_t bytes[4];
    Real32ToLEBytes(q[0], bytes);
    Byte2Hex(bytes[0], &out[0]);
    Byte2Hex(bytes[1], &out[2]);
    Byte2Hex(bytes[2], &out[4]);
    Byte2Hex(bytes[3], &out[6]);
    out[8] = ',';
    Real32ToLEBytes(q[1], bytes);
    Byte2Hex(bytes[0], &out[9]);
    Byte2Hex(bytes[1], &out[11]);
    Byte2Hex(bytes[2], &out[13]);
    Byte2Hex(bytes[3], &out[15]);
    out[17] = ',';
    Real32ToLEBytes(q[2], bytes);
    Byte2Hex(bytes[0], &out[18]);
    Byte2Hex(bytes[1], &out[20]);
    Byte2Hex(bytes[2], &out[22]);
    Byte2Hex(bytes[3], &out[24]);
    out[26] = ',';
    Real32ToLEBytes(q[3], bytes);
    Byte2Hex(bytes[0], &out[27]);
    Byte2Hex(bytes[1], &out[29]);
    Byte2Hex(bytes[2], &out[31]);
    Byte2Hex(bytes[3], &out[33]);
    out[35] = ',';
    out[36] = '\n';
    out[37] = '\0';
}