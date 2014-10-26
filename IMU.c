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

uint8_t accelRange;
uint8_t gyroRange;

/* ****************** FreeIMU ******************************
 * ********************************************************* */
float iq0, iq1, iq2, iq3;
float exInt, eyInt, ezInt; // scaled integral error
volatile float twoKp; // 2 * proportional gain (Kp)
volatile float twoKi; // 2 * integral gain (Ki)
volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx, integralFBy, integralFBz;
unsigned long lastUpdate, now; // sample period expressed in milliseconds
float sampleFreq = FREQ_HZ; // half the sample period expressed in seconds
int startLoopTime;

int gRawData[11]; // concatenate accelerometer, gyroscope, magnetometer, temperature and pressure data
float gRealData[11]; //

/* ****************** FreeIMU ******************************
 * ********************************************************* */

void IMU_Init(uint32_t i2cFreq, uint32_t sysFreq)
{
	// Initializes the i2c line, MPU60x0, and MAG3110
	I2C_Init(I2C_CALC_BRG(i2cFreq, sysFreq));

	accelRange = ACCEL_FS_2;
	gyroRange = GYRO_FS_250;
	MPU60xx_Init(accelRange, gyroRange, true);
	MAG3110_Init();

	IMU_SetMPUMaster();

	/* ****************** FreeIMU ******************************
	 * ********************************************************* */
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	twoKp = twoKpDef;
	twoKi = twoKiDef;
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
	lastUpdate = 0;
	now = 0;
	/* ****************** FreeIMU ******************************
	 * ********************************************************* */
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
	float magNormalizer = 10.0; // TODO: add support for user defined mag offset

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


/* ****************** FreeIMU ******************************
 * ********************************************************* */
void imu_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float halfex = 0.0f;
	float halfey = 0.0f;
	float halfez = 0.0f;
	float qa, qb, qc;

	// Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
	if ((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
		float hx, hy, bx, bz;
		float halfwx, halfwy, halfwz;

		// Normalise magnetometer measurement
		recipNorm = imu_invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of magnetic field
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (my * halfwz - mz * halfwy);
		halfey = (mz * halfwx - mx * halfwz);
		halfez = (mx * halfwy - my * halfwx);
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if ((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
		float halfvx, halfvy, halfvz;

		// Normalise accelerometer measurement
		recipNorm = imu_invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += (ay * halfvz - az * halfvy);
		halfey += (az * halfvx - ax * halfvz);
		halfez += (ax * halfvy - ay * halfvx);
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx; // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f; // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = imu_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void imu_getQ(IMU_Data imuData, float * q)
{
	// gyro values are expressed in deg/sec, convert to radians/sec
	imu_AHRSupdate(DEG2RAD(imuData.gyroX), DEG2RAD(imuData.gyroY), DEG2RAD(imuData.gyroZ), imuData.accelX, imuData.accelY, imuData.accelZ, imuData.magX, imuData.magY, imuData.magZ);

	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}

// Returns the Euler angles in radians defined with the Aerospace sequence.
// See Sebastian O.H. Madwick report
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation

void imu_getEuler(IMU_Data imuData, float * angles)
{
	float q[4]; // quaternion
	imu_getQ(imuData, q);
	angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1) * 180 / M_PI; // psi
	angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180 / M_PI; // theta
	angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180 / M_PI; // phi
}

void imu_getYawPitchRoll(IMU_Data imuData, float * ypr)
{
	float q[4]; // quaternion
	float gx, gy, gz; // estimated gravity direction
	imu_getQ(imuData, q);

	gx = 2 * (q[1] * q[3] - q[0] * q[2]);
	gy = 2 * (q[0] * q[1] + q[2] * q[3]);
	gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1) * 180 / M_PI;
	ypr[1] = atan(gx / sqrt(gy * gy + gz * gz)) * 180 / M_PI;
	ypr[2] = atan(gy / sqrt(gx * gx + gz * gz)) * 180 / M_PI;
}

float imu_invSqrt(float number)
{
	volatile long i;
	volatile float x, y;
	volatile const float f = 1.5F;

	x = number * 0.5F;
	y = number;
	i = *((long *) &y);
	i = 0x5f375a86 - (i >> 1);
	y = *((float *) &i);
	y = y * (f - (x * y * y));
	return y;
}
/* ****************** FreeIMU ******************************
 * ********************************************************* */