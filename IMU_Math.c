#include "IMU_Math.h"

#include <math.h>
#include <stdint.h>

// Specify how many buffers the MoveAvg8() can support. Set to 32 for no particular reason as this
// function is currently unused.
#define MOVING_AVG_BUFFER 32

void QuaternionToEuler(const float q[4], float angles[3])
{
	angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1); // psi
	angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
	angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

void YawPitchRollToQuaternion(const float ypr[3], float q[4])
{
    // Precalculate sine/cosines for the yaw/pitch/roll angles
    const float c_ypr[3] = {cos(ypr[0] / 2), cos(ypr[1] / 2), cos(ypr[2] / 2)};
    const float s_ypr[3] = {sin(ypr[0] / 2), sin(ypr[1] / 2), sin(ypr[2] / 2)};

    q[0] = c_ypr[0] * c_ypr[1] * c_ypr[2] + s_ypr[0] * s_ypr[1] * s_ypr[2];
    q[1] = c_ypr[0] * c_ypr[1] * s_ypr[2] - s_ypr[0] * s_ypr[1] * c_ypr[2];
    q[2] = c_ypr[0] * s_ypr[1] * c_ypr[2] + s_ypr[0] * c_ypr[1] * s_ypr[2];
    q[3] = s_ypr[0] * c_ypr[1] * c_ypr[2] - c_ypr[0] * s_ypr[1] * s_ypr[2];
}


void QuaternionToYawPitchRoll(const float q[4], float ypr[3])
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
void QuaternionToDCM(const float q[4], float dcm[3][3])
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

void QuaternionToString(const float q[4], char out[37])
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
}

int MoveAvg8(int NewValue, int bufferNum)
{
    // FIXME: Change the following array initializations to use standard C-syntax
	static int RunSum[MOVING_AVG_BUFFER] = {[0 ... (MOVING_AVG_BUFFER - 1)] = 0};
	static int Buffer[MOVING_AVG_BUFFER][8] = {[0 ... (MOVING_AVG_BUFFER - 1)] = {0, 0, 0, 0, 0, 0, 0, 0}};
	static unsigned char Newest[MOVING_AVG_BUFFER] = {[0 ... (MOVING_AVG_BUFFER - 1)] = 0};
	static unsigned char Oldest[MOVING_AVG_BUFFER] = {[0 ... (MOVING_AVG_BUFFER - 1)] = 1};

	// update the running sum: remove oldest value and add in the newest
	RunSum[bufferNum] = RunSum[bufferNum] - Buffer[bufferNum][Oldest[bufferNum]] + NewValue;

	// put the new value into the buffer
	Buffer[bufferNum][Newest[bufferNum]] = NewValue;

	// update the indices by incrementing modulo 8
	// for a binary modulo, the AND operation is faster than using %
	Newest[bufferNum] = (Newest[bufferNum] + 1) & 0x07;
	Oldest[bufferNum] = (Oldest[bufferNum] + 1) & 0x07;

	// now return the result by diving by 8 (shortcut: shift right by 3)
	return(RunSum[bufferNum] >> 3);
}