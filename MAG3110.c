/*
 * File:   MAG3110.c
 * Author: Jonathan Bruce
 *
 * Created on October 21, 2014, 10:23 AM
 */

#include "MAG3110.h"

void MAG3110_Init(void) {
	// Set the output data rate to 80Hz, over sampling to 16, enable reading
        // all 16-bit values, set normal operation, and enter standby mode.
	MAG3110_Set_CTRL_REG1(0x00);

	// Automatically reset the mags before every data read (recommended),
        // and ignore the user offset registers and return the raw magnetometer
        // readings.
	MAG3110_Set_CTRL_REG2(MAG_AUTO_MRST_EN | MAG_RAW);

	// Activate the MAG3110
	MAG3110_SetActive(true);
}


void MAG3110_GetXData(MAG3110_Data* sensorData) {
	sensorData->mag_X_msb = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_OUT_X_MSB);
	sensorData->mag_X_lsb = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_OUT_X_LSB);
	sensorData->magX = (((int16_t) sensorData->mag_X_msb) << 8) | sensorData->mag_X_lsb;
}


void MAG3110_GetYData(MAG3110_Data* sensorData) {
	// Have to read register MAG_OUT_X_MSB to insure data is recent and good
	I2C_ReadFromReg(MAG3110_ADDRESS, MAG_OUT_X_MSB);

	sensorData->mag_Y_msb = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_OUT_Y_MSB);
	sensorData->mag_Y_lsb = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_OUT_Y_LSB);
	sensorData->magY = (((int16_t) sensorData->mag_Y_msb) << 8) | sensorData->mag_Y_lsb;
}


void MAG3110_GetZData(MAG3110_Data* sensorData) {
	// Have to read register MAG_OUT_X_MSB to insure data is recent and good
	I2C_ReadFromReg(MAG3110_ADDRESS, MAG_OUT_X_MSB);

	sensorData->mag_Z_msb = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_OUT_Z_MSB);
	sensorData->mag_Z_lsb = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_OUT_Z_LSB);
	sensorData->magZ = (((int16_t) sensorData->mag_Z_msb) << 8) | sensorData->mag_Z_lsb;
}


void MAG3110_Get3AxisData(MAG3110_Data* sensorData) {
	// X Data
	MAG3110_GetXData(sensorData);
	// Y Data
	MAG3110_GetYData(sensorData);
	// Z Data
	MAG3110_GetZData(sensorData);
}


void MAG3110_GetTempurature(MAG3110_Data* sensorData) {
	sensorData->die_temp = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_DIE_TEMP);
}


bool MAG3110_Is_X_Data_Ready(void) {
	uint8_t temp = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_DR_STATUS);
	if (temp & MAG_XDR) {
		return true;
        } else {
		return false;
        }
}


bool MAG3110_Is_Y_Data_Ready(void) {
	uint8_t temp = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_DR_STATUS);
	if (temp & MAG_YDR) {
		return true;
        } else {
		return false;
        }
}


bool MAG3110_Is_Z_Data_Ready(void) {
	uint8_t temp = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_DR_STATUS);
	if (temp & MAG_ZDR) {
		return true;
        } else {
		return false;
        }
}


bool MAG3110_Is_ZYX_Data_Ready(void) {
	uint8_t temp = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_DR_STATUS);
	if (temp & MAG_ZYXDR) {
		return true;
        } else {
		return false;
        }
}


uint8_t MAG3110_Get_DR_STATUS(void) {
	return I2C_ReadFromReg(MAG3110_ADDRESS, MAG_DR_STATUS);
}


void MAG3110_Set_Offest_Correction(uint16_t X_offset, uint16_t Y_offset, uint16_t Z_offset) {
	// Max range of offsets is -10,000 to 10,000
	if (X_offset > 10000) {
		X_offset = 10000;
	}
	if (X_offset < -10000) {
		X_offset = -10000;
	}
	if (Y_offset > 10000) {
		Y_offset = 10000;
	}
	if (Y_offset < -10000) {
		Y_offset = -10000;
	}
	if (Z_offset > 10000) {
		Z_offset = 10000;
	}
	if (Z_offset < -10000) {
		Z_offset = -10000;
	}

	// X Offset
	I2C_WriteToReg(MAG3110_ADDRESS, MAG_OFF_X_MSB, (X_offset >> 8)&0xFF);
	I2C_WriteToReg(MAG3110_ADDRESS, MAG_OFF_X_LSB, (X_offset)&0xFF);
	// Y Offset
	I2C_WriteToReg(MAG3110_ADDRESS, MAG_OFF_Y_MSB, (Y_offset >> 8)&0xFF);
	I2C_WriteToReg(MAG3110_ADDRESS, MAG_OFF_Y_LSB, (Y_offset)&0xFF);
	// Z Offset
	I2C_WriteToReg(MAG3110_ADDRESS, MAG_OFF_Z_MSB, (Z_offset >> 8)&0xFF);
	I2C_WriteToReg(MAG3110_ADDRESS, MAG_OFF_Z_LSB, (Z_offset)&0xFF);

	// Diables Raw data output so that offests are applied
	MAG3110_SetRaw(false);
}


void MAG3110_Set_CTRL_REG1(unsigned char reg_value) {
	uint8_t temp;
	temp = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_CTRL_REG1);

	// Checks if the mag is active
	// if true, put into standby and change the register then place back into active mode
	// else just load the register
	if (temp & MAG_AC) {
		I2C_WriteToReg(MAG3110_ADDRESS, MAG_CTRL_REG1, (temp & ~(1 << MAG_AC_BITSHIFT)));
		I2C_WriteToReg(MAG3110_ADDRESS, MAG_CTRL_REG1, reg_value);
		I2C_WriteToReg(MAG3110_ADDRESS, MAG_CTRL_REG1, (temp | (1 << MAG_AC_BITSHIFT)));
	} else {
		I2C_WriteToReg(MAG3110_ADDRESS, MAG_CTRL_REG1, reg_value);
	}
}


void MAG3110_Set_CTRL_REG2(unsigned char reg_value) {
	I2C_WriteToReg(MAG3110_ADDRESS, MAG_CTRL_REG2, reg_value);
}


void MAG3110_SetActive(bool enabled) {
	uint8_t temp;
	temp = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_CTRL_REG1);
	if (enabled) {
		temp |= (1 << MAG_AC_BITSHIFT);
	} else {
		temp &= ~(1 << MAG_AC_BITSHIFT);
	}
	I2C_WriteToReg(MAG3110_ADDRESS, MAG_CTRL_REG1, temp);
}


void MAG3110_SetRaw(bool enabled) {
	uint8_t temp;
	temp = I2C_ReadFromReg(MAG3110_ADDRESS, MAG_CTRL_REG2);
	if (enabled) {
		temp |= (1 << MAG_RAW_BITSHIFT);
	} else {
		temp &= ~(1 << MAG_RAW_BITSHIFT);
	}
	MAG3110_Set_CTRL_REG2(temp);
}
