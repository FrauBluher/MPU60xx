/*
 * File:   MAG3110.h
 * Author: Jonathan Bruce
 *
 * Created on October 20, 2014, 5:33 PM
 */

#ifndef MAG3110_H
#define	MAG3110_H

// Standard headers
#include <stdint.h>
#include <stdbool.h>

// User headers
#include "I2CdsPIC.h"

typedef struct {
    int8_t  mag_X_msb;
    int8_t  mag_X_lsb;
    int16_t magX;
    int8_t  mag_Y_msb;
    int8_t  mag_Y_lsb;
    int16_t magY;
    int8_t  mag_Z_msb;
    int8_t  mag_Z_lsb;
    int16_t magZ;
    int8_t  die_temp;
}MAG3110_Data;

#define MAG_DR_STATUS		0x00

#define MAG_OUT_X_MSB		0x01
#define MAG_OUT_X_LSB		0x02
#define MAG_OUT_Y_MSB		0x03
#define MAG_OUT_Y_LSB		0x04
#define MAG_OUT_Z_MSB		0x05
#define MAG_OUT_Z_LSB		0x06

#define MAG_WHO_AM_I		0x07
#define MAG_SYSMOD		0x08

#define MAG_OFF_X_MSB		0x09
#define MAG_OFF_X_LSB		0x0A
#define MAG_OFF_Y_MSB		0x0B
#define MAG_OFF_Y_LSB		0x0C
#define MAG_OFF_Z_MSB		0x0D
#define MAG_OFF_Z_LSB		0x0E

#define MAG_DIE_TEMP		0x0F

#define MAG_CTRL_REG1		0x10
#define MAG_CTRL_REG2		0x11

//DR_STATUS
#define MAG_ZYXOW		0x80
#define MAG_ZOW			0x40
#define MAG_YOW			0x20
#define MAG_XOW			0x10
#define MAG_ZYXDR		0x08
#define MAG_ZDR			0x04
#define MAG_YDR			0x02
#define MAG_XDR			0x01

#define MAG_ZYXOW_BITSHIFT	7
#define MAG_ZOW_BITSHIFT	6
#define MAG_YOW_BITSHIFT	5
#define MAG_XOW_BITSHIFT	4
#define MAG_ZYXDR_BITSHIFT	3
#define MAG_ZDR_BITSHIFT	2
#define MAG_YDR_BITSHIFT	1
#define MAG_XDR_BITSHIFT	0

//SYSMOD
#define MAG_SYSMOD1		0x02
#define MAG_SYSMOD0		0x01

#define MAG_SYSMOD1_BITSHIFT	1
#define MAG_SYSMOD0_BITSHIFT	0

//CTRL_REG1
#define MAG_DR2			0x80
#define MAG_DR1			0x40
#define MAG_DR0			0x20
#define MAG_OS1			0x10
#define MAG_OS0			0x08
#define MAG_FR			0x04
#define MAG_TM			0x02
#define MAG_AC			0x01

#define MAG_DR2_BITSHIFT	7
#define MAG_DR1_BITSHIFT	6
#define MAG_DR0_BITSHIFT	5
#define MAG_OS1_BITSHIFT	4
#define MAG_OS0_BITSHIFT	3
#define MAG_FR_BITSHIFT		2
#define MAG_TM_BITSHIFT		1
#define MAG_AC_BITSHIFT 	0

//CTRL_REG2
#define MAG_AUTO_MRST_EN	0x80
#define MAG_RAW			0x20
#define MAG_Mag_RST		0x10

#define MAG_AUTO_MRST_EN_BITSHIFT	7
#define MAG_RAW_BITSHIFT        	5
#define MAG_Mag_RST_BITSHIFT            4

// The i2c device address for the MAG3110
#define MAG3110_ADDRESS 0x0E

/**
 * @brief Inits the MAG3110 and sets active
 * Initializes Auto Magnetic Sensor Read and Raw data output
 * Everything else is left as default
 *
 * @see pages 19-21 of MAG3110.pdf
 */
void MAG3110_Init(void);

/**
 * Retrieves X axis data from the MAG3110
 *
 * @param sensorData
 */
void MAG3110_GetXData(MAG3110_Data* sensorData);

/**
 * Retrieves Y axis data from the MAG3110
 *
 * @param sensorData
 */
void MAG3110_GetYData(MAG3110_Data* sensorData);

/**
 * Retrieves Z axis data from the MAG3110
 *
 * @param sensorData
 */
void MAG3110_GetZData(MAG3110_Data* sensorData);

/**
 * Retrieves Z, Y, & X axis data from MAG3110 using bursting mode
 * Bursting allows for all the data retrieved to be on the same sample time
 *
 * @param sensorData
 */
void MAG3110_Get3AxisData(MAG3110_Data* sensorData);

/**
 * Retrieves temperature data from the MAG3110
 *
 * @param sensorData
 */
void MAG3110_GetTempurature(MAG3110_Data* sensorData);

/**
 * Read the status register on the MAG3110 and checks to see if X axis data is ready
 *
 * @return True when X axis data is ready, false otherwise
 */
bool MAG3110_Is_X_Data_Ready(void);

/**
 * Read the status register on the MAG3110 and checks to see if Y axis data is ready
 *
 * @return True when Y axis data is ready, false otherwise
 */
bool MAG3110_Is_Y_Data_Ready(void);

/**
 * Read the status register on the MAG3110 and checks to see if Z axis data is ready
 *
 * @return True when Z axis data is ready, false otherwise
 */
bool MAG3110_Is_Z_Data_Ready(void);

/**
 * Read the status register on the MAG3110 and checks to see if any axis data is ready
 *
 * @return
 */
bool MAG3110_Is_ZYX_Data_Ready(void);

/**
 * Returns the full status register from the MAG3110
 *
 * @return The 8 bit value of the current status register
 */
uint8_t MAG3110_Get_DR_STATUS(void);

/**
 * This function will set user defined offset corrections for each X, Y, Z axis
 * on the MAG3110
 *
 * @param X_offset
 * @param Y_offset
 * @param Z_offset
 */
void MAG3110_Set_Offest_Correction(uint16_t X_offset, uint16_t Y_offset, uint16_t Z_offset);

/**
 *
 * @param reg_value
 */
void MAG3110_Set_CTRL_REG1(unsigned char reg_value);

/**
 *
 * @param reg_value
 */
void MAG3110_Set_CTRL_REG2(unsigned char reg_value);

/**
 *
 * @param enabled
 */
void MAG3110_SetActive(bool enabled);

/**
 *
 * @param enabled
 */
void MAG3110_SetRaw(bool enabled);

#endif	/* MAG3110_H */

