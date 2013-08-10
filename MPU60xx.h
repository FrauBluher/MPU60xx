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


#ifndef _MPU60xx_H_
#define _MPU60xx_H_

#include <stdint.h>

typedef struct {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t temperature;
    uint8_t newData;
} MPU6050_Data;

/**
 * @brief Address registers set by the address pin on the MPU6050, either high or low.
 */
#define ADDRESS_AD0_LOW     0x68
#define ADDRESS_AD0_HIGH    0x69
#define DEFAULT_ADDRESS     ADDRESS_AD0_LOW

/**
 * @brief Basic configuration registers for the MPU6050.
 */
#define RA_GYRO_CONFIG      0x1B
#define RA_ACCEL_CONFIG     0x1C
#define RA_ACCEL_XOUT_H     0x3B
#define RA_ACCEL_XOUT_L     0x3C
#define RA_ACCEL_YOUT_H     0x3D
#define RA_ACCEL_YOUT_L     0x3E
#define RA_ACCEL_ZOUT_H     0x3F
#define RA_ACCEL_ZOUT_L     0x40
#define RA_TEMP_OUT_H       0x41
#define RA_TEMP_OUT_L       0x42
#define RA_GYRO_XOUT_H      0x43
#define RA_GYRO_XOUT_L      0x44
#define RA_GYRO_YOUT_H      0x45
#define RA_GYRO_YOUT_L      0x46
#define RA_GYRO_ZOUT_H      0x47
#define RA_GYRO_ZOUT_L      0x48
#define RA_PWR_MGMT_1       0x6B
#define RA_PWR_MGMT_2       0x6C
#define RA_WHO_AM_I         0x75

/**
 * @brief Gyro range settings.
 * 0 = +/- 250 degrees/sec   131  LSB/degree/s
 * 1 = +/- 500 degrees/sec   65.5 LSB/degree/s
 * 2 = +/- 1000 degrees/sec  32.8 LSB/degree/s
 * 3 = +/- 2000 degrees/sec  16.4 LSB/degree/s
 */
#define GYRO_FS_250         0x00
#define GYRO_FS_500         0x01
#define GYRO_FS_1000        0x02
#define GYRO_FS_2000        0x03

/**
 * @brief Accelerometer range settings.
 * 0 = +/- 2g          16384 LSB/g
 * 1 = +/- 4g          8192  LSB/g
 * 2 = +/- 8g          4096  LSB/g
 * 3 = +/- 16g         2048  LSB/g
 */
#define ACCEL_FS_2          0x00
#define ACCEL_FS_4          0x01
#define ACCEL_FS_8          0x02
#define ACCEL_FS_16         0x03

/**
 * @brief PWR1 register bitfield.
 */
#define PWR1_DEVICE_RESET_BIT   7
#define PWR1_SLEEP_BIT          6
#define PWR1_CYCLE_BIT          5
#define PWR1_TEMP_DIS_BIT       3
#define PWR1_CLKSEL_BIT         2
#define PWR1_CLKSEL_LENGTH      3

/**
 * @brief Clock sources.
 */
#define CLOCK_INTERNAL          0x00
#define CLOCK_PLL_XGYRO         0x01
#define CLOCK_PLL_YGYRO         0x02
#define CLOCK_PLL_ZGYRO         0x03
#define CLOCK_PLL_EXT32K        0x04
#define CLOCK_PLL_EXT19M        0x05
#define CLOCK_KEEP_RESET        0x07

/**
 * @brief PWR1 register bitfield.
 */
#define PWR2_LP_WAKE_CTRL_BIT       7
#define PWR2_LP_WAKE_CTRL_LENGTH    2
#define PWR2_STBY_XA_BIT            5
#define PWR2_STBY_YA_BIT            4
#define PWR2_STBY_ZA_BIT            3
#define PWR2_STBY_XG_BIT            2
#define PWR2_STBY_YG_BIT            1
#define PWR2_STBY_ZG_BIT            0



/**
 * @brief Inits the MPU60xx for usage and sets the clock source to the X-Gyro
 */
void MPU60xx_Init();

/**
 * @brief Enable/Disable sleep mode.
 * @param enabled New sleep mode enabled status.
 */
void MPU60xx_Sleep(uint8_t enabled);

/**
 * @brief Sets accelerometer range and sensitivity.
 * @param range New full-scale gyroscope range setting
 * @see page 30 of RM-MPU-6000A.pdf
 */
void MPU60xx_SetGyroRange(uint8_t range);

/**
 * @brief Sets accelerometer range and sensitivity.
 * @param range New full-scale accelerometer range setting
 * @see page 32 of RM-MPU-6000A.pdf
 */
void MPU60xx_SetAccelRange(uint8_t range);

/**
 * @brief Gets Gyro and Accel readings.
 * @param *sensorData pointer to variable of type MPU60xx_Data for ouput.
 * @see page 30 and 32 of RM-MPU-6000A.pdf
 */
void MPU60xx_Get6AxisData(MPU6050_Data *sensorData);

/**
 * @brief Gets temperature readings from internal sensor.
 * @param *sensorData pointer to variable of type MPU6050_Data for ouput.
 *
 * Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
 * Scaling is up to whomever is implementing this API method.
 */
void MPU60xx_GetTemperature(MPU6050_Data *sensorData);

/**
 * @brief Enable internal temp sensor.
 * @param enabled Enable internal temperature sensor.
 *
 * 1 - Enabled  0 - Disabled
 */
void MPU60xx_SetTempSensorEnabled(uint8_t enabled);

/**
 * @brief Returns I2C address.
 * @return Device ID (0x68)
 */
uint8_t MPU60xx_GetDeviceID();

/**
 * @brief Chooses the clock source that the MPU-60xx will use.
 * @param source Clock source the MPU60xx will use.
 */
void MPU60xx_SetClockSource(uint8_t source);

#endif /* _MPU60xx_H_ */