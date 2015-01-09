/**
 * An I2C library for the dsPIC33 series processors.
 * Copyright (C) 2015 Pavlo Manovi

 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef I2C_DSPIC_H
#define I2C_DSPIC_H

#include <stdint.h>

// Set the expected delay on the i2c lines (seconds).
// See Equation 4.1 in the I2C documentation.
#define I2C_DELAY 120e-9

/**
 * Calculates the BRG register value for the I2C peripheral given the desired
 * operating frequency and desired peripheral frequency.
 * 
 * @param frequency The desired frequency.
 * @param f_pb The peripheral bus frequency (or system frequency if there is no peripheral bus)
 */
#define I2C_CALC_BRG(frequency, f_pb) ((uint16_t)((((1.0 / (frequency)) - I2C_DELAY) * (f_pb)) - 2.0))

/**
 * Initialize the I2C peripheral. This does not handle pin mappings.
 *
 * This cannot be used to re-initalize the I2C peripheral
 *
 * @param brg The BRG register. @see I2C_CALC_BRG()
 */
void I2C_Init(uint16_t brg);

/**
 * Write data to an I2C peripheral register.
 *
 * @param address The peripheral address to communicate with.
 * @param deviceRegister The register to write the data to.
 * @param data The data to send.
 */
void I2C_WriteToReg(uint8_t address, uint8_t deviceRegister, uint8_t data);

/**
 * Read an I2C register from a peripheral.
 *
 * @param address The peripheral address.
 * @param deviceRegister The register address to read from.
 * @return The data requested.
 */
uint8_t I2C_ReadFromReg(uint8_t address, uint8_t deviceRegister);

/**
 * Reads data to an I2C peripheral register, supports bursting
 * To read a single register, set burstNum to 1
 *
 * @param address The peripheral address to communicate with.
 * @param deviceRegister The register to write the data to.
 * @param data An array into which the function places the read data
 * @param burstNum Number of registers to read
 */
void I2C_ReadFromReg_Burst(uint8_t address, uint8_t deviceRegister, uint8_t* data, uint8_t burstNum);

#endif
