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


#ifndef I2C_DSPIC_H
#define I2C_DSPIC_H

#include <stdint.h>

// Set the expected delay on the i2c lines (seconds).
// See Equation 4.1 in the I2C documentation.
#define I2C_DELAY 120e-9

/**
 * Calculates the BRG register value for the I2C peripheral given the desired
 * operating frequency and baud rate.
 * 
 * @param baudrate The desired baud rate.
 * @param f_pb The peripheral bus frequency (or system frequency if there is no peripheral bus)
 */
#define I2C_CALC_BRG(baudrate, f_pb) ((uint16_t)((((1.0 / (baudrate)) - I2C_DELAY) * (f_pb)) - 2.0))

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

#endif