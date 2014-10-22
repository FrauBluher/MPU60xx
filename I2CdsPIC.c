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
#include <stdbool.h>

// Microchip headers
#include <xc.h>
#include <i2c.h>

// User headers
#include "I2CdsPIC.h"

void I2C_Init(uint16_t brg)
{
	I2C1CONbits.I2CEN = 0; // Disable I2C1
	I2C1BRG = brg + 1; // Set the baud rate
	I2C1CONbits.I2CEN = 1; // Enable I2C1
}

void I2C_WriteToReg(uint8_t address, uint8_t deviceRegister, uint8_t data)
{

	// Assert the start condition
	StartI2C1();
	while (I2C1CONbits.SEN);
	IFS1bits.MI2C1IF = 0;

	// Send the 7-bit I2C device address
	MasterWriteI2C1(address << 1);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Send the register address
	MasterWriteI2C1(deviceRegister);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Send the data
	MasterWriteI2C1(data);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Assert the stop condition
	StopI2C1();
	while (I2C1CONbits.PEN);

	// Sit idle on the bus
	IdleI2C1();
}

uint8_t I2C_ReadFromReg(uint8_t address, uint8_t deviceRegister)
{
	uint8_t tempData[0] = {0};

	I2C_ReadFromReg_Burst(address, deviceRegister, &tempData, 1);
	return tempData[0];
}

void I2C_ReadFromReg_Burst(uint8_t address, uint8_t deviceRegister, uint8_t* data, uint8_t burstNum)
{
	// Used for the Burst for loop
	uint8_t i;

	// Assert the start condition
	StartI2C1();
	while (I2C1CONbits.SEN);

	// Send the 7-bit device address
	MasterWriteI2C1(address << 1);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Send the register address
	MasterWriteI2C1(deviceRegister);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Start a new I2C transaction
	RestartI2C1();
	while (I2C1CONbits.RSEN);

	// Send the second address
	MasterWriteI2C1((address << 1) + 1);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Read the data
	data[0] = MasterReadI2C1();

	if (burstNum > 1) {
		for (i = 1; i < burstNum; i++) {
			AckI2C1();
			while (I2C1CONbits.ACKEN == 1);

			data[i] = MasterReadI2C1();
		}
	}

	// No need to ack reception of this data
	NotAckI2C1();
	while (I2C1CONbits.ACKEN == 1);

	// Stop the I2C transaction
	StopI2C1();
	while (I2C1CONbits.PEN);

	// Go idle on the bus
	IdleI2C1();

	return(data);
}