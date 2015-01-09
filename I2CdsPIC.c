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
	uint8_t tempData;

	I2C_ReadFromReg_Burst(address, deviceRegister, &tempData, 1);
	return tempData;
}

void I2C_ReadFromReg_Burst(uint8_t address, uint8_t deviceRegister, uint8_t* data, uint8_t burstNum)
{
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
                uint8_t i;
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
}
