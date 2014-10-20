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


void I2C_Init(uint16_t brg) {
    I2C1CONbits.I2CEN = 0; // Disable I2C1
    I2C1BRG = brg;         // Set the baud rate
    I2C1CONbits.I2CEN = 1; // Enable I2C1
}


void I2C_WriteToReg(uint8_t address, uint8_t deviceRegister, uint8_t data) {

    StartI2C1();
    while(I2C1CONbits.SEN);
    IFS1bits.MI2C1IF = 0;

    MasterWriteI2C1(address << 1);
    while(I2C1STATbits.TBF);   // 8 clock cycles
    while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
    IFS1bits.MI2C1IF = 0;     // Clear interrupt flag

    MasterWriteI2C1(deviceRegister);
    while(I2C1STATbits.TBF);   // 8 clock cycles
    while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
    IFS1bits.MI2C1IF = 0;     // Clear interrupt flag
    MasterWriteI2C1(data);

    while(I2C1STATbits.TBF);   // 8 clock cycles
    while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
    IFS1bits.MI2C1IF = 0;     // Clear interrupt flag

    StopI2C1();               // Write stop sequence.
    while(I2C1CONbits.PEN);
    IdleI2C1();
}


uint8_t I2C_ReadFromReg(uint8_t address, uint8_t deviceRegister) {
    uint8_t data = 0;
    StartI2C1();
    while(I2C1CONbits.SEN);

    MasterWriteI2C1(address << 1);
    while(I2C1STATbits.TBF);   // 8 clock cycles
    while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
    IFS1bits.MI2C1IF = 0;     // Clear interrupt flag

    MasterWriteI2C1(deviceRegister);
    while(I2C1STATbits.TBF);   // 8 clock cycles
    while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
    IFS1bits.MI2C1IF = 0;     // Clear interrupt flag

    RestartI2C1();            // Second start.
    while(I2C1CONbits.RSEN);

    MasterWriteI2C1((address << 1 )+ 1);
    while(I2C1STATbits.TBF);   // 8 clock cycles
    while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
    IFS1bits.MI2C1IF = 0;     // Clear interrupt flag

    data = MasterReadI2C1();

    NotAckI2C1();             // Read stop sequence.
    while(I2C1CONbits.ACKEN == 1);
    StopI2C1();
    while(I2C1CONbits.PEN);
    IdleI2C1();

    return(data);
}