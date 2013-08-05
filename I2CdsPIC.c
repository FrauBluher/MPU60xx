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


#include <xc.h>
#include <i2c.h>
#include "I2CdsPIC.h"

static uint8_t I2C_Init = 0;

uint16_t I2C_Init(uint16_t baudrate) {
    uint16_t actualRate = 0;

    if (I2C_Init != 1) {
        I2C1CONbits.I2CEN = 0; //Disable I2C1
        I2C1BRG = (F_PB / (2 * baudrate)) - 2; //Set BRG based off of baudrate
        actualRate = (F_PB) / ((I2C1BRG + 2)*2); //Compute actual baudrate
        I2C1CONbits.I2CEN = 1; //Enable I2C1
        I2C_Init = 1;
    }

    return (actualRate);
}

uint8_t I2C_WriteToReg(uint8_t I2CAddress, uint8_t deviceRegister, uint8_t data) {
    I2C1CONbits.SEN = 1;
    while (I2C1CONbits.SEN == 1);

    I2C1TRN = (I2CAddress << 1);
    while (I2C1STATbits.TRSTAT != 0);
    if (I2C1STATbits.ACKSTAT == 1) {
        //NACK CONDITION HERE
    }

    I2C1TRN = deviceRegister;
    while (I2C1STATbits.TRSTAT != 0);
    if (I2C1STATbits.ACKSTAT == 1) {
        //NACK CONDITION HERE
    }

    I2C1TRN = data;
    while (I2C1STATbits.TRSTAT != 0);
    if (I2C1STATbits.ACKSTAT == 1) {
        //NACK CONDITION HERE
    }

    I2C1CONbits.PEN = 1;
    while (I2C1CONbits.PEN == 1);

    return 1;
}


uint8_t I2C_ReadFromReg(uint8_t I2CAddress, uint8_t deviceRegister) {
    uint8_t data = 0;
    I2C1CONbits.SEN = 1;
    while (I2C1CONbits.SEN == 1);

    I2C1TRN = I2CAddress << 1;
    while (I2C1STATbits.TRSTAT != 0);
    if (I2C1STATbits.ACKSTAT == 1) {
        //NACK CONDITION HERE
    }

    I2C1TRN = deviceRegister;
    while (I2C1STATbits.TRSTAT != 0);
    if (I2C1STATbits.ACKSTAT == 1) {
        //NACK CONDITION HERE
    }

    I2C1CONbits.RSEN = 1;
    while (I2C1CONbits.RSEN == 1);

    I2C1TRN = (I2CAddress << 1) + 1;
    while (I2C1STATbits.TRSTAT != 0);
    if (I2C1STATbits.ACKSTAT == 1) {
        //NACK CONDITION HERE
    }

    I2C1CONbits.RCEN = 1;
    while (I2C1STATbits.RBF != 1);

    data = I2C1RCV;
    I2C1CONbits.PEN = 1;
    while (I2C1CONbits.PEN == 1);
    return(data);
}