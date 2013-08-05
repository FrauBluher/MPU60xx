/* 
 * File:   main.c
 * Author: Pavlo
 *
 * Created on August 3, 2013, 1:10 PM
 */


#include <xc.h>
#include "I2CdsPIC.h"

_FOSCSEL(FNOSC_FRC);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
_FWDT(FWDTEN_OFF);
_FICD(JTAGEN_OFF & ICS_PGD2);


// Instantiate Drive and Data objects
//START
//Write address (write)
//ACK (from slave)
//Write register (sub) address <-- THIS tells the slave which register to start the read from
//ACK (from slave)
//RESTART
//Write address (read)
//ACK (from slave)
//Read data
//ACK (from master)
//Read data
//ACK (from master)

int main(void) {
    PLLFBD = 38; // M = 40 MIPS
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 0; // N2 = 2
    OSCTUN = 0;
    RCONbits.SWDTEN = 0;

    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to Primary (3?)

    __builtin_write_OSCCONL(0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur

    while (OSCCONbits.LOCK != 1) {
    };
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;


    while (1) {

    };
    //In embedded environments you don't reach the end of main.
}