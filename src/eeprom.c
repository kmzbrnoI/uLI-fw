/* EEPROM access implementation */

#include <xc.h>
#include "eeprom.h"

unsigned char ReadEEPROM(uint8_t address) {
	EECON1 = 0; //ensure CFGS=0 and EEPGD=0
	EEADR = address;
	EECON1bits.RD = 1;
	return EEDATA;
}

void WriteEEPROM(uint8_t address, uint8_t data) {
	char SaveInt;
	SaveInt = INTCON;           //save interrupt status
	EECON1 = 0;                 //ensure CFGS=0 and EEPGD=0
	EECON1bits.WREN = 1;        //enable write to EEPROM
	EEADR = address;            //setup Address
	EEDATA = data;              //and data
	INTCONbits.GIE = 0;         //No interrupts
	EECON2 = 0x55;              //required sequence #1
	EECON2 = 0xaa;              //#2
	EECON1bits.WR = 1;          //#3 = actual write
	INTCON = SaveInt;           //restore interrupts
	while (!PIR2bits.EEIF);     //wait until finished
	EECON1bits.WREN = 0;        //disable write to EEPROM
}
