/* EEPROM access header file. */

#ifndef EEPROM_H
#define EEPROM_H

unsigned char ReadEEPROM(unsigned char address);
void WriteEEPROM(unsigned char address, unsigned char data);

#endif /* EEPROM_H */
