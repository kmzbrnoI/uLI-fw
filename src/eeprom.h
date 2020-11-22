/* EEPROM access header file. */

#ifndef EEPROM_H
#define EEPROM_H

#include <inttypes.h>

unsigned char ReadEEPROM(uint8_t address);
void WriteEEPROM(uint8_t address, uint8_t data);

#endif /* EEPROM_H */
