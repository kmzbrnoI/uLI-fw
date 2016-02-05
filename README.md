# uLI
Ultimate LI.

uLI is XpressNET &#8596; USB convertor behaving as COM port in PC.

* Used processor: PIC18F14K50
* PCB: available soon
* Authors: Jan Horacek, Michal Petrilak (c) 2016

## EEPROM

PIC stores its xpressnet address at 0x00 in EEPROM. It is not ncecessary to
init EEPROM with valid address. When invalid address is loaded from EEPROM,
PIC sets its xpressnet address to `DEFAULT_XPRESSNET_ADDR` defined in `main.h`
(usually `25`) and saves this address to EEPROM.
