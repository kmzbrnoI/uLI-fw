# uLI
Ultimate LI.

uLI is XpressNET &#8596; USB convertor behaving as COM port in PC (CDC).

* Processor: PIC18F14K50
* Programming language: C
* PCB: uLI-02
* Authors: Jan Horacek, Michal Petrilak (c) 2016

## Used tools

- MPLAB X IDE v3.15 for Windows
- C18 LITE v3.47 compiler for Windows

## EEPROM

PIC stores its xpressnet address at 0x00 in EEPROM. It is not ncecessary to
init EEPROM with valid address. When invalid address is loaded from EEPROM,
PIC sets its xpressnet address to `DEFAULT_XPRESSNET_ADDR` defined in `main.h`
(usually `25`) and saves this address to EEPROM.

## Programming

We used `PICPgm` at RaspberryPi to program the PIC.

Note: When programming the processor for first time, do not forget to include
`-p_cfg` argument to program fuses into the processor. Fuses are stored in main
hex file.

## Further reading

- [About XpressNET](http://www.opendcc.de/info/xpressnet/xpressnet_e.html)
- uLI meets [XpressNET specification v3](http://www.lenzusa.com/1newsite1/Manuals/xpressnet.pdf)
- uLI should also meet [XpressNET specification v3.6](http://wiki.rocrail.net/lib/exe/fetch.php?id=xpressnet-en&cache=cache&media=xpressnet:xpressnet-lan-usb-23151-v1.pdf) (available only in German)
