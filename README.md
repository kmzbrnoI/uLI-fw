# uLI
Ultimate LI.

uLI is XpressNET &#8596; USB converter behaving as COM port in PC (CDC).

* Processor: PIC18F14K50
* Programming language: C
* PCB: [uLI](https://github.com/kmzbrnoI/uLI-pcb)
* Authors: Jan Horacek, Michal Petrilak (c) 2016

## Used tools

- MPLAB X IDE v3.45
- C18 LITE v3.47 compiler (version for linux available
  [here](https://github.com/Manouchehri/Microchip-C18-Lite))
- clang-format to format code

## Windows driver

Windows CDC driver could be found in the
[Microchip Libraries for Applications](http://www.microchip.com/mplab/microchip-libraries-for-applications).

However you need to download ~ 280 MB of data to get ~ 10 kB driver so we added
driver to the [driver_win](driver_win/) directory of this repo.

## COM port specification

* Any speed.
* No flow control.

## EEPROM

PIC stores its xpressnet address at 0x00 in EEPROM. It is not necessary to
init EEPROM with valid address. When invalid address is loaded from EEPROM,
PIC sets its xpressnet address to `DEFAULT_XPRESSNET_ADDR` defined in `main.h`
(usually `25`) and saves this address to EEPROM.

## Programming

We used `PICPgm` at RaspberryPi to program the PIC.

Note: When programming the processor for first time, do not forget to include
`-p_cfg` argument to program fuses into the processor. Fuses are stored in main
hex file.

## LEDs

### Input LED (green)
This LED is turned on by default. It turns off for a few milliseconds when a
command arrives from a command station to uLI. This LED usually blinks as it
shows *normal inquiry* packets arriving. If this LED is permanently turned on,
the command station is not sending *normal inquiry* packets and something went
really wring.

### Output LED (green)
This LED is turned on before a valid connection with PC is established. After
establishing the connection, this LED turns off and blinks only when a command
is being sent from the uLI to the command station.

### Status LED (yellow)
- 1 blink = normal operations
- 2 blinks = framing errors on XpressNET
- 4 blinks = buffer half full
- 6 blinks = framing errors on XpressNET & buffer half full

## Timeouts

uLI has several types of timeouts. We believe one should know how these timeouts
work to communicate with uLI properly.

### USB input timeout
When only part of the message is received from PC, the next part must
follow in < 100 ms. Otherwise, the first part is removed from USB input buffer and
*Error occurred between the interfaces and the PC* message is transmitted to PC.

### XpressNET input timeout
When only part of the message is received from command station, the next part must
follow in < 20 ms. Otherwise, the first part is removed from buffer and
*Error occurred between the interfaces and the command station* message is
transmitted to PC.

### Timeslot timeouts
- The command station should send *normal inquiry* to each xpressnet device
  once a while. When uLI does not receive "normal inquiry" for **5 s** (this value is
  taken from LI specification) it sends *The Command Station is no longer providing
  the LI100 a timeslot for communication* to PC. At this point, USB → XpressNET
  buffer is cleared and no more data could be received from PC until command
  station sends *normal inquiry* to uLI.

  When computer tries to send data in such a situation, uLI responds
  *Buffer overflow in the LI100*.

- uLI also listens for *service mode entry* and *normal operations resumed*
  broadcasts. When the command station is in programming mode (e.g. *service
  mode entry* was received) the previous timeout is extended to **1:30** (1 minute,
  30 seconds). Tsis value is also taken from official LI specification.

  When the command station starts sending *normal inquiry* after 5 s < x < 1:30,
  the uLI sends to PC *normal operations resumed*. This mechanism is based
  on Lenz\` LI, which does the same thing. However, this feature is not documented
  in official Lenz` LI specification.

  uLI also sends *normal operations resumed* after receiving *normal inquiry*
  after sending some specific commands from PC to command station. E. g. if PC
  sends "read CV 4", the command station stops adressing uLI. After some time
  the command stations starts adressing uLI again (it sends *normal inqury*).
  At this point *normal operations resumed* is transferred to PC.

  *Normal opeartions resumed* is transmitted to PC always after the timeout
  overflowed and the command station starts adressing uLI again.

## Things good to know

Sometimes, the ultimate LI might behave weirdly from your point of view, so it
it is good to know why is is doing so.

- uLI DOES NOT guarantee to preserve order of messages for command station and
  uLI. E. g. when you send a mesage for command station and a message for uLI
  (e. g. determine uLI version), version response might be (and probably will be)
  the first response of uLI. Response from command station will follow.


## Further reading

- [About XpressNET](http://www.opendcc.de/info/xpressnet/xpressnet_e.html)
- uLI meets [XpressNET specification v3](http://www.lenzusa.com/1newsite1/Manuals/xpressnet.pdf)
- uLI should also meet [XpressNET specification v3.6](http://wiki.rocrail.net/lib/exe/fetch.php?id=xpressnet-en&cache=cache&media=xpressnet:xpressnet-lan-usb-23151-v1.pdf) (available only in German)
- uLI should meet official Lenz` [LI specification](http://www.lenzusa.com/1newsite1/Manuals/LI-USB_XpressNet_Supplement.pdf)
