/*
 * Main header file.
 * (c) Michal Petrilak, Jan Horacek 2016
 * Version: 1.0.1
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

#define DEFAULT_XPRESSNET_ADDR  29
#define XN_EEPROM_ADDR          0x00
    
#define VERSION_FW      0x11        // firmware version 1.0
#define VERSION_HW      0x20        // hardware version 2.0

#define FERR_FEATURE                // undef to disable FERR feature


#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

