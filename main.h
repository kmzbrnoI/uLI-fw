/*
 * Main header file.
 * (c) Michal Petrilak, Jan Horacek 2016
 * Version: 1.3
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

//#define DEBUG

#define DEFAULT_XPRESSNET_ADDR  29
#define XN_EEPROM_ADDR          0x00

#define VERSION_FW      0x13        // firmware version 1.3
#define VERSION_HW      0x20        // hardware version 2.0

#define FERR_FEATURE                // undef to disable FERR feature

typedef union {
    struct {
    	BOOL version :1;
        BOOL addr :1;
        BOOL baud_rate :1;
        BOOL ferr :1;
        BOOL full_buffer :1;
        BOOL xor_error :1;
        BOOL cs_timeout :1;
        BOOL ok :1;
        BOOL pc_timeout :1;
    } bits;
    WORD all;
} send_waiting;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */

