/* Main header file. */

#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>

#define DEFAULT_XPRESSNET_ADDR  29
#define XN_EEPROM_ADDR          0x00

#define VERSION_FW      0x23        // firmware version 2.3
#define VERSION_HW      0x20        // hardware version 2.0

#define FERR_FEATURE                // undef to disable FERR feature

typedef union {
	struct {
		bool version : 1;
		bool addr : 1;
		bool baud_rate : 1;
		bool ferr : 1;
		bool full_buffer : 1;
		bool xor_error : 1;
		bool cs_timeout : 1;
		bool ok : 1;
		bool pc_timeout : 1;
		bool timeslot_timeout : 1;
	} bits;
	uint16_t all;
} send_waiting;

#endif /* MAIN_H */
