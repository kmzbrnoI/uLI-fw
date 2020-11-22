/* Main functionality of whole project */

/** INCLUDES ******************************************************************/

#include <xc.h>
#include <stdbool.h>
#include <inttypes.h>

#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "eeprom.h"
#include "config.h"
#include "main.h"
#include "ringBuffer.h"
#include "usart.h"
#include "usb_stack/usb.h"
#include "usb_stack/usb_config.h"
#include "usb_stack/usb_device.h"
#include "usb_stack/usb_device_cdc.h"

/** DEFINES *******************************************************************/

#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define MAX(a,b) (((a) > (b)) ? (a) : (b))

// len WITH header byte and WITH xor byte
#define USB_msg_len(start)      ((ring_USB_datain.data[start] & 0x0F) + 2)
#define USB_last_message_len()  ringDistance(ring_USB_datain, last_start, ring_USB_datain.ptr_e)

// length of xpressnet message is 4-lower bits in second byte
#define USART_msg_len(start) \
	((ring_USART_datain.data[(start + 1) & ring_USART_datain.max] & 0x0F)+3)
#define USART_last_message_len \
	ringDistance(ring_USART_datain, USART_last_start, ring_USART_datain.ptr_e)
#define USART_msg_to_send \
	(ringLength(ring_USB_datain) >= MAX(2, USB_msg_len(ring_USB_datain.ptr_b)))

#define USB_MAX_TIMEOUT                   10        // 100 ms
#define USART_MAX_TIMEOUT                  2        // 20 ms
#define TIMESLOT_MAX_TIMEOUT             500        // 5 s (according to specification)
#define TIMESLOT_LONG_MAX_TIMEOUT       9000        // 1:30 min (according to specification)
#define FERR_TIMEOUT                    1000        // 10 s

#define MLED_XN_MAX_TIMEOUT                5        // 50 ms
#define MLED_DATA_MAX_TIMEOUT              7        // 70 ms

#define PWR_LED_SHORT_COUNT               15        // 150 ms
#define PWR_LED_LONG_COUNT                40        // 400 ms
#define PWR_LED_FERR_COUNT                10        // status led indicates >10 framing errors

/** VARIABLES *****************************************************************/

volatile ring_generic ring_USB_datain;
volatile ring_generic ring_USART_datain;
volatile uint8_t tmp_baud_rate;

volatile uint8_t our_frame = 0;         // 0 = we cannot send messages
                                        // 1..80 = we can send messages
volatile uint8_t usb_timeout = 0;       // increment every 10 ms -> 100 ms timeout = 10
volatile uint8_t usart_timeout = 0;     // increment every 10 ms -> 20 ms timeout = 2
volatile uint8_t usart_to_send = 0;     // byte to send to USART
                                        // I made this public volatile variable,
                                        // beacause it is accessed in interrupts and in main too.

// timeslot errors
volatile uint16_t timeslot_timeout = 0; // timeslot timeout
volatile bool timeslot_err = true;      // true if timeslot error

// XpressNET framing error counting
#ifdef FERR_FEATURE
volatile uint32_t ferr_in_10_s = 0;     // number of framing errors in 10 seconds
volatile uint32_t ferr_counter = 0;
#endif

volatile uint8_t mLED_XN_Timeout = 2 * MLED_XN_MAX_TIMEOUT;
volatile uint8_t mLED_Data_Timeout = 2 * MLED_DATA_MAX_TIMEOUT;
volatile bool usart_longer_timeout = false;
volatile uint8_t xn_addr = DEFAULT_XPRESSNET_ADDR;
volatile bool force_ok_response = false;

// Power led blinks pwr_led_status times, then stays blank for some time
//  and then repeats the whole cycle. This lets user to see software status.
volatile uint8_t pwr_led_base_timeout = PWR_LED_SHORT_COUNT;
volatile uint8_t pwr_led_base_counter = 0;
volatile uint8_t pwr_led_status_counter = 0;
volatile uint8_t pwr_led_status = 1;

volatile bool programming_mode = false;

volatile uint8_t USART_last_start = 0;

// messages waiting to be sent to PC
volatile send_waiting pc_send_waiting = { 0 };

/* This lock disables sending of the last message in ring_USB_datain buffer to
 * the command station.
 * Currently, this lock disables start of transmission of all messages in buffer,
 * if you feel like having problems with performance, you could make this lock
 * actually lock only the last message in the buffer.
 */
volatile bool ring_USB_datain_backlocked = false;

uint8_t buf[32]; // any-purpose buffer

/** PRIVATE  PROTOTYPES *******************************************************/

void init(void);
void init_EEPROM(void);

void USB_send(void);
void USB_receive(void);
bool USB_parse_data(uint8_t start, uint8_t len);

void USART_receive_interrupt(void);
void USART_send(void);
void USART_check_timeouts(void);

void dump_buf_to_USB(ring_generic* buf);
void check_response_to_PC(uint8_t header, uint8_t id);
void check_device_data_to_USB(void);

void check_XN_timeout_supress(uint8_t ring_USB_msg_start);
void check_pwr_LED_status(void);
void check_broadcast(uint8_t xn_start_index);

void timer_10ms(void);

/** INTERRUPTS ****************************************************************/

void __interrupt(high_priority) high_isr(void) {
	USBDeviceTasks();

	// USART send interrupt
	if ((PIE1bits.TXIE) && (PIR1bits.TXIF))
		USART_send();

	// USART receive interrupt
	if (PIR1bits.RCIF)
		USART_receive_interrupt();
}

void __interrupt(low_priority) low_isr(void) {
	static volatile uint8_t ten_ms_counter = 0;

	if ((PIE1bits.TMR2IE) && (PIR1bits.TMR2IF)) {
		// Timer2 on 100 us
		if (ten_ms_counter < 100) {
			ten_ms_counter++;
		} else {
			ten_ms_counter = 0;
			timer_10ms();
		}
		PIR1bits.TMR2IF = 0; // reset overflow flag
	}
}

/** FUNCTIONS *****************************************************************/

void main(void) {
	init();
	USBDeviceAttach();

	while (true) {
		USART_check_timeouts();

		USB_receive();
		USB_send();

		check_pwr_LED_status();
		CDCTxService();

		ClrWdt(); // clear watchdog timer
	}
}

void init(void) {
#if (defined(__18CXX) & !defined(PIC18F87J50_PIM))
	ADCON1 |= 0x0F; // Default all pins to digital
#endif

#if defined(USE_USB_BUS_SENSE_IO)
	tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

#if defined(USE_SELF_POWER_SENSE_IO)
	tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif

	// init ring buffers
	ringBufferInit(ring_USB_datain, 32);
	ringBufferInit(ring_USART_datain, 32);

	// switch off AD convertors (USART is not working when not switched off manually)
	ANSEL = 0x00;
	ANSELH = 0x00;

	// Initialize all of the LED pins
	mInitAllLEDs();
	mLED_XN_On();
	mLED_Pwr_Off();
	mLED_Data_On();

	// setup timer2 on 100 us
	T2CONbits.T2CKPS = 0b11;    // timer2 prescaler 16x
	PR2 = 75;                   // timer2 setup period register to 100 us
	TMR2 = 0x00;                // timer2 reset counter
	PIR1bits.TMR2IF = 0;        // timer2 reset overflow flag
	PIE1bits.TMR2IE = 1;        // timer2 enable interrupt
	IPR1bits.TMR2IP = 0;        // timer2 interrupt low level
	INTCONbits.PEIE = 1;        // Enable peripheral interrupts (for usart)
	T2CONbits.TMR2ON = 1;       // timer2 enable

	init_EEPROM();
	USBDeviceInit();
	USARTInit();

	INTCONbits.GIEL = 1;        // Enable low-level interrupts
	INTCONbits.GIEH = 1;        // Enable high-level interrupts
	RCONbits.IPEN = 1;          // enable all interrupts
}

void timer_10ms(void) {
	if (usb_timeout < USB_MAX_TIMEOUT)
		usb_timeout++;

	if (usart_timeout < USART_MAX_TIMEOUT)
		usart_timeout++;

	if (timeslot_timeout < TIMESLOT_LONG_MAX_TIMEOUT)
		timeslot_timeout++;

#ifndef DEBUG
	// mLEDout timeout
	if (mLED_Data_Timeout < 2 * MLED_DATA_MAX_TIMEOUT) {
		mLED_Data_Timeout++;
		if (mLED_Data_Timeout == MLED_DATA_MAX_TIMEOUT) {
			mLED_Data_Off();
		}
	}
#endif

#ifdef FERR_FEATURE
	// framing error counting
	ferr_counter++;
	if (ferr_counter >= FERR_TIMEOUT) {
		ferr_in_10_s = 0;
		ferr_counter = 0;
	}
#endif

#ifndef DEBUG
	// mLEDIn timeout
	if (mLED_XN_Timeout < 2 * MLED_XN_MAX_TIMEOUT) {
		mLED_XN_Timeout++;
		if (mLED_XN_Timeout == MLED_XN_MAX_TIMEOUT) {
			mLED_XN_On();
		}
	}
#endif

	// pwrLED toggling
	pwr_led_base_counter++;
	if (pwr_led_base_counter >= pwr_led_base_timeout) {
		pwr_led_base_counter = 0;
		pwr_led_status_counter++;

		if (pwr_led_status_counter == 2 * pwr_led_status) {
			// wait between cycles
			pwr_led_base_timeout = PWR_LED_LONG_COUNT;
			mLED_Pwr_Off();
		} else if (pwr_led_status_counter > 2 * pwr_led_status) {
			// new base cycle
			pwr_led_base_timeout = PWR_LED_SHORT_COUNT;
			pwr_led_status_counter = 0;
			mLED_Pwr_On();
		} else {
			mLED_Pwr_Toggle();
		}
	}
}

// ************** USB Callback Functions **************************************

bool USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, uint16_t size) {
	USBCDCEventHandler(event, pdata, size);

	switch( (int) event )
	{
		case EVENT_TRANSFER:
			break;

		case EVENT_SOF:
			break;

		case EVENT_SUSPEND:
			mLED_Data_On();
			ringClear(&ring_USART_datain);
			ringClear(&ring_USB_datain);
			break;

		case EVENT_RESUME:
			mLED_Data_Off();
			break;

		case EVENT_CONFIGURED:
			CDCInitEP();
			mLED_Data_Off();
			break;

		case EVENT_SET_DESCRIPTOR:
			break;

		case EVENT_EP0_REQUEST:
			USBCheckCDCRequest();
			break;

		case EVENT_BUS_ERROR:
			break;

		case EVENT_TRANSFER_TERMINATED:
			break;

		default:
			break;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/* How does USART receiving work:
 * By default, USART waits for normal inquiry. When data are sent do uLI,
 * receive interrupt is fired and data are received in USART_receive_interrupt.
 * This function should be very fast, its main purpose is to start transmission
 * from uLI to command station very fast (< 80 us). This turned out to be quite
 * big problem.
 */

/* CHECKING USART TIMEOUTS
 * This function can be called from main, it does not require specific timing.
 */
void USART_check_timeouts(void) {
	// check for (short) timeout
	if ((USART_last_start != ring_USART_datain.ptr_e) && (usart_timeout >= USART_MAX_TIMEOUT)) {
		// delete last incoming message and wait for next message
		ring_USART_datain.ptr_e = USART_last_start;
		if (ring_USART_datain.ptr_e == ring_USART_datain.ptr_b)
			ring_USART_datain.empty = true;
		usart_timeout = 0;
		RCSTAbits.ADDEN = 1; // receive just first message

		// inform PC about timeout
		if (USBGetDeviceState() == CONFIGURED_STATE)
			pc_send_waiting.bits.cs_timeout = true;

		return;
	}

	// check timeslot_timeout
	// shorter timeout is default, longer timeout is for programming commands
	if ((((timeslot_timeout >= TIMESLOT_MAX_TIMEOUT) && (!usart_longer_timeout)) ||
	     (timeslot_timeout >= TIMESLOT_LONG_MAX_TIMEOUT)) && (!timeslot_err)) {
		// Command station is no longer providing timeslot for communication.
		timeslot_err = true;

		// remove buffers
		ringClear(&ring_USART_datain);
		ringClear(&ring_USB_datain);

		// send info to PC
		if (USBGetDeviceState() == CONFIGURED_STATE)
			pc_send_waiting.bits.timeslot_timeout = true;
	}
}

/* RECEIVING A uint8_t FROM USART
 * This function must be as fast as possible!
 */
void USART_receive_interrupt(void) {
	bool parity;
	static volatile uint8_t xor = 0;
	uint8_t tmp;
	nine_data USART_received;

	USART_received = USARTReadByte();

	usart_timeout = 0;

#ifdef FERR_FEATURE
	// increment framing eror counter in case of framing error
	ferr_counter += USART_received.FERR;
#endif

	if (USART_received.ninth) {
		// 9 bit is 1 -> header byte
		// we are waiting for call byte with our address

		tmp = (USART_received.data & 0x1F);
		if ((tmp != xn_addr) && (tmp != 0)) return;

		// new message for us -> check for parity
		if ((tmp = USART_received.data) & 1) parity = !parity;
		if ((tmp = tmp >> 1) & 1) parity = !parity;
		if ((tmp = tmp >> 1) & 1) parity = !parity;
		if ((tmp = tmp >> 1) & 1) parity = !parity;
		if ((tmp = tmp >> 1) & 1) parity = !parity;
		if ((tmp = tmp >> 1) & 1) parity = !parity;
		if ((tmp = tmp >> 1) & 1) parity = !parity;
		if ((tmp = tmp >> 1) & 1) parity = !parity;
		if (parity != 0) return;

		if (((USART_received.data >> 5) & 0b11) == 0b10) {
			// normal inquiry

			// first thing to do: send data as soon as possible! (we have only 80 us)
			if ((USART_msg_to_send) && (!ring_USB_datain_backlocked)) {
				XPRESSNET_DIR = XPRESSNET_OUT;
				USART_send();
			}

			timeslot_err = false;
			usart_longer_timeout = programming_mode;
			if ((timeslot_timeout >= TIMESLOT_MAX_TIMEOUT) || (force_ok_response)) {
				// ok response must be sent always after short timeout
				if (force_ok_response) force_ok_response = false;
				if (USBGetDeviceState() == CONFIGURED_STATE)
					pc_send_waiting.bits.ok = true;
			}
			timeslot_timeout = 0;

			if (USART_msg_to_send)
				check_XN_timeout_supress(ring_USB_datain.ptr_b);

		} else if (((USART_received.data >> 5) & 0b11) == 0b00) {
			// request acknowledgement
			// send Acknowledgement Response to command station (this should be done by LI)

			if (ringFreeSpace(ring_USB_datain) < 2) {
				// This situation should not happen. 2 bytes in ring_USB_datain
				// are always reserved for acknowledgement response.
				ringClear(&ring_USB_datain);
			}

			// add ACK to beginning of the buffer
			/* This code must not interfere with any other ring_USB_datain operation
			 * possibly called outside of interrupt!
			 */
			ring_USB_datain.ptr_b = (ring_USB_datain.ptr_b - 2) & ring_USB_datain.max;
			ring_USB_datain.empty = false;
			ring_USB_datain.data[ring_USB_datain.ptr_b] = 0x20;
			ring_USB_datain.data[(ring_USB_datain.ptr_b + 1) & ring_USB_datain.max] = 0x20;
			usart_to_send = ring_USB_datain.ptr_b;

			// send ACK to command station
			XPRESSNET_DIR = XPRESSNET_OUT;
			USART_send();

		} else {
			// normal message

			if (ring_USART_datain.ptr_e != USART_last_start) {
				// beginning of new message received before previous mesage was completely received -> delete previous message
				ring_USART_datain.ptr_e = USART_last_start;
				if (ring_USART_datain.ptr_e == ring_USART_datain.ptr_b)
					ring_USART_datain.empty = true;

				// send info to PC
				if (USBGetDeviceState() == CONFIGURED_STATE)
					pc_send_waiting.bits.cs_timeout = true;
			}

			// start of message for us (or broadcast)

			if (USBGetDeviceState() != CONFIGURED_STATE) return;

			// -> check space in buffer
			if (ringFull(ring_USART_datain)) {
				// buffer full -> probably no space to send data to PC
				// -> do not send information message to PC
				return;
			}

			RCSTAbits.ADDEN = 0; // receive all messages
			USART_last_start = ring_USART_datain.ptr_e;
			xor = 0;
			ringAddByte(&ring_USART_datain, USART_received.data);

#ifndef DEBUG
			if (mLED_Data_Timeout >= 2 * MLED_DATA_MAX_TIMEOUT) {
				mLED_Data_On();
				mLED_Data_Timeout = 0;
			}
#endif
		}
	} else {

		// ninth bit is 0
		// this situation happens only if message is for us (guaranteed by RCSTAbits.ADDEN)
		if (ringFull(ring_USART_datain)) {
			// reset buffer and wait for next message
			ring_USART_datain.ptr_e = USART_last_start;
			RCSTAbits.ADDEN = 1;
			if (ring_USART_datain.ptr_e == ring_USART_datain.ptr_b)
				ring_USART_datain.empty = true;

			// inform PC about buffer overflow
			pc_send_waiting.bits.full_buffer = true;

			return;
		}
		ringAddByte(&ring_USART_datain, USART_received.data);
		xor ^= USART_received.data;

		if (USART_last_message_len >= USART_msg_len(USART_last_start)) {
			// whole message received

			if (xor != 0) {
				// xor error
				ring_USART_datain.ptr_e = USART_last_start;
				if (ring_USART_datain.ptr_e == ring_USART_datain.ptr_b)
					ring_USART_datain.empty = true;
				pc_send_waiting.bits.xor_error = true;
			} else {
				// xor ok
				check_broadcast(USART_last_start);
				USART_last_start = ring_USART_datain.ptr_e; // whole message succesfully received
			}

			// listen for beginning of message (9. bit == 1)
			RCSTAbits.ADDEN = 1;
		}
	}

#ifndef DEBUG
	// toggle LED
	if (mLED_XN_Timeout >= 2 * MLED_XN_MAX_TIMEOUT) {
		mLED_XN_Off();
		mLED_XN_Timeout = 0;
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Check for data in ring_USART_datain and send complete data to USB.

void USB_send(void) {
	if (pc_send_waiting.all > 0)
		check_device_data_to_USB();

	if (!mUSBUSARTIsTxTrfReady()) return;

	if (((ringLength(ring_USART_datain)) >= 1) &&
	    (ringLength(ring_USART_datain) >= USART_msg_len(ring_USART_datain.ptr_b))) {
		// send message
		ringSerialize(&ring_USART_datain, buf, ring_USART_datain.ptr_b,
		              USART_msg_len(ring_USART_datain.ptr_b));
		putUSBUSART((uint8_t*)(buf + 1), ((buf[1]) & 0x0F) + 2);
		ringRemoveFrame(&ring_USART_datain, ((buf[1]) & 0x0F) + 3);
	}
}

////////////////////////////////////////////////////////////////////////////////
/* Receive data from USB and add it to ring_USB_datain.
 * Index of start of last message is in
 * WARNING! This function cannot rely on ring_USB_datain.ptr_b value!
 * ring_USB_datain.ptr_b could be changed in interrupt called at any time!
 * More specifically, USART_receive_interrupt could add data at beginning of
 * the USB buffer. It is necessarry to keep this information always in mind.
 */

void USB_receive(void) {
	static uint8_t last_start = 0;

	if ((USBDeviceState != CONFIGURED_STATE) || (USBIsDeviceSuspended())) return;

	// ring_USB_datain overflow check
	// 2 bytes in the buffer are always reserved for the Acknowledgement
	// response.
	if (ringFreeSpace(ring_USB_datain) < 2) {
		// delete last message
		ring_USB_datain.ptr_e = last_start;
		if (ring_USB_datain.ptr_b == ring_USB_datain.ptr_e)
			ring_USART_datain.empty = true;

		// inform PC about full buffer
		pc_send_waiting.bits.full_buffer = true;

		return;
	}

	/* +++ "Lock" ring_USB_datain +++
	 * End of USB ring buffer must be locked to prevent USART_RX_INTERRUPT to
	 * send message, which is not intended for command station, but for LI.
	 */
	ring_USB_datain_backlocked = true;

	uint8_t received_len = getsUSBUSART(&ring_USB_datain, ringFreeSpace(ring_USB_datain));

	if (received_len == 0) {
		ring_USB_datain_backlocked = false;
		// check for timeout
		if ((usb_timeout >= USB_MAX_TIMEOUT) && (last_start != ring_USB_datain.ptr_e)) {
			ring_USB_datain.ptr_e = last_start;
			usb_timeout = 0;
			if (ring_USB_datain.ptr_e == ring_USB_datain.ptr_b)
				ring_USB_datain.empty = true;

			// inform PC about timeout
			pc_send_waiting.bits.pc_timeout = true;
		}
		return;
	}

	usb_timeout = 0;

	// data received -> parse data
	while ((USB_last_message_len() > 0) && (USB_last_message_len() >= USB_msg_len(last_start))) {
		// whole message received -> check for xor
		uint8_t xor = 0;
		for (uint8_t i = 0; i < USB_msg_len(last_start); i++)
			xor ^= ring_USB_datain.data[(i + last_start) & ring_USB_datain.max];

		if (xor != 0) {
			// xor error
			// here, we need to delete content in the middle of ring buffer
			ringRemoveFromMiddle(&ring_USB_datain, last_start, USB_msg_len(last_start));
			pc_send_waiting.bits.xor_error = true;
		} else {
			// xor ok -> parse data
			if (USB_parse_data(last_start, USB_msg_len(last_start))) {
				// timeslot not available -> respond "Buffer full"
				if (timeslot_err) {
					ringRemoveFromMiddle(&ring_USB_datain, last_start, USB_msg_len(last_start));
					pc_send_waiting.bits.timeslot_timeout = true;
				} else {
					// data waiting for sending to xpressnet -> move last_start
					last_start = (last_start + USB_msg_len(last_start)) & ring_USB_datain.max;
				}
			}
		}
	}

ret:
	ring_USB_datain_backlocked = false;

#ifndef DEBUG
	if (mLED_Data_Timeout >= 2 * MLED_DATA_MAX_TIMEOUT) {
		mLED_Data_On();
		mLED_Data_Timeout = 0;
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////
/* Parse data from USB
 * \start and \len reference to ring_USB_datain.
 * \len is WITH header byte and WITH xor byte
 * Returns:
 *  true if data is waiting for sending to xpressnet
 *  false if data were processed
 * Warning: you must delete data from ring_USB_datain if data were processed.
 * (otherwise the program will freeze)
 */

bool USB_parse_data(uint8_t start, uint8_t len) {
	if (ring_USB_datain.data[start] == 0xF0) {
		// Instruction for the determination of the version and code number of LI
		ringRemoveFromMiddle(&ring_USB_datain, start, 2);
		pc_send_waiting.bits.version = true;
	} else if ((ring_USB_datain.data[start] == 0xF2)
	    && (ring_USB_datain.data[(start + 1) & ring_USB_datain.max] == 0x01)) {
		// Instruction for setting the LI101’s XpressNet address
		ringRemoveFromMiddle(&ring_USB_datain, start, 4);

		// set xpressnet addr
		if ((ring_USB_datain.data[(start + 2) & ring_USB_datain.max] > 0)
		    && (ring_USB_datain.data[(start + 2) & ring_USB_datain.max] < 32)) {
			xn_addr = ring_USB_datain.data[(start + 2) & ring_USB_datain.max] & 0x1F;
			WriteEEPROM(XN_EEPROM_ADDR, xn_addr);
		}

		// according to specification, when invalid address is passed to LI
		// LI should not change its address, but respond with current address
		// This allows PC to determine LI`s address.
		pc_send_waiting.bits.addr = true;

	} else if ((ring_USB_datain.data[start] == 0xF2)
	    && (ring_USB_datain.data[(start + 1) & ring_USB_datain.max] == 0x02)) {
		// Instruction for setting the LI101 Baud Rate
		ringRemoveFromMiddle(&ring_USB_datain, start, 4);
		tmp_baud_rate = ring_USB_datain.data[(start + 2) & ring_USB_datain.max];
		pc_send_waiting.bits.baud_rate = true;

#ifdef FERR_FEATURE
	} else if ((ring_USB_datain.data[start] == 0xF1)
	    && (ring_USB_datain.data[(start + 1) & ring_USB_datain.max] == 0x05)) {
		// special feture of uLI: framing error response
		// FERR is sent as response to 0xF1 0x05 0xF4 as 0xF4 0x05 FERR_HH FERR_H FERR_L XOR
		ringRemoveFromMiddle(&ring_USB_datain, start, 3);
		pc_send_waiting.bits.ferr = true;
#endif

	} else {
		// command for command station
		return true;
	}

	return false;
}

////////////////////////////////////////////////////////////////////////////////
/* Send data to USART.
 * How it works:
 *  This functinn is called when TX is enabled and sends each byte manually.
 *  It always sends message from beginning of the ring_USB_datain buffer.
 *  \to_send is index of byte in ring_USB_datain to be send next time.
 *  Notice: if this function has anything to send, it has to send it,
 *  because this function is called only once after TX gets ready.
 *  Returns: true if any data were send, otherwise false
 *  This function should be called only when there is anything to send and bus is in output state (at least 1 byte).
 *  WARNING: this function is called from interrupt!
 */

void USART_send(void) {
	uint8_t head = 0, id = 0;

	// according to specification, ninth bit is always 0
	USARTWriteByte(0, ring_USB_datain.data[usart_to_send]);
	usart_to_send = (usart_to_send + 1) & ring_USB_datain.max;

	if (usart_to_send ==
	    ((ring_USB_datain.ptr_b + USB_msg_len(ring_USB_datain.ptr_b)) & ring_USB_datain.max)) {
		// last byte sending
		head = ring_USB_datain.data[ring_USB_datain.ptr_b];
		id = ring_USB_datain.data[(ring_USB_datain.ptr_b + 1) & ring_USB_datain.max];

		ring_USB_datain.ptr_b = usart_to_send; // whole message sent
		if (ring_USB_datain.ptr_b == ring_USB_datain.ptr_e) ring_USB_datain.empty = true;

		PIE1bits.TXIE = 0;

		// XPRESSNET_DIR is set to XPRESSNET_IN after successful tranfer of last byte
		// This part of function is usually called from high-priority interrupt,
		// so nothing will overwrite us.
		// It is really important to switch direction as soon as possible.
		while (!TXSTAbits.TRMT);
		XPRESSNET_DIR = XPRESSNET_IN;

		check_response_to_PC(head, id); // send OK response to PC
	} else {
		// other-than-last byte sending
		PIE1bits.TXIE = 1;
	}
}

////////////////////////////////////////////////////////////////////////////////
// Debug function: dump ring buffer to USB

void dump_buf_to_USB(ring_generic* ring) {
	for (size_t i = 0; i <= ring->max && i < 32; i++)
		buf[i] = ring->data[(i + ring->ptr_b) & ring->max];
	putUSBUSART(buf, ring->max + 1);
}

////////////////////////////////////////////////////////////////////////////////
/* LI is supposed to answer 01/04/05 when some commands were succesfully
 * transmitted from LI to command station. This function takes care about it
 */

void check_response_to_PC(uint8_t header, uint8_t id) {
	static uint8_t respond_ok[] = { 0x22, 0x52, 0x83, 0x84, 0xE4, 0xE6, 0xE3 };

	if ((header >= 0x90) && (header <= 0x9F)) {
		pc_send_waiting.bits.ok = true;
		return;
	}

	for (size_t i = 0; i < sizeof(respond_ok); i++) {
		if ((header == respond_ok[i]) && ((header != 0xE3) || (id == 0x44)) &&
		    ((header != 0x22) || (id == 0x22))) {
			/* response is sent only if
			 *  0x44 follows 0xE3
			 *  0x22 follows 0x22
			 */
			pc_send_waiting.bits.ok = true;
			return;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void init_EEPROM(void) {
	xn_addr = ReadEEPROM(XN_EEPROM_ADDR);
	if ((xn_addr < 1) || (xn_addr > 31)) {
		xn_addr = DEFAULT_XPRESSNET_ADDR;
		WriteEEPROM(XN_EEPROM_ADDR, xn_addr);
	}
}

////////////////////////////////////////////////////////////////////////////////

void check_XN_timeout_supress(uint8_t ring_USB_msg_start) {
	// 0x22 and 0x23 should move supress sending of "no longer providing timeslot" message
	// however, "normal operations resumed" should be sent after normal operations resumed
	// DO send OK in this cases HEADER: 0x91, 0x92, 0x9N, 0x22, 0x52, 0x83, 0x84, 0xE4, 0xE6, 0xE3

	if ((ring_USB_datain.data[ring_USB_msg_start] == 0x22)
	    || (ring_USB_datain.data[ring_USB_msg_start] == 0x23)) {
		usart_longer_timeout = true;
		force_ok_response = true;
	}
}

////////////////////////////////////////////////////////////////////////////////
// Update flasihing of power LED.

void check_pwr_LED_status(void) {
	uint8_t new = 0;
	// new = (ferr_in_10_s > PWR_LED_FERR_COUNT) << 1;
	if (ferr_in_10_s > PWR_LED_FERR_COUNT)
		new |= 2;
	if ((ringLength(ring_USART_datain) >= (ring_USART_datain.max + 1) / 2)
	           || (ringLength(ring_USB_datain) >= (ring_USB_datain.max + 1) / 2))
	    new |= 4;
	if (new == 0) new = 1;
	pwr_led_status = new;
}

////////////////////////////////////////////////////////////////////////////////
/* Check broadcast from command station.
 * The purpose of getting broadcasts from command station is to
 * set proper length of timeslot_timeout.
 */

void check_broadcast(uint8_t xn_start_index) {
	uint8_t data[3];

	// serialize data from buffer
	ringSerialize(&ring_USART_datain, data, xn_start_index, 3);

	if ((data[0] == 0x60) && (data[1] == 0x61) && (data[2] == 0x02)) {
		// service mode entry
		programming_mode = true;
		usart_longer_timeout = true;
	}

	if ((data[0] == 0x60) && (data[1] == 0x61) && (data[2] == 0x01)) {
		// normal operations resumed
		programming_mode = false;
		usart_longer_timeout = false;
	}
}

////////////////////////////////////////////////////////////////////////////////
/* Check internal message "buffer" and send awaiting messages to PC.
 * This function is called periodically when no data are being received
 * to USART_input buffer.
 * Notice: this function must move ring_USART_datain.ptr_e before actually
 * adding data to ring_USART_datain or make addition of ALL data to buffer
 * atomic at once. Because at every time, this functon could be interrupted
 * by USART_receive_interrupt function, which also uses ring_USART_datain buffer.
 * It is important to keep these two functions in symbiosis.
 */

// This function relies on sending data to PC in non-interrupt function!

void check_device_data_to_USB(void) {
	if (!mUSBUSARTIsTxTrfReady()) return;

	if (pc_send_waiting.bits.version) {
		pc_send_waiting.bits.version = false;
		buf[0] = 0x02; buf[1] = VERSION_HW; buf[2] = VERSION_FW;
		buf[3] = (buf[0] ^ buf[1] ^ buf[2]);
		putUSBUSART(buf, 4);

	} else if (pc_send_waiting.bits.addr) {
		pc_send_waiting.bits.addr = false;
		buf[0] = 0xF2; buf[1] = 0x01; buf[2] = xn_addr;
		buf[3] = (buf[0] ^ buf[1] ^ buf[2]);
		putUSBUSART(buf, 4);

	} else if (pc_send_waiting.bits.baud_rate) {
		pc_send_waiting.bits.baud_rate = false;
		buf[0] = 0xF2; buf[1] = 0x02; buf[2] = tmp_baud_rate;
		buf[3] = (buf[0] ^ buf[1] ^ buf[2]);
		putUSBUSART(buf, 4);

	} else if (pc_send_waiting.bits.ok) {
		pc_send_waiting.bits.ok = false;
		buf[0] = 0x01; buf[1] = 0x04; buf[2] = 0x05;
		putUSBUSART(buf, 3);

	} else if (pc_send_waiting.bits.xor_error) {
		pc_send_waiting.bits.xor_error = false;
		buf[0] = 0x01; buf[1] = 0x03; buf[2] = 0x02;
		putUSBUSART(buf, 3);

	} else if (pc_send_waiting.bits.full_buffer) {
		pc_send_waiting.bits.full_buffer = false;
		buf[0] = 0x01; buf[1] = 0x06; buf[2] = 0x07;
		putUSBUSART(buf, 3);

	} else if (pc_send_waiting.bits.cs_timeout) {
		pc_send_waiting.bits.cs_timeout = false;
		buf[0] = 0x01; buf[1] = 0x02; buf[2] = 0x03;
		putUSBUSART(buf, 3);

	} else if (pc_send_waiting.bits.pc_timeout) {
		pc_send_waiting.bits.pc_timeout = false;
		buf[0] = 0x01; buf[1] = 0x01; buf[2] = 0x00;
		putUSBUSART(buf, 3);

	} else if (pc_send_waiting.bits.timeslot_timeout) {
		pc_send_waiting.bits.timeslot_timeout = false;
		buf[0] = 0x01; buf[1] = 0x05; buf[2] = 0x04;
		putUSBUSART(buf, 3);

#ifdef FERR_FEATURE
	} else if (pc_send_waiting.bits.ferr) {
		pc_send_waiting.bits.ferr = false;
		buf[0] = 0xF4;
		buf[1] = 0x05;
		buf[2] = (ferr_in_10_s >> 16) & 0xFF;
		buf[3] = (ferr_in_10_s >> 8) & 0xFF;
		buf[4] = ferr_in_10_s & 0xFF;
		buf[5] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4];
		putUSBUSART(buf, 6);
#endif
	}
}

////////////////////////////////////////////////////////////////////////////////
