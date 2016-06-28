/********************************************************************
 FileName:		main.c
 Processor:		PIC18F14K50
 Hardware:		uLI 2 - JH&MP 2015
 Complier:		Microchip C18
 Author:		Jan Horacek, Michal Petrilak 
 * 
/** INCLUDES *******************************************************/
#include <p18f14k50.h>

#include "usb.h"
#include "usb_function_cdc.h"
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "usb_device.h"
#include "usb.h"
#include "usart.h"
#include "main.h"
#include "ringBuffer.h"
#include "eeprom.h"

/** CONFIGURATION **************************************************/

// CONFIG1L
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection bits (No CPU System Clock divide)
#pragma config USBDIV = ON		// USB Clock Selection bit (USB clock comes from the OSC1/OSC2 divided by 2)

// CONFIG1H
#pragma config FOSC = HS		// Oscillator Selection bits (HS oscillator)
#pragma config PLLEN = ON		// 4 X PLL Enable bit (Oscillator multiplied by 4)
#pragma config PCLKEN = ON		// Primary Clock Enable bit (Primary clock enabled)
#pragma config FCMEN = OFF		// Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF		// Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON		// Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = SBORDIS	// Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 27		// Brown-out Reset Voltage bits (VBOR set to 2.7 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF		// Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config WDTPS = 32768	// Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config HFOFST = OFF		// HFINTOSC Fast Start-up bit (The system clock is held off until the HFINTOSC is stable.)
#pragma config MCLRE = ON		// MCLR Pin Enable bit (MCLR pin enabled; RA3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON		// Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON			// Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config BBSIZ = ON		// Boot Block Size Select bit (2kW boot block size)
#pragma config XINST = OFF		// Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

#pragma config CP0 = OFF		// Code Protection bit (Block 0 not code-protected)
#pragma config CP1 = OFF		// Code Protection bit (Block 1 not code-protected)
#pragma config CPB = OFF		// Boot Block Code Protection bit (Boot block not code-protected)
#pragma config CPD = OFF		// Data EEPROM Code Protection bit (Data EEPROM not code-protected)
#pragma config WRT0 = OFF		// Table Write Protection bit (Block 0 not write-protected)
#pragma config WRT1 = OFF		// Table Write Protection bit (Block 1 not write-protected)
#pragma config WRTC = OFF		// Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTB = OFF		// Boot Block Write Protection bit (Boot block not write-protected)
#pragma config WRTD = OFF		// Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config EBTR0 = OFF		// Table Read Protection bit (Block 0 not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF		// Table Read Protection bit (Block 1 not protected from table reads executed in other blocks)
#pragma config EBTRB = OFF		// Boot Block Table Read Protection bit (Boot block not protected from table reads executed in other blocks)

/** D E F I N E S ************************************************************/
#define USB_msg_len(start)		((ring_USB_datain.data[start] & 0x0F)+2)	  // len WITH header byte and WITH xor byte
#define USB_last_message_len	ringDistance(ring_USB_datain, last_start, ring_USB_datain.ptr_e)

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define USART_msg_len(start)	((ring_USART_datain.data[(start+1) & ring_USART_datain.max] & 0x0F)+3)	// length of xpressnet message is 4-lower bits in second byte
#define USART_last_message_len	ringDistance(ring_USART_datain, USART_last_start, ring_USART_datain.ptr_e)
#define USART_msg_to_send		(ringLength(ring_USB_datain) >= MAX(2, USB_msg_len(ring_USB_datain.ptr_b)))

#define USB_MAX_TIMEOUT                   10		// 100 ms
#define USART_MAX_TIMEOUT                  2		// 20 ms
#define TIMESLOT_MAX_TIMEOUT             500		// 5 s (according to specification)
#define TIMESLOT_LONG_MAX_TIMEOUT       9000		// 1:30 min (according to specification)
#define FERR_TIMEOUT                    1000		// 10 s

#define MLED_IN_MAX_TIMEOUT                5		// 50 ms
#define MLED_OUT_MAX_TIMEOUT               7        // 70 ms

#define PWR_LED_SHORT_COUNT               15		// 150 ms
#define PWR_LED_LONG_COUNT                40		// 400 ms
#define PWR_LED_FERR_COUNT                10		// status led indicates >10 framing errors

/** V A R I A B L E S ********************************************************/
#pragma udata
volatile char USB_Out_Buffer[32];                   // WARNING: this buffer should not be manipulated in interrupts!

volatile ring_generic ring_USB_datain;
volatile ring_generic ring_USART_datain;

#pragma idata

volatile BYTE our_frame = 0;
	// 0 = we cannot send messages
	// 1..80 = we can send messages
volatile BYTE usb_timeout = 0;
	// increment every 100 us -> 100 ms timeout = 1 000
volatile WORD usart_timeout = 0;
	// increment every 100 us -> 100 ms timeout = 1 000
volatile BYTE usart_to_send = 0;
	// byte to send to USART
	// I rather made this public volatile variable, beacause it is accessed in interrupts and in main too.
volatile BYTE usart_last_byte_sent = 0;
	// wheter the last byte of message to command station was sent and bus could be switched to IN direction
volatile BYTE ten_ms_counter = 0;
    // 10 ms counter

// timeslot errors
volatile WORD timeslot_timeout = 0;		  // timeslot timeout (1s = 100)
volatile BOOL timeslot_err = TRUE;		  // TRUE if timeslot error

// XpressNET framing error counting
#ifdef FERR_FEATURE
	volatile UINT24 ferr_in_10_s = 0;		  // number of framing errors in 10 seconds
	volatile UINT24 ferr_counter = 0;
#endif

volatile BYTE mLED_In_Timeout = 2*MLED_IN_MAX_TIMEOUT;
volatile BYTE mLED_Out_Timeout = 2*MLED_OUT_MAX_TIMEOUT;
volatile BOOL usart_longer_timeout = FALSE;
volatile BYTE xn_addr = DEFAULT_XPRESSNET_ADDR;
volatile BOOL force_ok_response = FALSE;

// Power led blinks pwr_led_status times, then stays blank for some time
//	and then repeats the whole cycle. This lets user to see software status.
volatile BYTE pwr_led_base_timeout = PWR_LED_SHORT_COUNT;
volatile BYTE pwr_led_base_counter = 0;
volatile BYTE pwr_led_status_counter = 0;
volatile BYTE pwr_led_status = 1;

volatile BOOL usb_configured = FALSE;
volatile BOOL programming_mode = FALSE;

volatile nine_data USART_received = {0, 0, FALSE};
volatile BYTE USART_last_start = 0;


/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void BlinkUSBStatus(void);
void UserInit(void);

void USART_receive_main(void);
void USART_receive_interrupt(void);
void USART_process_message(void);
void USART_send(void);
void USART_check_timeouts(void);

void USB_send(void);
void USB_receive(void);
BOOL USB_parse_data(BYTE start, BYTE len);
BYTE calc_xor(BYTE* data, BYTE len);
void Timer2(void);
void ProcessIO(void);
void dumpBufToUSB(ring_generic* buf);
void checkResponseToPC(BYTE header, BYTE id);
void InitEEPROM(void);
void Check_XN_timeout_supress(BYTE ring_USB_msg_start);
void CheckPwrLEDStatus(void);
void CheckBroadcast(int xn_start_index);

void respondOK(void);
void respondXORerror(void);
void respondBufferFull(void);
void respondCommandStationTimeout(void);

/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
		_asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
		_asm goto YourLowPriorityISRCode _endasm
	}
	
	#pragma code
	
	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
		#if defined(USB_INTERRUPT)
			USBDeviceTasks();
		#endif

		// USART send interrupt
		if ((PIE1bits.TXIE) && (PIR1bits.TXIF)) {
			USART_send();
		}
            
        if ((PIE1bits.RCIE) && (PIR1bits.RCIF)) {
            // receive interrupt just for first byte
            USART_receive_interrupt();
        }

	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
		
		// Timer2 on 100 us
		if ((PIE1bits.TMR2IE) && (PIR1bits.TMR2IF)) {
			
            if (ten_ms_counter < 100) { ten_ms_counter++; }
            else {
                ten_ms_counter = 0;
                
                // 10 ms overflow:
                
                // usb receive timeout
                if (usb_timeout < USB_MAX_TIMEOUT) usb_timeout++;
	
                // usart receive timeout
                if (usart_timeout < USART_MAX_TIMEOUT) usart_timeout++;
			
                // timeslot timeout
                if (timeslot_timeout < TIMESLOT_LONG_MAX_TIMEOUT) timeslot_timeout++;
                
                // mLEDout timeout
    			if (mLED_Out_Timeout < 2*MLED_OUT_MAX_TIMEOUT) {
    				mLED_Out_Timeout++;
    				if (mLED_Out_Timeout == MLED_OUT_MAX_TIMEOUT) {
    					mLED_Out_Off();
    				}
    			}
                
    			#ifdef FERR_FEATURE
    				// framing error counting
    				ferr_counter++;
    				if (ferr_counter >= FERR_TIMEOUT) {
    					ferr_in_10_s = 0;
    					ferr_counter = 0;
    				}
    			#endif
                
    			// mLEDIn timeout
    			if (mLED_In_Timeout < 2*MLED_IN_MAX_TIMEOUT) {
    				mLED_In_Timeout++;
    				if (mLED_In_Timeout == MLED_IN_MAX_TIMEOUT) {
    					mLED_In_On();
    				}
    			}
			
    			// pwrLED toggling
    			pwr_led_base_counter++;
    			if (pwr_led_base_counter >= pwr_led_base_timeout) {
    				pwr_led_base_counter = 0;
    				pwr_led_status_counter++;
				
                    if (pwr_led_status_counter == 2*pwr_led_status) {
    					// wait between cycles
    					pwr_led_base_timeout = PWR_LED_LONG_COUNT;
                        mLED_Pwr_Off();
    				} else if (pwr_led_status_counter > 2*pwr_led_status) {
    					// new base cycle
    					pwr_led_base_timeout = PWR_LED_SHORT_COUNT;
    					pwr_led_status_counter = 0;
    					mLED_Pwr_On();
    				} else {
    					mLED_Pwr_Toggle();
    				}
                }
                
                // end of 10 ms counter
            }
            
			// XPRESSNET_DIR is set to XPRESSNET_IN after successful tranfer of last byte
			if ((usart_last_byte_sent) && (TXSTAbits.TRMT)) {
				XPRESSNET_DIR = XPRESSNET_IN;
				usart_last_byte_sent = 0;
			}
			
			PIR1bits.TMR2IF = 0;		// reset overflow flag
		}
		
	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 

#endif


/** DECLARATIONS ***************************************************/
#pragma code

void main(void)
{
	InitializeSystem();

	while(1)
	{
		#if defined(USB_INTERRUPT)
			if(USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE))
			{
				USBDeviceAttach();
			}
		#endif

        // USART receiving
        // first byte is receive in interrupt, others in main
        if (USART_received.ready) {
            // message in buffer -> process
            USART_process_message();
        } else {
            if (!PIE1bits.RCIE) {
                // not receiving by interrupt -> receive in main
                USART_receive_main();
                if (USART_received.ready) { USART_process_message(); }
            }
        }
        USART_check_timeouts();
            
		USB_receive();
		USB_send();

		CheckPwrLEDStatus();
		CDCTxService();
	}//end while
}//end main

void InitializeSystem(void)
{
	#if (defined(__18CXX) & !defined(PIC18F87J50_PIM))
		ADCON1 |= 0x0F;					// Default all pins to digital
	#endif

	#if defined(USE_USB_BUS_SENSE_IO)
		tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
	#endif
	
	#if defined(USE_SELF_POWER_SENSE_IO)
		tris_self_power = INPUT_PIN;	// See HardwareProfile.h
	#endif

	UserInit();
	InitEEPROM();
	USBDeviceInit();
	USARTInit();
}

void UserInit(void)
{
	// init ring buffers
	ringBufferInit(ring_USB_datain, 32);
	ringBufferInit(ring_USART_datain, 32);
 
	// switch off AD convertors (USART is not working when not switched off manually)
	ANSEL = 0x00;
	ANSELH = 0x00;
	
	// Initialize all of the LED pins
	mInitAllLEDs();
	mLED_In_On();
	mLED_Pwr_Off();
	mLED_Out_On();
	
	// setup timer2 on 100 us
	T2CONbits.T2CKPS = 0b11;	// prescaler 16x
	PR2 = 75;					// setup timer period register to interrupt every 100 us
	TMR2 = 0x00;				// reset timer counter
	PIR1bits.TMR2IF = 0;		// reset overflow flag
	PIE1bits.TMR2IE = 1;		// enable timer2 interrupts
	IPR1bits.TMR2IP = 0;		// timer2 interrupt low level
	
	RCONbits.IPEN = 1;			// enable high and low priority interrupts
	INTCONbits.PEIE = 1;	    // Enable peripheral interrupts (for usart)
	INTCONbits.GIE = 1;			// enable global interrupts
	INTCONbits.GIEH = 1;
	INTCONbits.GIEL = 1;
	
	T2CONbits.TMR2ON = 1;		// enable timer2
}//end UserInit

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events. For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device. In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function. You should modify these callback functions to take appropriate actions for each of these
// conditions. For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

void USBCBSuspend(void)
{
	#if defined(__C30__)
		USBSleepOnSuspend();
	#endif

	usb_configured = FALSE;
	mLED_Out_On();
	ringClear((ring_generic*)&ring_USART_datain);
	ringClear((ring_generic*)&ring_USB_datain);
}

void USBCBWakeFromSuspend(void)
{
	usb_configured = TRUE;
	mLED_Out_Off();
}

void USBCB_SOF_Handler(void)
{
	
}

void USBCBErrorHandler(void)
{
	
}

void USBCBCheckOtherReq(void)
{
	USBCheckCDCRequest();
}//end

void USBCBStdSetDscHandler(void)
{
	// Must claim session ownership if supporting this request
}

void USBCBInitEP(void)
{
	CDCInitEP();
	usb_configured = TRUE;
	mLED_Out_Off();
}

void USBCBSendResume(void)
{
	
}

#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
	switch(event)
	{
		case EVENT_CONFIGURED: 
			USBCBInitEP();
			break;
		case EVENT_SET_DESCRIPTOR:
			USBCBStdSetDscHandler();
			break;
		case EVENT_EP0_REQUEST:
			USBCBCheckOtherReq();
			break;
		case EVENT_SOF:
			USBCB_SOF_Handler();
			break;
		case EVENT_SUSPEND:
			USBCBSuspend();
			break;
		case EVENT_RESUME:
			USBCBWakeFromSuspend();
			break;
		case EVENT_BUS_ERROR:
			USBCBErrorHandler();
			break;
		case EVENT_TRANSFER:
			Nop();
			break;
		default:
			break;
	}	   
	return TRUE; 
}

////////////////////////////////////////////////////////////////////////////////

BYTE calc_xor(BYTE* data, BYTE len)
{
	int xor = 0, i;
	for (i = 0; i < len; i++) xor ^= data[i];
	return xor;
}

////////////////////////////////////////////////////////////////////////////////
/* How does USART receiving work:
 * By default, USART waits for normal inquiry. When data are sent do uLI,
 * receive interrupt is fired and data are received in USART_receive_interrupt.
 * This function should be very fast, its main purpose is to start transmission
 * from uLI to command station very fast (< 80 us). This turned out to be quite
 * big problem. All bytes except the first are received in USART_receive_main,
 * which is called from main. So, first by is received in interrupt, all other
 * bytes are received in main.
 */

/* CHECKING USART TIMEOUTS
 * This function can be called from main, it does not require specific timing.
 */
void USART_check_timeouts(void)
{
	// check for (short) timeout
	if ((USART_last_start != ring_USART_datain.ptr_e) && (usart_timeout >= USART_MAX_TIMEOUT)) {
		// delete last incoming message and wait for next message
		ring_USART_datain.ptr_e = USART_last_start;
		if (ring_USART_datain.ptr_e == ring_USART_datain.ptr_b) ring_USART_datain.empty = TRUE;
		usart_timeout = 0;
		RCSTAbits.ADDEN = 1;	// receive just first message
        PIE1bits.RCIE = 1;      // receive by interrupt
		
		// inform PC about timeout
		respondCommandStationTimeout();
		
		return;
	}
	
	// check timeslot_timeout
	// shorter timeout is default, longer timeout is for programming commands
	if ((((timeslot_timeout >= TIMESLOT_MAX_TIMEOUT) && (!usart_longer_timeout)) || (timeslot_timeout >= TIMESLOT_LONG_MAX_TIMEOUT))
			&& (!timeslot_err)) {
        // Command station is no longer providing timeslot for communication.
		timeslot_err = TRUE;
        
        // remove buffers
        ringClear((ring_generic*)&ring_USART_datain);
        ringClear((ring_generic*)&ring_USB_datain);
        		
        /* timeslot timeout -> connection to PC is probably without any data ->
         * we could send data to USB directly and assume mUSBUSARTIsTxTrfReady()
         * is always true.
         */        
        if (mUSBUSARTIsTxTrfReady()) {
            USB_Out_Buffer[0] = 0x01;
            USB_Out_Buffer[1] = 0x05;
            USB_Out_Buffer[2] = 0x04;
            putUSBUSART((char*)USB_Out_Buffer, 3);
        }
	}    
}

/* RECEIVING A BYTE FROM MAIN LOOP
 *  This function should be called only when USART_received buffer is free.
 */
void USART_receive_main(void)
{
	if ((XPRESSNET_DIR == XPRESSNET_OUT) || (!USARTInputData())) return;
	usart_timeout = 0;
		
	USART_received = USARTReadByte();

	#ifdef FERR_FEATURE
		// increment framing eror counter in case of framing error
		ferr_counter += USART_received.FERR;
	#endif
}

/* RECEIVING A BYTE FROM AN INTERRUPT
 * This function is called only if ninth bit is 1.
 * This function must be as fast as possible!
 * This and only this function processes normal inquiery and request acknowledgement.
 * Other messages are processes in USART_process_message.
 */
void USART_receive_interrupt(void)
{
    USART_received = USARTReadByte();
    
    // data not for us -> will be processed in USART_receive_main
    if ((USART_received.data & 0x1F) != xn_addr) return;

    usart_timeout = 0;
    // I do not care about the parity. No time!
    
    if (((USART_received.data >> 5) & 0b11) == 0b10) {
        // normal inquiry
        
        // first thing to do: send data as soon as possible! (we have only 80 us)
        if (USART_msg_to_send) {
            XPRESSNET_DIR = XPRESSNET_OUT;
            USART_send();
        }
        
        timeslot_err = FALSE;
        usart_longer_timeout = programming_mode;
        if ((timeslot_timeout >= TIMESLOT_MAX_TIMEOUT) || (force_ok_response)) {
            // ok response must be sent always after short timeout					
            if (force_ok_response) force_ok_response = FALSE;
            respondOK();
        }
        timeslot_timeout = 0;
				
		if (USART_msg_to_send) { Check_XN_timeout_supress(ring_USB_datain.ptr_b); }
        USART_received.ready = FALSE; // message processed
        
	} else if (((USART_received.data >> 5) & 0b11) == 0b00) {
        // request acknowledgement
        // send Acknowledgement Response to command station (this should be done by LI)
                    
        if (ringFreeSpace(ring_USB_datain) < 2) {
            // This situation should not happen. 2 bytes in ring_USB_datain
            // are always reserved for acknowledgement response.
            ringClear((ring_generic*)&ring_USB_datain);
        }

        // add ACK to beginning of the buffer
        ring_USB_datain.ptr_b = (ring_USB_datain.ptr_b-2) & ring_USB_datain.max;
        ring_USB_datain.empty = FALSE;
        ring_USB_datain.data[ring_USB_datain.ptr_b] = 0x20;
        ring_USB_datain.data[(ring_USB_datain.ptr_b+1) & ring_USB_datain.max] = 0x20;
        usart_to_send = ring_USB_datain.ptr_b;
								
        // send ACK to command station
        XPRESSNET_DIR = XPRESSNET_OUT;
        USART_send();
                
        USART_received.ready = FALSE; // message processes
    } else {
        // normal message -> read in main
        PIE1bits.RCIE = 0;
    }
    
	#ifdef FERR_FEATURE
		// increment framing eror counter in case of framing error
		ferr_counter += USART_received.FERR;
	#endif
    
    // toggle LED
    if (mLED_In_Timeout >= 2*MLED_IN_MAX_TIMEOUT) {
        mLED_In_Off();
        mLED_In_Timeout = 0;
    }
}

/* PROCESSING INPUT DATA (other than normal inquiry and request for ACK)
 * This function must be called only if USART_received.ready
 * This function cannot be called with normal inquiiry or request for acknowledgement.
 * In this function, we have a lot of time.
 */
void USART_process_message(void)
{
    BOOL parity;    
    static BYTE xor = 0;
    BYTE i;
    
    USART_received.ready = FALSE;       // byte is parsed
    
    if (USART_received.ninth) {
		// 9 bit is 1 -> header byte
		// we are waiting for call byte with our address

		if ((xn_addr == (USART_received.data & 0x1F)) || ((USART_received.data & 0x1F) == 0)) {
			// new message for us -> check for parity
			for(i = 0, parity = 0; i < 8; i++) if ((USART_received.data >> i) & 1) parity = !parity;
			if (parity != 0) {
				// parity error
				respondXORerror();
				return;
			}
			
			if (ring_USART_datain.ptr_e != USART_last_start) {
				// beginning of new message received before previous mesage was completely received -> delete previous message
				ring_USART_datain.ptr_e = USART_last_start;
				if (ring_USART_datain.ptr_e == ring_USART_datain.ptr_b) ring_USART_datain.empty = TRUE;
				
				// send info to PC
				respondCommandStationTimeout();
			}
			
			// start of message for us (or broadcast)
				
			if (!usb_configured) return;
				
			// -> check space in buffer
			if (ringFull(ring_USART_datain)) {					
                // buffer full -> probably no space to send data to PC
                // -> do not send information message to PC
                //respondBufferFull();
				return;
			}
					
			RCSTAbits.ADDEN = 0;	// receive all messages
            PIE1bits.RCIE = 0;      // receive in main
			USART_last_start = ring_USART_datain.ptr_e;
			xor = 0;
			ringAddByte((ring_generic*)&ring_USART_datain, USART_received.data);
		}
	} else {
		// ninth bit is 0
		// this situation happens only if message is for us (guaranteed by RCSTAbits.ADDEN)
		if (ringFull(ring_USART_datain)) {
			// reset buffer and wait for next message
			ring_USART_datain.ptr_e = USART_last_start;
			RCSTAbits.ADDEN = 1;
            PIE1bits.RCIE = 1;
			if (ring_USART_datain.ptr_e == ring_USART_datain.ptr_b) ring_USART_datain.empty = TRUE;
			
			// inform PC about buffer overflow
			respondBufferFull();
			
			return;
		}
		ringAddByte((ring_generic*)&ring_USART_datain, USART_received.data);
		xor ^= USART_received.data;
		
		if (USART_last_message_len >= USART_msg_len(USART_last_start)) {
			// whole message received
            
			if (xor != 0) {
				// xor error
				ring_USART_datain.ptr_e = USART_last_start;
				if (ring_USART_datain.ptr_e == ring_USART_datain.ptr_b) ring_USART_datain.empty = TRUE;
				respondXORerror();
			} else {
				// xor ok
                CheckBroadcast(USART_last_start);
				USART_last_start = ring_USART_datain.ptr_e;	// whole message succesfully received                
			}

			// listen for beginning of message (9. bit == 1)
			RCSTAbits.ADDEN = 1;
            PIE1bits.RCIE = 1;      // receive by interrupt
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// Check for data in ring_USART_datain and send complete data to USB.

void USB_send(void)
{	
	// check for USB ready
	if (!mUSBUSARTIsTxTrfReady()) return;

	if (((ringLength(ring_USART_datain)) >= 1) && (ringLength(ring_USART_datain) >= USART_msg_len(ring_USART_datain.ptr_b))) {
		// send message
		ringSerialize((ring_generic*)&ring_USART_datain, (BYTE*)USB_Out_Buffer, ring_USART_datain.ptr_b, USART_msg_len(ring_USART_datain.ptr_b));
		putUSBUSART((char*)USB_Out_Buffer+1, ((USB_Out_Buffer[1])&0x0F)+2);
		ringRemoveFrame((ring_generic*)&ring_USART_datain, ((USB_Out_Buffer[1])&0x0F)+3);
	}
}

////////////////////////////////////////////////////////////////////////////////
/* Receive data from USB and add it to ring_USB_datain.
 * Index of start of last message is in 
 */

void USB_receive(void)
{
	static BYTE last_start = 0;
	BYTE xor, i;
	BYTE received_len;
	
	if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

	if(mUSBUSARTIsTxTrfReady())
	{
		// ring_USB_datain overflow check
		// 2 bytes in the buffer are always reserved for the Acknowledgement
		// response.
		if (ringFreeSpace(ring_USB_datain) < 2) {
			// delete last message
			ring_USB_datain.ptr_e = last_start;
			if (ring_USB_datain.ptr_b == ring_USB_datain.ptr_e) ring_USART_datain.empty = TRUE;
			
			// inform PC about full buffer
			respondBufferFull();
			
			return;
		}
		
		received_len = getsUSBUSART((ring_generic*)&ring_USB_datain, ringFreeSpace(ring_USB_datain));
		if (received_len == 0) {
			// check for timeout
			if ((usb_timeout >= USB_MAX_TIMEOUT) && (last_start != ring_USB_datain.ptr_e)) {
				ring_USB_datain.ptr_e = last_start;
				usb_timeout = 0;
				if (ring_USB_datain.ptr_e == ring_USB_datain.ptr_b) ring_USB_datain.empty = TRUE;
				
				// inform PC about timeout
                if (mUSBUSARTIsTxTrfReady()) {
                    USB_Out_Buffer[0] = 0x01;
                    USB_Out_Buffer[1] = 0x01;
                    USB_Out_Buffer[2] = 0x00;
                    putUSBUSART((char*)USB_Out_Buffer, 3);
                }
			}
			return;
		}
		usb_timeout = 0;
				
		// data received -> parse data
		while ((ringDistance(ring_USB_datain, last_start, ring_USB_datain.ptr_e) > 0) &&
				(USB_last_message_len >= USB_msg_len(last_start))) {
			
			// whole message received -> check for xor
			for (i = 0, xor = 0; i < USB_msg_len(last_start)-1; i++)
				xor ^= ring_USB_datain.data[(i+last_start) & ring_USB_datain.max];
			
			if (xor != ring_USB_datain.data[(i+last_start) & ring_USB_datain.max]) {
				// xor error
				// here, we need to delete content in the middle of ring buffer
				ringRemoveFromMiddle((ring_generic*)&ring_USB_datain, last_start, USB_msg_len(last_start));
				respondXORerror();
				return;
			}
			
			// xor ok -> parse data
			if (USB_parse_data(last_start, USB_msg_len(last_start))) {
				// timeslot not available -> respond "Buffer full"
				if (timeslot_err) {
					ringRemoveFromMiddle((ring_generic*)&ring_USB_datain, last_start, USB_msg_len(last_start));
					respondBufferFull();
					return;
				}
				
				// data waiting for sending to xpressnet -> move last_start
				last_start = (last_start+USB_msg_len(last_start))&ring_USB_datain.max;
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
/* Parse data from USB
 * \start and \len reference to ring_USB_datain.
 * \len is WITH header byte and WITH xor byte
 * Returns:
 *	TRUE if data is waiting for sending to xpressnet
 *	FALSE if data were processed
 * Achtung: you must delete data from ring_USB_datain if data were processed.
 * (otherwise the program will freeze)
 */ 

BOOL USB_parse_data(BYTE start, BYTE len)
{
	if (ring_USB_datain.data[start] == 0xF0) {
		// Instruction for the determination of the version and code number of LI
		ringRemoveFromMiddle((ring_generic*)&ring_USB_datain, start, 2);
        if (mUSBUSARTIsTxTrfReady()) {
            USB_Out_Buffer[0] = 0x02;
            USB_Out_Buffer[1] = VERSION_HW;
            USB_Out_Buffer[2] = VERSION_FW;
    		USB_Out_Buffer[3] = calc_xor((BYTE*)USB_Out_Buffer, 3);
    		putUSBUSART((char*)USB_Out_Buffer, 4);
        }
	} else if ((ring_USB_datain.data[start] == 0xF2) &&
			(ring_USB_datain.data[(start+1)&ring_USB_datain.max] == 0x01)) {
		// Instruction for setting the LI101’s XpressNet address
		ringRemoveFromMiddle((ring_generic*)&ring_USB_datain, start, 4);

		// set xpressnet addr
		if ((ring_USB_datain.data[(start+2)&ring_USB_datain.max] > 0) && (ring_USB_datain.data[(start+2)&ring_USB_datain.max] < 32)) {
			xn_addr = ring_USB_datain.data[(start+2)&ring_USB_datain.max] & 0x1F;
			WriteEEPROM(XN_EEPROM_ADDR, xn_addr);
		}
		
		// according to specification, when invalid address is passed to LI
		// LI should not change its address, but respond with current address
		// This allows PC to determine LI`s address.
		
        if (mUSBUSARTIsTxTrfReady()) {
            USB_Out_Buffer[0] = 0xF2;
            USB_Out_Buffer[1] = 0x01;
            USB_Out_Buffer[2] = xn_addr;
            USB_Out_Buffer[3] = calc_xor((BYTE*)USB_Out_Buffer, 3);
            putUSBUSART((char*)USB_Out_Buffer, 4);
        }
	} else if ((ring_USB_datain.data[start] == 0xF2) &&
			(ring_USB_datain.data[(start+1)&ring_USB_datain.max] == 0x02)) {
		// Instruction for setting the LI101 Baud Rate
		ringRemoveFromMiddle((ring_generic*)&ring_USB_datain, start, 4);
        if (mUSBUSARTIsTxTrfReady()) {
            USB_Out_Buffer[0] = 0xF2;
            USB_Out_Buffer[1] = 0x02;
            USB_Out_Buffer[2] = ring_USB_datain.data[(start+2)&ring_USB_datain.max];
            USB_Out_Buffer[3] = calc_xor((BYTE*)USB_Out_Buffer, 3);
            putUSBUSART((char*)USB_Out_Buffer, 4);
        }
	
	#ifdef FERR_FEATURE
	} else if ((ring_USB_datain.data[start] == 0xF1) &&
			(ring_USB_datain.data[(start+1)&ring_USB_datain.max] == 0x05)) {
		// special feture of uLI: framing error response
		// FERR is sent as response to 0xF1 0x05 0xF4 as 0xF4 0x05 FERR_HH FERR_H FERR_L XOR
		ringRemoveFromMiddle((ring_generic*)&ring_USB_datain, start, 3);

        if (mUSBUSARTIsTxTrfReady()) {
            USB_Out_Buffer[0] = 0xF4;
            USB_Out_Buffer[1] = 0x05;
            USB_Out_Buffer[2] = (ferr_in_10_s >> 16) & 0xFF;
            USB_Out_Buffer[3] = (ferr_in_10_s >> 8) & 0xFF;
            USB_Out_Buffer[4] = ferr_in_10_s & 0xFF;
    		USB_Out_Buffer[5] = calc_xor((BYTE*)USB_Out_Buffer, 5);
    		if (mUSBUSARTIsTxTrfReady()) putUSBUSART((char*)USB_Out_Buffer, 6);
        }
	#endif

	} else {
		// command for command station
		return TRUE;
	}
	
	return FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/* Send data to USART.
 * How it works:
 *	This functino is called when TX is enabled and sends each byte manually.
 *	It always sends message from beginning of the ring_USB_datain buffer.
 *	\to_send is index of byte in ring_USB_datain to be send next time.
 *	Note: if this function has anything to send, it has to send it,
 *	because this function is called only once after TX gets ready.
 *	Returns: true if any data were send, otherwise false
 *	This function should be called only when there is anything to send and bus is in output state (at least 1 byte).
 */ 
void USART_send(void)
{
	static BYTE head = 0, id = 0;
	
	// according to specification, ninth bit is always 0
	USARTWriteByte(0, ring_USB_datain.data[usart_to_send]);
	usart_to_send = (usart_to_send+1)&ring_USB_datain.max;
	
	if (usart_to_send == ((ring_USB_datain.ptr_b + USB_msg_len(ring_USB_datain.ptr_b))&ring_USB_datain.max)) {
		// last byte sending
		head = ring_USB_datain.data[ring_USB_datain.ptr_b];
		id = ring_USB_datain.data[(ring_USB_datain.ptr_b+1)&ring_USB_datain.max];
		
		ring_USB_datain.ptr_b = usart_to_send;  // whole message sent
		if (ring_USB_datain.ptr_b == ring_USB_datain.ptr_e) ring_USB_datain.empty = TRUE;
		
		checkResponseToPC(head, id); // send OK response to PC
		
		PIE1bits.TXIE = 0;
		usart_last_byte_sent = 1;
	} else {
		// other-than-last byte sending
		PIE1bits.TXIE = 1;
	}
    
    if (mLED_Out_Timeout >= 2*MLED_OUT_MAX_TIMEOUT) {
        mLED_Out_On();
        mLED_Out_Timeout = 0;
    }    
}

////////////////////////////////////////////////////////////////////////////////
// Debug function: dump eing buffer to USB
void dumpBufToUSB(ring_generic* buf)
{
	int i;
	for (i = 0; i <= buf->max; i++) USB_Out_Buffer[i] = buf->data[i];
	putUSBUSART((char*)USB_Out_Buffer, buf->max+1);
}

////////////////////////////////////////////////////////////////////////////////

void respondOK(void)
{
    if (mUSBUSARTIsTxTrfReady()) {
        USB_Out_Buffer[0] = 0x01;
        USB_Out_Buffer[1] = 0x04;
        USB_Out_Buffer[2] = 0x05;
        putUSBUSART((char*)USB_Out_Buffer, 3);
    }
}

void respondXORerror(void)
{
    if (mUSBUSARTIsTxTrfReady()) {
        USB_Out_Buffer[0] = 0x01;
        USB_Out_Buffer[1] = 0x03;
    	USB_Out_Buffer[2] = 0x02;
    	putUSBUSART((char*)USB_Out_Buffer, 3);
    }
}

void respondBufferFull(void)
{
    if (mUSBUSARTIsTxTrfReady()) {
        USB_Out_Buffer[0] = 0x01;
        USB_Out_Buffer[1] = 0x06;
        USB_Out_Buffer[2] = 0x07;
    	putUSBUSART((char*)USB_Out_Buffer, 3);
    }
}

void respondCommandStationTimeout(void)
{
    if (mUSBUSARTIsTxTrfReady()) {
        USB_Out_Buffer[0] = 0x01;
        USB_Out_Buffer[1] = 0x02;
    	USB_Out_Buffer[2] = 0x03;
    	putUSBUSART((char*)USB_Out_Buffer, 3);
    }
}

////////////////////////////////////////////////////////////////////////////////
/* LI is supposed to answer 01/04/05 when some commands were succesfully
 * transmitted from LI to command station. This function takes care about it
 */
void checkResponseToPC(BYTE header, BYTE id)
{
	static BYTE respond_ok[] = {0x22, /*0x23,*/ 0x52, 0x83, 0x84, 0xE4, 0xE6, 0xE3};
	int i;
	
	if ((header >= 0x90) && (header <= 0x9F)) {
		respondOK();
		return;
	}

	for (i = 0; i < sizeof(respond_ok); i++)
		if ((header == respond_ok[i]) && ((header != 0xE3) || (id == 0x44)) && ((header != 0x22) || (id == 0x22))) {
			/* response is sent only if
			 *	0x44 follows 0xE3
			 *	0x22 follows 0x22
			 */ 
			respondOK();
			return;			   
		}
}

////////////////////////////////////////////////////////////////////////////////

void InitEEPROM(void)
{
	xn_addr = ReadEEPROM(XN_EEPROM_ADDR);
	if ((xn_addr < 1) || (xn_addr > 31)) {
		xn_addr = DEFAULT_XPRESSNET_ADDR;
		WriteEEPROM(XN_EEPROM_ADDR, xn_addr);
	}
}

////////////////////////////////////////////////////////////////////////////////

void Check_XN_timeout_supress(BYTE ring_USB_msg_start)
{
	// 0x22 and 0x23 should move supress sending of "no longer providing timeslot" message
	// however, "normal operations resumed" should be sent after normal operations resumed
	// DO send OK in this cases HEADER: 0x91, 0x92, 0x9N, 0x22, 0x52, 0x83, 0x84, 0xE4, 0xE6, 0xE3
	
	if ((ring_USB_datain.data[ring_USB_msg_start] == 0x22) || 
        (ring_USB_datain.data[ring_USB_msg_start] == 0x23)) {
        usart_longer_timeout = TRUE;
        force_ok_response = TRUE;
    }
}

////////////////////////////////////////////////////////////////////////////////

void CheckPwrLEDStatus(void)
{
	BYTE new;
	new = (ferr_in_10_s > PWR_LED_FERR_COUNT) << 1;
	new |= ((ringLength(ring_USART_datain) >= (ring_USART_datain.max+1)/2) || (ringLength(ring_USB_datain) >= (ring_USB_datain.max+1)/2)) << 2;
	if (new == 0) new = 1;
	pwr_led_status = new;
}

////////////////////////////////////////////////////////////////////////////////

// Check broadcast from command station.
// The purpose of getting broadcasts from command station is to
// set proper length of timeslot_timeout.
void CheckBroadcast(int xn_start_index)
{
    static BYTE data[3];
    
    // serialize data from buffer
    ringSerialize((ring_generic*)&ring_USART_datain, data, xn_start_index, 4);
    
    if ((data[0] == 0x60) && (data[1] == 0x61) && (data[2] == 0x02)) {
        // service mode entry
        programming_mode = TRUE;
        usart_longer_timeout = TRUE;
    }
    
    if ((data[0] == 0x60) && (data[1] == 0x61) && (data[2] == 0x01)) {
        // normal operations resumed
        programming_mode = FALSE;
        usart_longer_timeout = FALSE;
    }
}

////////////////////////////////////////////////////////////////////////////////
