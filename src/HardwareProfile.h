#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

/*******************************************************************/
/******** USB stack hardware selection options *********************/
/*******************************************************************/
//This section is the set of definitions required by the MCHPFSUSB
//  framework.  These definitions tell the firmware what mode it is
//  running in, and where it can find the results to some information
//  that the stack needs.
//These definitions are required by every application developed with
//  this revision of the MCHPFSUSB framework.  Please review each
//  option carefully and determine which options are desired/required
//  for your application.

//#define USE_SELF_POWER_SENSE_IO
#define tris_self_power     TRISAbits.TRISA2    // Input
#if defined(USE_SELF_POWER_SENSE_IO)
#define self_power          PORTAbits.RA2
#else
#define self_power          1
#endif

//#define USE_USB_BUS_SENSE_IO
#define tris_usb_bus_sense  TRISAbits.TRISA1    // Input
#if defined(USE_USB_BUS_SENSE_IO)
#define USB_BUS_SENSE       PORTAbits.RA1
#else
#define USB_BUS_SENSE       1
#endif

/*******************************************************************/
/*******************************************************************/
/*******************************************************************/
/******** Application specific definitions *************************/
/*******************************************************************/
/*******************************************************************/
/*******************************************************************/

//Uncomment the following line to make the output HEX of this
//  project work with the HID Bootloader
//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER

/** Board definition ***********************************************/

#define IO_OUT(TRIS, MASK) { TRIS &= ~MASK; }
#define IO_IN(TRIS, MASK) { TRIS |= MASK; }

/** LEDs ***********************************************************/

#define IO_LED_XN_LAT          LATCbits.LC0
#define IO_LED_XN_On()          { IO_LED_XN_LAT = 1; }
#define IO_LED_XN_Off()        { IO_LED_XN_LAT = 0; }
#define IO_LED_XN_Toggle()     { IO_LED_XN_LAT = !IO_LED_XN_LAT; }

#define IO_LED_DATA_LAT        LATCbits.LC1
#define IO_LED_Data_On()       { IO_LED_DATA_LAT = 1; }
#define IO_LED_Data_Off()      { IO_LED_DATA_LAT = 0; }
#define IO_LED_Data_Toggle()   { IO_LED_DATA_LAT = !IO_LED_DATA_LAT; }

#define IO_LED_PWR_LAT         LATCbits.LC2
#define IO_LED_Pwr_On()        { IO_LED_PWR_LAT = 1; }
#define IO_LED_Pwr_Off()       { IO_LED_PWR_LAT = 0; }
#define IO_LED_Pwr_Toggle()    { IO_LED_PWR_LAT = !IO_LED_PWR_LAT; }

/** IO ************************************************************/

#define IO_HW_VERSION_PORT     PORTCbits.RC7
#define IO_HW_VERSION_TRIS     TRISC
#define IO_HW_VERSION_MASK     0x80

static inline void IO_init(void) {
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;

    IO_LED_XN_On();
    IO_LED_Data_On();
    IO_LED_Pwr_On();
}

#endif  //HARDWARE_PROFILE_H
