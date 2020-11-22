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
//These defintions will tell the main() function which board is
//  currently selected.  This will allow the application to add
//  the correct configuration bits as wells use the correct
//  initialization functions for the board.  These defitions are only
//  required in the stack provided demos.  They are not required in
//  final application design.

#define DEMO_BOARD LOW_PIN_COUNT_USB_DEVELOPMENT_KIT
#define LOW_PIN_COUNT_USB_DEVELOPMENT_KIT
#define CLOCK_FREQ 48000000
#define GetSystemClock() CLOCK_FREQ

/** LED ************************************************************/
#define mInitAllLEDs();      TRISC &= 0xF8;

#define mLED_XN              PORTCbits.RC0
#define mLED_Data            PORTCbits.RC1
#define mLED_Pwr             PORTCbits.RC2

#define mLED_XN_On();        mLED_XN = 1;
#define mLED_Data_On();      mLED_Data = 1;
#define mLED_Pwr_On();       mLED_Pwr = 1;

#define mLED_XN_Off();       mLED_XN = 0;
#define mLED_Data_Off();     mLED_Data = 0;
#define mLED_Pwr_Off();      mLED_Pwr = 0;

#define mLED_XN_Toggle();    mLED_XN = !mLED_XN;
#define mLED_Data_Toggle();  mLED_Data = !mLED_Data;
#define mLED_Pwr_Toggle();   mLED_Pwr = !mLED_Pwr;

#endif  //HARDWARE_PROFILE_H
