/* Usart communication library implementation */

#include <xc.h>

#include "GenericTypeDefs.h"
#include "usart.h"

void USARTInit(void) {
	// Baud Rate = 62500 Baud per second
	// on 48 MHz

	TRISBbits.TRISB5 = 1; // RB5/RX is for read
	TRISBbits.TRISB6 = 0; // RB7/TX is for write
	TRISBbits.TRISB7 = 0; // RB6/RS485 direction set is for write

	// timing: 8-bit / asynchronous (based on datasheet p. 187)
	TXSTAbits.BRGH = 0;
	BAUDCONbits.BRG16 = 0;
	SPBRG = 11;

	XPRESSNET_DIR = XPRESSNET_IN; // switch bus for read

	TXSTAbits.SYNC = 0;  // enable async mode
	RCSTAbits.SPEN = 1;  // enable async mode
	TXSTAbits.TX9 = 1;   // 9-bit sending
	RCSTAbits.RX9 = 1;   // 9-bit receiving
	RCSTAbits.ADDEN = 1; // enable address detection
	                     // TX interrupt must be in high level (otherwise will not match 80 us window)
	RCSTAbits.CREN = 1;  // enable RX
	TXSTAbits.TXEN = 1;  // enable TX
	                     //
	IPR1bits.RCIP = 1;   // receive interrupt high priority
	PIE1bits.RCIE = 1;   // enable read interrupt
}

// Write byte to USART
// Note: by setting PIE1bits.TXIE to 1, interrupt is called when a word is transmitted
//	Use for transmitting of multiple words.
//	And do not forget to set TXIE back to 0 after last character is transmitted.
void USARTWriteByte(unsigned ninth, uint8_t data) {
	TXSTAbits.TX9D = ninth;
	TXREG = data;
}

// Check if data waiting on input.
BOOL USARTInputData(void) {
	return PIR1bits.RCIF;
}

// This function must be called only if PIR1bits.RCIF set
nine_data USARTReadByte(void) {
	nine_data received;

	// Framing error bit must be read before RCREG
	received.FERR = RCSTAbits.FERR;

	// error in RCSTA
	received.ninth = RCSTAbits.RX9D; // ninth bit (most significant) must be read before reading RCREG
	received.data = RCREG;
	// RCSTAbits.ADDEN <- set to 1 to ignore messages with 9. bit "0" -- useful for address detecting, clean ADDEN after receiving first word for me
	if (RCSTAbits.OERR) RCSTAbits.CREN = 0; // Overrun Error must be cleared manually
	PIR1bits.RCIF = 0;

	return received;
}
