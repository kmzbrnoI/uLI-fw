/* Usart communication library header file */

#ifndef USART_H
#define USART_H

#include <inttypes.h>
#include <stdbool.h>
#include "GenericTypeDefs.h"

typedef struct {
	uint8_t data;
	bool ninth;
	bool FERR;
} nine_data;

void USARTInit(void);
void USARTWriteByte(bool ninth, uint8_t data);
nine_data USARTReadByte(void);
bool USARTInputData(void);
void USARTEnableReceiveInterrupt(void);
void USARTDisableReceiveInterrupt(void);

#define XPRESSNET_DIR       LATBbits.LATB6
#define XPRESSNET_OUT       1
#define XPRESSNET_IN        0

#endif /* USART_H */
