/*
 * Usart communication library header file
 * (c) Jan Horacek 2016
 */

#ifndef USART_H
#define	USART_H

#include "GenericTypeDefs.h"    
    
typedef struct {
    unsigned char data;
    BOOL ninth;
    BOOL FERR;
} nine_data;
    
void USARTInit(void);
void USARTWriteByte(unsigned ninth, char data);
nine_data USARTReadByte(void);
BOOL USARTInputData(void);

#define XPRESSNET_DIR       LATBbits.LATB6
#define XPRESSNET_OUT       1
#define XPRESSNET_IN        0

#endif	/* USART_H */

