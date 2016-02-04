/* 
 * File:   usart.h
 * Author: JanHoracek
 *
 * Created on 29. ledna 2016, 17:27
 */

#ifndef USART_H
#define	USART_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "GenericTypeDefs.h"    
    
typedef struct {
    unsigned char data;
    BOOL ninth;
} nine_data;
    
void USARTInit(void);
void USARTWriteByte(unsigned ninth, char data);
nine_data USARTReadByte(void);
BOOL USARTInputData(void);

#define XPRESSNET_DIR       LATBbits.LATB6
#define XPRESSNET_OUT       1
#define XPRESSNET_IN        0

#ifdef	__cplusplus
}
#endif

#endif	/* USART_H */

