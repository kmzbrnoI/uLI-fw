#line 1 "../isr.c"
#line 1 "../isr.c"



#line 1 "../usart_pic16.h"

#line 23 "../usart_pic16.h"
 


#line 27 "../usart_pic16.h"



#line 33 "../usart_pic16.h"


#line 36 "../usart_pic16.h"


volatile char URBuff[64 ];	
volatile int8_t UQFront;
volatile int8_t UQEnd;

void USARTInit(uint16_t baud_rate);
void USARTWriteChar(char ch);
void USARTWriteString(const char *str);
void USARTWriteLine(const char *str);
void USARTWriteInt(int16_t val, int8_t field_length);
void USARTHandleRxInt();
char USARTReadData();
uint8_t USARTDataAvailable();
void USARTGotoNewLine();
void USARTReadBuffer(char *buff,uint16_t len);
void USARTFlushBuffer();



#line 59 "../usart_pic16.h"

#line 61 "../usart_pic16.h"

#line 4 "../isr.c"


void interrupt ISR(void)
{
    if (RCIE && RCIF) {
        USARTHandleRxInt();
        return;
    }
}