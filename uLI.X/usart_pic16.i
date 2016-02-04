#line 1 "../usart_pic16.c"
#line 1 "../usart_pic16.c"

#line 25 "../usart_pic16.c"
 



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

#line 29 "../usart_pic16.c"


void USARTInit(uint16_t baud_rate)
{
    
    UQFront=UQEnd=-1;
        
    
    switch(baud_rate)
    {
     case 9600:
        SPBRG=129;
        break;
     case 19200:
        SPBRG=64;
        break;
     case 28800:
        SPBRG=42;
        break;
     case 33600:
        SPBRG=36;
        break;
    }
    
    TXSTAbits.TX9=0;  
    TXSTAbits.TXEN=1; 
    TXSTAbits.SYNC=0; 
    TXSTAbits.BRGH=1; 

    
    RCSTAbits.SPEN=1;   
    RCSTAbits.RX9=0;    
    RCSTAbits.CREN=1;   
    RCSTAbits.ADDEN=0;  

    
    RCIE=1;
    PEIE=1;

    ei();
}

void USARTWriteChar(char ch)
{
  while(!PIR1bits.TXIF);

  TXREG=ch;
}

void USARTWriteString(const char *str)
{
  while(*str!='\0')
  {
      USARTWriteChar(*str);
      str++;
  }
}

void USARTWriteLine(const char *str)
{
    USARTWriteChar('\r');
    USARTWriteChar('\n');

    USARTWriteString(str);
}

void USARTHandleRxInt()
{
  if(RB1==1)
    RB1=0;
  else
    RB1=1;
  
    
    char data=RCREG;

    
    if(((UQEnd==64 -1) && UQFront==0) || ((UQEnd+1)==UQFront))
    {
        
	UQFront++;
	if(UQFront==64 ) UQFront=0;
    }

    if(UQEnd==64 -1)
        UQEnd=0;
    else
	UQEnd++;

    URBuff[UQEnd]=data;

    if(UQFront==-1) UQFront=0;
    
}

char USARTReadData()
{
    char data;

    
    if(UQFront==-1)
	return 0;

    data=URBuff[UQFront];

    if(UQFront==UQEnd)
    {
        
	
	UQFront=UQEnd=-1;
    }
    else
    {
	UQFront++;

	if(UQFront==64 )
            UQFront=0;
    }

    return data;
}

uint8_t USARTDataAvailable()
{
    if(UQFront==-1) return 0;
    if(UQFront<UQEnd)
	return(UQEnd-UQFront+1);
    else if(UQFront>UQEnd)
	return (64 -UQFront+UQEnd+1);
    else
	return 1;
}

void USARTWriteInt(int16_t val, int8_t field_length)
{
    char str[5]={0,0,0,0,0};
    int8_t i=4,j=0;

    
    if(val<0)
    {
        USARTWriteChar('-');   
        val=val*-1;     
    }
    else
    {
        USARTWriteChar(' ');
    }

    if(val==0 && field_length<1)
    {
        USARTWriteChar('0');
        return;
    }
    while(val)
    {
        str[i]=val%10;
        val=val/10;
        i--;
    }

    if(field_length==-1)
        while(str[j]==0) j++;
    else
        j=5-field_length;


    for(i=j;i<5;i++)
    {
        USARTWriteChar('0'+str[i]);
    }
}

void USARTGotoNewLine()
{
    USARTWriteChar('\r');
    USARTWriteChar('\n');
}

void USARTReadBuffer(char *buff,uint16_t len)
{
	uint16_t i;
	for(i=0;i<len;i++)
	{
		buff[i]=USARTReadData();
	}
}
void USARTFlushBuffer()
{
	while(USARTDataAvailable()>0)
	{
		USARTReadData();
	}
}