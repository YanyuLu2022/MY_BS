#ifndef __USART_H_
#define __USART_H_
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "FreeRTOS.h"  
#include "queue.h"
#include "usart.h"
//是否开启调试代码
#define Text_Uart   0

#define UartQueue_Name "Uart"
#define UartQueue_Name_Len sizeof(UartQueue_Name)
//队列长度
#define UartQueueLenght 10

typedef struct
{

	uint16_t Uart_D;
	char * str_name;	

}Uart_Struct;
void USART_Init( void );
uint8_t Usart_Thread_Start( StaticTask_t * PThread,StackType_t * P_Stack,int L_Stack);
void UsartPrintf(void * pusart,int time, char *fmt, ...);
QueueHandle_t Return_P_UartStruct(void);
void Usart_Close(void * PThread);

#endif // __USART_H_

