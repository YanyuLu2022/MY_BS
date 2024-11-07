#include "stdio.h"
#include "string.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h"               // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core
#include <stdarg.h>
#include <stdlib.h>

#include "my_usart.h"



//串口发送时是否要等待数据发送完成,如果启用的话不能再中断里面发送
#define TxSemaphore 0

#if Text_Uart
#define RX_Counter_Max  15
extern void Update_PID(int PID,float P,float I,float D,float V);
#else
#define RX_Counter_Max  4
#endif

//信号量句柄
static SemaphoreHandle_t USART1_RxSemaphore;//接收
static SemaphoreHandle_t USART1_TxSemaphore;//发送
//队列句柄
static QueueHandle_t Uart_Queue;
//结构体名字
static char V_Queue_name[UartQueue_Name_Len] = "Uart";


//CreateSemaphore
uint8_t Rx_Buffer[RX_Counter_Max];//接收缓冲区
uint8_t Rx_str[RX_Counter_Max];//接收到的数据
uint16_t  Rx_Counter = 0;//下标
uint8_t Rx_char = {0};//接收字符
int Rx_state = 0;//接收状态机的状态
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

static Uart_Struct V_Date = {0};//速度更新结构体




/**
返回Uart_Queue句柄地址
*/
QueueHandle_t Return_P_UartStruct(void)
{
	
	return Uart_Queue;

}

//清除缓冲区
void Rx_Buffer_Close(void)
{
	Rx_Counter = 0;
	memset(Rx_Buffer,0,sizeof(Rx_Buffer));
}
//清空接收字符串
void Rx_Str_Close(void)
{
	memset(Rx_str,0,sizeof(Rx_str));
}




//发送函数
/**
void * xhusart 发送的串口
uint8_t *datas 发送的字符串
int len 字符串长度
int timeout_ms 等待时间
*/
static int Stm32_uart_Send (void * xhusart,uint8_t *datas, int len ,int timeout_ms)
{
	
	//串口发送数据
	HAL_UART_Transmit_IT(xhusart,datas,len);
	
	//等待信号量
#if TxSemaphore
	if(pdTRUE != xSemaphoreTake(USART1_TxSemaphore,timeout_ms))return -1;//信号量句柄	
#endif
	return 0;

}

void UsartPrintf(void * pusart,int time, char *fmt, ...) {
		
	static unsigned  char UsartPrintfBuf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf((char *)UsartPrintfBuf, sizeof(UsartPrintfBuf), fmt, ap); 
    va_end(ap);
	Stm32_uart_Send(pusart,(uint8_t*)UsartPrintfBuf,strlen((const char *)UsartPrintfBuf),time);			
    
} 

/*
	串口初始化
*/
void USART_Init( void ){
	//创建二值信号量
	USART1_RxSemaphore = xSemaphoreCreateBinary();
	if(NULL == USART1_RxSemaphore)return;
	USART1_TxSemaphore = xSemaphoreCreateBinary();
	if(NULL == USART1_TxSemaphore)return;
	HAL_UART_Receive_IT(&huart1, &Rx_char,1);
	//创建队列
	Uart_Queue = xQueueCreate(UartQueueLenght,sizeof(Uart_Struct));
	V_Date.str_name = V_Queue_name;
	xSemaphoreTake(USART1_RxSemaphore,0);

}



void Usart1_Thread(void *argument)
{
	//调试模式
#if Text_Uart	
	float P1 = 0.00 , I1 = 0.00  , D1 = 0.00 ;
	float P2 = 0.00 , I2 = 0.00  , D2 = 0.00 ;
	float V1 = 0.0,V2 = 0.0;
#else

#endif


	while(1)
	{
		xSemaphoreTake(USART1_RxSemaphore,portMAX_DELAY);		
//调试模式
#if Text_Uart
		
		if(0x55 == Rx_str[1])
		{
			
			//UsartPrintf(&huart1,1000,"调试模式\r\n");
			
			if(0x01 == Rx_str[2])
			{
				//UsartPrintf(&huart1,1000,"ok\r\n");
				P1 = Rx_str[4];
				P2 = Rx_str[5];
				P2 = P2 >= 99.0 ? 0.99 : (P2 / 100);
				I1 = Rx_str[6];
				I2 = Rx_str[7];
				I2 = I2 >= 99.0 ? 0.99 : (I2 / 100);
				D1 = Rx_str[8];
				D2 = Rx_str[9];	
				D2 = D2 >= 99.0 ? 0.99 : (D2 / 100);
				if(Rx_str[10] != 0XFF)
				{
					V1 = Rx_str[10];
					V2 = Rx_str[11];	
					V2 = V2 >= 99 ? 0.99 : (V2 / 100);				
				}

				P1 = (P1 + P2);
				I1 = (I1 + I2);
				D1 = (D1 + D2);
				V1 = (V1 + V2);
				if(Rx_str[10] == 0xFF)
				{
					P1 = -P1;
				}
				if(Rx_str[11] == 0xFF)
				{
					I1 = -I1;
				}
				if(Rx_str[12] == 0xFF)
				{
					D1 = -D1;
				}
				
				if(0x0A == Rx_str[3])
				{
					if(Rx_str[10] != 0XFF)
					Update_PID(1,P1,I1,D1,V1);
					//UsartPrintf(&huart1,1000,"更新A轮，P:%.2f I:%.2f D:%.2f \r\n",P1,I1,D1);	
				}
				if(0x0B == Rx_str[3])
				{
					if(Rx_str[10] != 0XFF)
					Update_PID(2,P1,I1,D1,V1);
					//UsartPrintf(&huart1,1000,"更新B轮，P:%.2f I:%.2f D:%.2f \r\n",P1,I1,D1);			
				}
				if(0x0C == Rx_str[3])
				{
					if(Rx_str[10] != 0XFF)
					Update_PID(3,P1,I1,D1,V1);
					//UsartPrintf(&huart1,1000,"更新C轮，P:%.2f I:%.2f D:%.2f \r\n",P1,I1,D1);						
				}
				if(0x0D == Rx_str[3])
				{
					if(Rx_str[10] != 0XFF)
					Update_PID(4,P1,I1,D1,V1);
					//UsartPrintf(&huart1,1000,"更新D轮，P:%.2f I:%.2f D:%.2f \r\n",P1,I1,D1);						
				}
				
			}
		}
#else
			//控制模式
			V_Date.Uart_D = Rx_str[1];
			xQueueSend(Uart_Queue,&V_Date,0);
		
	
		
#endif	//调试模式	
		
		Rx_Str_Close();
	}
}
uint8_t Usart_Thread_Start( StaticTask_t * PThread,StackType_t * P_Stack,int L_Stack)
{
	if(NULL !=xTaskCreateStatic(Usart1_Thread,
											"uart1_th",
											L_Stack,
											NULL,
											(osPriority_t) osPriorityNormal,
											P_Stack,
											PThread))
	{
		
		return 1;
	} 
	return 0;
}


void Usart_Close(void * PThread)
{
	vTaskDelete(PThread);//删除任务
	vSemaphoreDelete(USART1_RxSemaphore);
	vSemaphoreDelete(USART1_TxSemaphore);
	vQueueDelete(Uart_Queue);
}



//发送中断
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart == &huart1)
		{
			//释放信号量
			xSemaphoreGiveFromISR(USART1_TxSemaphore,NULL);
		}				
}	
/*
串口接收中断回调函数
*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口3接收完成回调函数
{
	if(huart->Instance == USART1)
	{

		switch(Rx_state)
		{
			case 0:
				if(Rx_char == 0x25)
				{
					Rx_state = 1;
					Rx_Buffer[Rx_Counter++] = Rx_char;					
				}else{
					Rx_Buffer_Close();
					Rx_state = 0;
				}	
			break;
			case 1:
				if(Rx_Counter < RX_Counter_Max-2)
				{
					Rx_Buffer[Rx_Counter++] = Rx_char;
				}else if(Rx_char == 0xA0 ){
					Rx_Buffer[Rx_Counter++] = Rx_char;
					Rx_state = 2;
				}else{
					Rx_Buffer_Close();
					Rx_state = 0;					
				}
			break;			
			case 2:
				if( Rx_char == 0x55)
				{
					Rx_state = 0;
					Rx_Buffer[Rx_Counter] = Rx_char;
					memcpy(Rx_str,Rx_Buffer,sizeof(Rx_Buffer));
					Rx_Buffer_Close();
					xSemaphoreGiveFromISR(USART1_RxSemaphore, NULL);
				}else
				{
					Rx_state = 0;
					Rx_Buffer_Close();
				}
				break;					
			
			
		}
		Rx_char = 0;
		HAL_UART_Receive_IT(&huart1,&Rx_char, 1);	//重新使能中断

	}
}









