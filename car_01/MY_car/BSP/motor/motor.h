#ifndef _MOTOR__H_
#define _MOTOR__H_
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "FreeRTOS.h"  
#include "queue.h"

#define Text_MOTOR   0
//队列

#define MOTOR_QueueLength 10
typedef enum 
{
       v_0,v_15,v_30,v_45,v_60
}XYZ_V;
typedef enum
{
	  CAR_STOP = 0,Xfornt,Yfornt,Xrear,Yrear,RIGHT,LEFT,UPDATE_GZ
}XYZ_Dire;


typedef struct 
{
	
	 XYZ_V Car_V;
	 XYZ_Dire move_model;
	 int16_t G_Z;//偏移角度
	
//	 int16_t Vy;//y坐标
//	 int16_t Vx;//x坐标

}sMOTOR_COORD;

void Motor_Init(void);
QueueHandle_t Re_PMotor_Queue(void);//获取队列地址
uint8_t Motor_Thread_Start( StaticTask_t * PThread,StackType_t * P_Stack,int L_Stack);
void Motor_Close(void * PThread);

#endif 
