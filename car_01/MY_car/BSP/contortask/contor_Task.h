#ifndef __CONTOR_TASK_H
#define __CONTOR_TASK_H
#include "stm32f1xx_hal.h"
#include "Freertos.h"
#include "gpio.h"
#include "FreeRTOS.h"  
#include "queue.h"


void Contor_Init_Start(void);
uint8_t Contor_Thread_Start(void * PThread,StackType_t * PThread_Stack,int LMotor_Stack);


#endif //__CONTOR_TASK_H
