#ifndef __MPU6050__H
#define __MPU6050__H
#include "FreeRTOS.h"  
#include "queue.h"
#include "task.h"
#define MPU6050_QUEUELEN 10

#define MPU6050Queue_Name "MPU6050"
#define MPU6050Queue_Name_Len sizeof(MPU6050Queue_Name)
int MPU6050_Init(void);
typedef struct MPU6050_Date_struct{
	int32_t AccX;
	int32_t AccY;
	int32_t AccZ;
	int32_t GyroX;
	int32_t GyroY;
	int32_t GyroZ;
	char * str_name;
}MPU6050_Struct;

uint8_t MPU6050_Thread_Start( StaticTask_t * PThread,StackType_t * P_Stack,int L_Stack);

QueueHandle_t RetQueueMPU6050(void);
#endif //__MPU6050__H
