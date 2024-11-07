#include "string.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"	  // ARM.FreeRTOS::RTOS:Core
#include "task.h"		  // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h" // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"		  // ARM.FreeRTOS::RTOS:Core
#include "tim.h"

#include "contor_Task.h"
#include "motor.h"
#include "my_Usart.h"
#include "MPU6050.h"
// 信号输入类型
typedef enum
{
	mold_CAN,
	mold_BLe
} Car_mold;

static StackType_t Contor_StackBuffer[256];	 // 主任务堆栈空间
static StackType_t Motor_StackBuffer[256];	 // Motor堆栈空间
static StackType_t Usart1_StackBuffer[128];	 // Usart1堆栈空间
static StackType_t MPU6050_StackBuffer[128]; // MPU6050堆栈空间

static StaticTimer_t Timer_StackBuffer; // 定时器堆栈空间

static StaticTask_t Motor_Task;	  // Motor任务句柄
static StaticTask_t Usart1_Task;  // Usart1任务句柄
static StaticTask_t MPU6050_Task; // MPU6050任务句柄
static StaticTask_t Contor_Task;  // 主任务句柄

TimerHandle_t Tim_contor; // 定时器句柄

static QueueHandle_t Uart_Queue;	   // 串口队列句柄
static QueueHandle_t MPU6050_Queue;	   // MPU6050任务句柄
static QueueHandle_t Motor_Queue;	   // 电机队列句柄
static QueueHandle_t g_xQueueStrInput; // 队列集

static uint16_t Car_move = 0;
static uint16_t Mpu6050_model = 0;
static int16_t Car_z = 0; // 目标角度
static int16_t ADD_GZ;
static int move_time = 0; // 移动时间

/*
模式控制
0: 静止模式
1：CAN总线模式
2：蓝牙控制模式
3：自动驾驶模式
*/
static uint8_t Car_mod = 2;
uint8_t Contor_Thread_Start(void *PThread, StackType_t *PThread_Stack, int LMotor_Stack);
uint8_t Timer1_Start(StaticTimer_t *PThread_Stack);

void Contor_Init_Start(void)
{

	MPU6050_Init();
	USART_Init();
	Motor_Init();
	// 获取队列句柄
	Uart_Queue = Return_P_UartStruct(); // 串口队列
	MPU6050_Queue = RetQueueMPU6050();
	Motor_Queue = Re_PMotor_Queue(); // 电机队列
	// 创建队列集,队列集的大小为所有队列最大值相加
	g_xQueueStrInput = xQueueCreateSet(MPU6050_QUEUELEN + UartQueueLenght);
	xQueueAddToSet(Uart_Queue, g_xQueueStrInput);
	xQueueAddToSet(MPU6050_Queue, g_xQueueStrInput);

	Timer1_Start(&Timer_StackBuffer);
	// 创建静态Motor 任务，Motor_Task：句柄。Motor_StackBuffer：存储空间。
	Motor_Thread_Start(&Motor_Task, Motor_StackBuffer, sizeof(Motor_StackBuffer) / sizeof(Motor_StackBuffer[0]));
	Usart_Thread_Start(&Usart1_Task, Usart1_StackBuffer, sizeof(Usart1_StackBuffer) / sizeof(Usart1_StackBuffer[0]));
	MPU6050_Thread_Start(&MPU6050_Task, MPU6050_StackBuffer, sizeof(MPU6050_StackBuffer) / sizeof(MPU6050_StackBuffer[0]));
	Contor_Thread_Start(&Contor_Task, Contor_StackBuffer, sizeof(Contor_StackBuffer) / sizeof(Contor_StackBuffer[0]));
}

void Write_Motor(Car_mold Mold, sMOTOR_COORD *Send_MOTOR_FROM)
{

	switch (Car_move)
	{
	case 10:
		// 停
		Send_MOTOR_FROM->move_model = CAR_STOP;
		Mpu6050_model = 0;
		break;
	case 1:

		Send_MOTOR_FROM->move_model = LEFT;
		Mpu6050_model = 1;
		// 左转
		break;
	case 2:

		Send_MOTOR_FROM->move_model = RIGHT;
		Mpu6050_model = 2;
		// 右转
		break;
	case 3:

		Send_MOTOR_FROM->move_model = Yfornt;
		if (v_0 == Send_MOTOR_FROM->Car_V)
		{
			Send_MOTOR_FROM->Car_V = v_15;
		}
		Mpu6050_model = 3;
		// 上
		break;
	case 4:
		Send_MOTOR_FROM->move_model = Yrear;
		if (v_0 == Send_MOTOR_FROM->Car_V)
		{
			Send_MOTOR_FROM->Car_V = v_15;
		}
		Mpu6050_model = 3;
		// 下
		break;
	case 5:

		// 左
		Send_MOTOR_FROM->move_model = Xfornt;
		if (v_0 == Send_MOTOR_FROM->Car_V)
		{
			Send_MOTOR_FROM->Car_V = v_15;
		}
		Mpu6050_model = 3;
		break;
	case 6:
		// 右
		Send_MOTOR_FROM->move_model = Xrear;
		if (v_0 == Send_MOTOR_FROM->Car_V)
		{
			Send_MOTOR_FROM->Car_V = v_15;
		}
		Mpu6050_model = 3;
		break;
	case 7:
		// 加速

		if (v_45 == Send_MOTOR_FROM->Car_V)
			Send_MOTOR_FROM->Car_V = v_60;
		else if (v_30 == Send_MOTOR_FROM->Car_V)
			Send_MOTOR_FROM->Car_V = v_45;
		else if (v_15 == Send_MOTOR_FROM->Car_V)
			Send_MOTOR_FROM->Car_V = v_30;
		else if (v_0 == Send_MOTOR_FROM->Car_V)
			Send_MOTOR_FROM->Car_V = v_15;

		break;
	case 8:
		// 减速
		if (v_60 == Send_MOTOR_FROM->Car_V)
			Send_MOTOR_FROM->Car_V = v_45;
		else if (v_45 == Send_MOTOR_FROM->Car_V)
			Send_MOTOR_FROM->Car_V = v_30;
		else if (v_30 == Send_MOTOR_FROM->Car_V)
			Send_MOTOR_FROM->Car_V = v_15;
		else if (v_15 == Send_MOTOR_FROM->Car_V)
			Send_MOTOR_FROM->Car_V = v_0;

		break;
	}
	Car_move = 0;
	xQueueSend(Motor_Queue, Send_MOTOR_FROM, 0); // 写入队列
}
void Read_Uart(QueueSetMemberHandle_t IntupQueue_V)
{
	Uart_Struct Uart_DATE;
	xQueueReceive(IntupQueue_V, &Uart_DATE, 0);
	if (Car_mod == 0 || Car_mod == 2)
	{
		Car_mod = 2;
		move_time = 10;
		switch (Uart_DATE.Uart_D)
		{
		case 0x40:
			Car_move = 10;
			break;
		case 0x41:
			// 左转
			Car_move = 1;
			break;
		case 0x42:
			// 右转
			Car_move = 2;
			break;
		case 0x43:
			// 上
			Car_move = 3;
			break;
		case 0x44:
			// 下
			Car_move = 4;
			break;
		case 0x45:
			// 左

			Car_move = 5;
			break;
		case 0x46:
			// 右
			Car_move = 6;
			break;
		case 0x47:
			// 加速
			Car_move = 7;
			break;
		case 0x48:
			// 减速
			Car_move = 8;
			break;
		}
	}
}
void Read_MPU6050(QueueSetMemberHandle_t IntupQueue_V, sMOTOR_COORD *Send_MOTOR_FROM)
{
	static MPU6050_Struct MPU6050_DATE;

	xQueueReceive(IntupQueue_V, &MPU6050_DATE, 0);

	ADD_GZ += MPU6050_DATE.GyroZ;
	ADD_GZ %= 360;
	if (ADD_GZ > 0)
	{
		if (360 - ADD_GZ < ADD_GZ)
		{
			ADD_GZ = -ADD_GZ;
		}
	}
	else if (ADD_GZ < 0)
	{
		if (-360 - ADD_GZ > ADD_GZ)
		{
			ADD_GZ = -ADD_GZ;
		}
	}
	if (Mpu6050_model == 1)
	{
		Car_z = ADD_GZ + (100);
	}
	else if (Mpu6050_model == 2)
	{
		Car_z = ADD_GZ - (100);
	}
	else if (Mpu6050_model == 0)
	{
		Car_z = ADD_GZ;
	}

	ADD_GZ = ADD_GZ - Car_z;

	Send_MOTOR_FROM->G_Z = ADD_GZ;
	Send_MOTOR_FROM->move_model = UPDATE_GZ;
	xQueueSend(Motor_Queue, Send_MOTOR_FROM, 0); // 写入队列
	//	UsartPrintf(&huart1,1000,"%d",ADD_GZ);
}

/*
控制任务：中间任务，读取指令输入目标值
argument:传入参数
*/
void Contor_Thread(void *argument)
{
	static sMOTOR_COORD Send_MOTOR;
	while (1)
	{

		QueueSetMemberHandle_t IntupQueue;
		// 读队列集得到句柄

		IntupQueue = xQueueSelectFromSet(g_xQueueStrInput, portMAX_DELAY);
		// 读句柄得到队列内容,并且处理数据
		if (IntupQueue == Uart_Queue)
		{
			Read_Uart(IntupQueue);
			Write_Motor(mold_CAN, &Send_MOTOR);
		}
		if (IntupQueue == MPU6050_Queue)
		{
			Read_MPU6050(IntupQueue, &Send_MOTOR);
		}
	}
}

// 定时器回调函数
static void vContorTimerFunc(TimerHandle_t xTimer)
{
	if (move_time <= 0)
	{
		Car_mod = 0;
	}
	else
		move_time--;
}

/**

软件定时器初始化
*/
uint8_t Timer1_Start(StaticTimer_t *PThread_Stack)
{
	Tim_contor = xTimerCreateStatic("tim1_contor",
									100,
									pdTRUE,
									NULL,
									vContorTimerFunc,
									PThread_Stack);
	if (NULL != Tim_contor)
	{

		return 1;
	}
	return NULL;
}

/*
创建任务
PThread:任务句柄
PThread_Stack：任务存储空间地址
LMotor_Stack: 任务存储空间大小
*/
uint8_t Contor_Thread_Start(void *PThread, StackType_t *PThread_Stack, int LMotor_Stack)
{
	if (NULL != xTaskCreateStatic(Contor_Thread,
								  "main_mod",
								  LMotor_Stack,
								  NULL,
								  (osPriority_t)osPriorityNormal,
								  PThread_Stack,
								  PThread))
	{

		return 1;
	}
	return NULL;
}
/*
删除任务
*/
