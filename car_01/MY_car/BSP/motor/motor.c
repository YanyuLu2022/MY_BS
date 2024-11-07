#include "string.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"	  // ARM.FreeRTOS::RTOS:Core
#include "task.h"		  // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h" // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"		  // ARM.FreeRTOS::RTOS:Core
#include "tim.h"

#include "motor.h"
#include "PID.h"

#define A1_OUT_GPIO GPIOC
#define A2_OUT_GPIO GPIOC
#define B1_OUT_GPIO GPIOB
#define B2_OUT_GPIO GPIOB
#define C1_OUT_GPIO GPIOA
#define C2_OUT_GPIO GPIOB
#define D1_OUT_GPIO GPIOA
#define D2_OUT_GPIO GPIOB

#define A1_OUT_GPIO_PIN GPIO_PIN_14
#define A2_OUT_GPIO_PIN GPIO_PIN_15
#define B1_OUT_GPIO_PIN GPIO_PIN_8
#define B2_OUT_GPIO_PIN GPIO_PIN_9
#define C1_OUT_GPIO_PIN GPIO_PIN_8
#define C2_OUT_GPIO_PIN GPIO_PIN_5
#define D1_OUT_GPIO_PIN GPIO_PIN_15
#define D2_OUT_GPIO_PIN GPIO_PIN_3
// 定时器
#define P_A__hTIME &htim3
#define P_B__hTIME &htim3
#define P_C__hTIME &htim3
#define P_D__hTIME &htim3
#define Motor_TIME_nvic &htim2

#define A_TIM_channe TIM_CHANNEL_1
#define B_TIM_channe TIM_CHANNEL_2
#define C_TIM_channe TIM_CHANNEL_3
#define D_TIM_channe TIM_CHANNEL_4

// 小车长宽与轮子半径
#define Car_Long 23.0
#define Car_Wide 21.5
#define Wheel_R 4.85

// 轮子直径cm
#define WHEEL_DIAMETER 9.6
#define WHEEL_Pai 3.1416
// 编码值
#define Maximum_count_A 450
#define Maximum_count_B 450
#define Maximum_count_C 450
#define Maximum_count_D 450
// 限幅
#define max_pwm 500
// 50%一半占空比的速度cm/s
#define PWM50_speed_A 140.74
#define PWM50_speed_B 159.29
#define PWM50_speed_C 146.76
#define PWM50_speed_D 131.00

static int16_t omega = 0;
static int16_t Vy = 0;
static int16_t Vx = 0;
// 编码器脉冲数
int Encoder_A = 0;
int Encoder_B = 0;
int Encoder_C = 0;
int Encoder_D = 0;

static PID PID_data_A = {0};
static PID PID_data_B = {0};
static PID PID_data_C = {0};
static PID PID_data_D = {0};
static float CarV = 0;
// 互斥量句柄
static SemaphoreHandle_t Motor_Semaphore;
// 队列句柄
static QueueHandle_t Motor_Queue;
static uint8_t CMOD = 0;

// 调试
// #include "usart.h"
// void UsartPrintf(void * pusart,int time, char *fmt, ...);
#if Text_MOTOR
#include "usart.h"
void UsartPrintf(void *pusart, int time, char *fmt, ...);
float TEXT_V = 000.00;
#endif
QueueHandle_t Re_PMotor_Queue(void) // 返回队列地址
{
	return Motor_Queue;
}
void Address_Data(sMOTOR_COORD DATA) // 计算轮子输出目标值
{
	//	xSemaphoreTake(Motor_Semaphore,pdTRUE);//获取互斥量上锁，优先级继承
	if (DATA.Car_V == v_0)
	{
		CarV = 0;
		CMOD = 0;
	}
	else if (DATA.Car_V == v_15)
		CarV = 15;
	else if (DATA.Car_V == v_30)
		CarV = 30;
	else if (DATA.Car_V == v_45)
		CarV = 45;
	else if (DATA.Car_V == v_60)
		CarV = 60;
	switch (DATA.move_model)
	{

	case CAR_STOP:
		CMOD = 0;
		break;
	case Xfornt:
		CMOD = 1;
		break;
	case Yfornt:
		CMOD = 2;
		break;
	case Xrear:
		CMOD = 3;
		break;
	case Yrear:
		CMOD = 4;
		break;
	case RIGHT:
		CMOD = 5;
		omega = DATA.G_Z * 5;
		break;
	case LEFT:
		CMOD = 6;
		omega = DATA.G_Z * 5;
		break;
	case UPDATE_GZ:
		omega = DATA.G_Z;
		if (DATA.Car_V == v_60)
		{
			omega  *= 0.6;
		}else if (DATA.Car_V == v_45)
		{
			omega  *= 0.45;			
		}else if (DATA.Car_V == v_30)
		{
			omega  *= 0.3;				
		}else if (DATA.Car_V == v_15)
		{
			omega  *= 0.15;				
		}
		

		break;
	}

	if (0 == CMOD)
	{
		Vx = 0;
		Vy = 0;
	}
	else if (1 == CMOD)
	{
		Vx = CarV;
		Vy = 0;
	}
	else if (2 == CMOD)
	{
		Vx = 0;
		Vy = CarV;
	}
	else if (3 == CMOD)
	{
		Vx = -CarV;
		Vy = 0;
	}
	else if (4 == CMOD)
	{
		Vx = 0;
		Vy = -CarV;
	}
	else if (5 == CMOD)
	{
		Vx = 0;
		Vy = 0;
	}
	else if (6 == CMOD)
	{
		Vx = 0;
		Vy = 0;
	}
	//	xSemaphoreGive(Motor_Semaphore);//释放互斥量解锁
}
/*
调试代码
需要使用时掉用UART并且将Text_MOTOR设置为1
*/
#if Text_MOTOR
void Update_PID(int PID, float P, float I, float D, float V)
{
	if (1 == PID)
	{
		PID_data_A.Kp = P;
		PID_data_A.Ki = I;
		PID_data_A.Kd = D;
		TEXT_V = V;
	}
	if (2 == PID)
	{
		PID_data_B.Kp = P;
		PID_data_B.Ki = I;
		PID_data_B.Kd = D;
		TEXT_V = V;
	}
	if (3 == PID)
	{
		PID_data_C.Kp = P;
		PID_data_C.Ki = I;
		PID_data_C.Kd = D;
		TEXT_V = V;
	}
	if (4 == PID)
	{
		PID_data_D.Kp = P;
		PID_data_D.Ki = I;
		PID_data_D.Kd = D;
		TEXT_V = V;
	}
}
#endif
/**
轮子方向1正转2反转
*/
void left_front(uint8_t i)
{
	if (i == 1)
	{
		HAL_GPIO_WritePin(A1_OUT_GPIO, A1_OUT_GPIO_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(A2_OUT_GPIO, A2_OUT_GPIO_PIN, GPIO_PIN_RESET);
	}
	else if (i == 2)
	{
		HAL_GPIO_WritePin(A1_OUT_GPIO, A1_OUT_GPIO_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A2_OUT_GPIO, A2_OUT_GPIO_PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(A1_OUT_GPIO, A1_OUT_GPIO_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A2_OUT_GPIO, A2_OUT_GPIO_PIN, GPIO_PIN_RESET);
	}
}
void right_front(uint8_t i)
{
	if (i == 1)
	{
		HAL_GPIO_WritePin(B1_OUT_GPIO, B1_OUT_GPIO_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(B2_OUT_GPIO, B2_OUT_GPIO_PIN, GPIO_PIN_RESET);
	}
	else if (i == 2)
	{
		HAL_GPIO_WritePin(B1_OUT_GPIO, B1_OUT_GPIO_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B2_OUT_GPIO, B2_OUT_GPIO_PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(B1_OUT_GPIO, B1_OUT_GPIO_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B2_OUT_GPIO, B2_OUT_GPIO_PIN, GPIO_PIN_RESET);
	}
}
void left_behind(uint8_t i)
{
	if (i == 1)
	{
		HAL_GPIO_WritePin(C1_OUT_GPIO, C1_OUT_GPIO_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(C2_OUT_GPIO, C2_OUT_GPIO_PIN, GPIO_PIN_RESET);
	}
	else if (i == 2)
	{
		HAL_GPIO_WritePin(C1_OUT_GPIO, C1_OUT_GPIO_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(C2_OUT_GPIO, C2_OUT_GPIO_PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(C1_OUT_GPIO, C1_OUT_GPIO_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(C2_OUT_GPIO, C2_OUT_GPIO_PIN, GPIO_PIN_RESET);
	}
}
void right_behind(uint8_t i)
{
	if (i == 1)
	{
		HAL_GPIO_WritePin(D1_OUT_GPIO, D1_OUT_GPIO_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(D2_OUT_GPIO, D2_OUT_GPIO_PIN, GPIO_PIN_RESET);
	}
	else if (i == 2)
	{
		HAL_GPIO_WritePin(D1_OUT_GPIO, D1_OUT_GPIO_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D2_OUT_GPIO, D2_OUT_GPIO_PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(D1_OUT_GPIO, D1_OUT_GPIO_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D2_OUT_GPIO, D2_OUT_GPIO_PIN, GPIO_PIN_RESET);
	}
}

/**
更新占空比函数
输入A-D轮速度
*/

static void Motor_OUT(float A_S, float B_S,
					  float C_S, float D_S)
{

	// 计算占空比

#if Text_MOTOR
	int A = 500;
	int B = 500;
	int C = 500;
	int D = 500;

#else

	int A = (A_S / (PWM50_speed_A * 2)) * 1000;
	int B = (B_S / (PWM50_speed_B * 2)) * 1000;
	int C = (C_S / (PWM50_speed_C * 2)) * 1000;
	int D = (D_S / (PWM50_speed_D * 2)) * 1000;

#endif

	// 限幅
	A = A > max_pwm ? max_pwm : A;
	A = A < -max_pwm ? -max_pwm : A;
	B = B > max_pwm ? max_pwm : B;
	B = B < -max_pwm ? -max_pwm : B;
	C = C > max_pwm ? max_pwm : C;
	C = C < -max_pwm ? -max_pwm : C;
	D = D > max_pwm ? max_pwm : D;
	D = D < -max_pwm ? -max_pwm : D;
	if (A == 0)
	{
		// A电机转向停止 PWM输出值设置为0
		__HAL_TIM_SET_COMPARE(P_A__hTIME, A_TIM_channe, 0);
		left_front(0);
	}
	else if (A > 0)
	{
		// 1修改极性
		left_front(1);
		// 2设置PWM占空比
		__HAL_TIM_SET_COMPARE(P_A__hTIME, A_TIM_channe, A);
	}
	else
	{
		// 1修改极性
		left_front(2);
		// 2设置PWM占空比
		__HAL_TIM_SET_COMPARE(P_A__hTIME, A_TIM_channe, -A);
	}

	if (B == 0)
	{
		// A电机转向停止 PWM输出值设置为0
		__HAL_TIM_SET_COMPARE(P_B__hTIME, B_TIM_channe, 0);
		right_front(0);
	}
	else if (B > 0)
	{
		// 1修改极性
		right_front(1);
		// 2设置PWM占空比
		__HAL_TIM_SET_COMPARE(P_B__hTIME, B_TIM_channe, B);
	}
	else
	{
		// 1修改极性
		right_front(2);
		// 2设置PWM占空比
		__HAL_TIM_SET_COMPARE(P_B__hTIME, B_TIM_channe, -B);
	}
	if (C == 0)
	{
		// A电机转向停止 PWM输出值设置为0
		__HAL_TIM_SET_COMPARE(P_C__hTIME, C_TIM_channe, 0);
		left_behind(0);
	}
	else if (C > 0)
	{
		// 1修改极性
		left_behind(1);
		// 2设置PWM占空比
		__HAL_TIM_SET_COMPARE(P_C__hTIME, C_TIM_channe, C);
	}
	else
	{
		// 1修改极性
		left_behind(2);
		// 2设置PWM占空比
		__HAL_TIM_SET_COMPARE(P_C__hTIME, C_TIM_channe, -C);
	}

	if (D == 0)
	{
		// A电机转向停止 PWM输出值设置为0
		__HAL_TIM_SET_COMPARE(P_D__hTIME, D_TIM_channe, 0);
		right_behind(0);
	}
	else if (D > 0)
	{
		// 1修改极性
		right_behind(1);
		// 2设置PWM占空比
		__HAL_TIM_SET_COMPARE(P_D__hTIME, D_TIM_channe, D);
	}
	else
	{
		// 1修改极性
		right_behind(2);
		// 2设置PWM占空比
		__HAL_TIM_SET_COMPARE(P_D__hTIME, D_TIM_channe, -D);
	}
}

void Motor_Init(void)
{

	Motor_Queue = xQueueCreate(MOTOR_QueueLength, sizeof(sMOTOR_COORD)); // 创建队列
	Motor_Semaphore = xSemaphoreCreateMutex();							 // 创建互斥量
	if (NULL == Motor_Semaphore)
		return;

	HAL_TIM_PWM_Start(P_A__hTIME, A_TIM_channe);
	HAL_TIM_PWM_Start(P_B__hTIME, B_TIM_channe);
	HAL_TIM_PWM_Start(P_C__hTIME, C_TIM_channe);
	HAL_TIM_PWM_Start(P_D__hTIME, D_TIM_channe);
	HAL_TIM_Base_Start_IT(Motor_TIME_nvic);

	PID_Init(&PID_data_A, 0.25, 0.1, 0, PWM50_speed_A * 2);
	PID_Init(&PID_data_B, 0.25, 0.1, 0, PWM50_speed_B * 2); // 0.5,0.38,0.64  0.22,0.16,0.037
	PID_Init(&PID_data_C, 0.25, 0.1, 0, PWM50_speed_C * 2);
	PID_Init(&PID_data_D, 0.25, 0.1, 0, PWM50_speed_D * 2);
}

/*
Maximum_count 一圈的编码值
WHEEL_DIAMETER 9.6  轮子的直径，单位为 cm
pulse_count 计算一次的编码器脉冲数
*/
// 将编码值转换为电机速度，每10ms执行一次
float calculate_speed(int pulse_count, int Maximum_count)
{
	float l = pulse_count * (WHEEL_DIAMETER * WHEEL_Pai) / Maximum_count;

	return l * 100.0; // cm/s
}

// 定时器中断,读取速度并且更新PWM值
void Motor_Timer_ISR(void)
{
	//
	portDISABLE_INTERRUPTS();
//	UsartPrintf(&huart1,1000," A:%d B:%d C:%d D:%d \r\n",Encoder_A,Encoder_B,Encoder_C,Encoder_D);
#if 1
	// 读取目标值
	// omega = omega * (Car_Long + Car_Wide) / Wheel_R;
	// omega  *= 0.6;
	if (CMOD == 0 || CMOD == 5 || CMOD == 6)
	{
		omega = omega > (30) ? (30) : omega;
		omega = omega < -(30) ? -(30) : omega;
	}
	else
	{
		omega = omega > (CarV) ? (CarV) : omega;
		omega = omega < -(CarV) ? -(CarV) : omega;
	}

	float left_front_TargetSpeed = -(Vy - Vx);	//+ omega);
	float right_front_TargetSpeed = -(Vy + Vx); //+ omega);
	float left_behind_TargetSpeed = (Vy + Vx);	//- omega);
	float right_behind_TargetSpeed = (Vy - Vx); //- omega);
	if (1 == CMOD)
	{
		//左
		left_front_TargetSpeed += +omega;
		right_front_TargetSpeed += -omega;
		left_behind_TargetSpeed += -omega;
		right_behind_TargetSpeed += +omega;
	}
	else if (2 == CMOD)
	{
		//上
		left_front_TargetSpeed += -omega;
		right_front_TargetSpeed += -omega;
		left_behind_TargetSpeed += -omega;
		right_behind_TargetSpeed += -omega;
	}
	else if (3 == CMOD)
	{
		//右
		left_front_TargetSpeed += -omega;
		right_front_TargetSpeed += +omega;
		left_behind_TargetSpeed += +omega;
		right_behind_TargetSpeed += -omega;
	}
	else if (4 == CMOD)
	{
		//下
		left_front_TargetSpeed += omega;
		right_front_TargetSpeed += omega;
		left_behind_TargetSpeed += omega;
		right_behind_TargetSpeed += omega;
	}
	else if (5 == CMOD)
	{
		left_front_TargetSpeed += -omega;
		right_front_TargetSpeed += -omega;
		left_behind_TargetSpeed += -omega;
		right_behind_TargetSpeed += -omega;
	}
	else if (6 == CMOD)
	{
		left_front_TargetSpeed += -omega;
		right_front_TargetSpeed += -omega;
		left_behind_TargetSpeed += -omega;
		right_behind_TargetSpeed += -omega;
	}
//		UsartPrintf(&huart1,1000," omega:%d \r\n",omega);
#endif

#if 1
	// 读取编码器的值，获取读取速度
	float a_v = calculate_speed(Encoder_A, Maximum_count_A);
	float b_v = calculate_speed(Encoder_B, Maximum_count_B);
	float c_v = calculate_speed(Encoder_C, Maximum_count_C);
	float d_v = calculate_speed(Encoder_D, Maximum_count_D);
	Encoder_A = 0;
	Encoder_B = 0;
	Encoder_C = 0;
	Encoder_D = 0;
#endif
#if 1

	//		使用PID算法后得到输出值
	PID_Formula(left_front_TargetSpeed, a_v, &PID_data_A);
	PID_Formula(right_front_TargetSpeed, b_v, &PID_data_B);
	PID_Formula(left_behind_TargetSpeed, c_v, &PID_data_C);
	PID_Formula(right_behind_TargetSpeed, d_v, &PID_data_D);

//		将获取到的值输出到电机里面
#endif
	//	Motor_OUT(left_front_TargetSpeed,right_front_TargetSpeed,left_behind_TargetSpeed,right_behind_TargetSpeed);

	Motor_OUT(PID_data_A.Velocity_return, PID_data_B.Velocity_return, PID_data_C.Velocity_return, PID_data_D.Velocity_return);

	portENABLE_INTERRUPTS();
}
/**
//电机任务

*/

void Motor_Thread(void *argument)
{

	// 更新数据的结构体
	sMOTOR_COORD coord;
	while (1)
	{

		xQueueReceive(Motor_Queue, &coord, portMAX_DELAY);
		// 读到数据
		Address_Data(coord);
	}
}
// 关闭任务
void Motor_Close(void *PThread)
{
	// str_Motor *s_speed = argument;
	vTaskDelete(PThread);	   // 删除任务
	vQueueDelete(Motor_Queue); // 删除队列
	// 关闭定时器
	HAL_TIM_Base_Stop_IT(Motor_TIME_nvic);
	// 关闭pwm;
	HAL_TIM_PWM_Stop(P_A__hTIME, A_TIM_channe);
	HAL_TIM_PWM_Stop(P_B__hTIME, B_TIM_channe);
	HAL_TIM_PWM_Stop(P_C__hTIME, C_TIM_channe);
	HAL_TIM_PWM_Stop(P_D__hTIME, D_TIM_channe);
	// 关闭外部中断
	__HAL_GPIO_EXTI_CLEAR_IT(EXTI0_IRQn);
	__HAL_GPIO_EXTI_CLEAR_IT(EXTI1_IRQn);
	__HAL_GPIO_EXTI_CLEAR_IT(EXTI3_IRQn);
	__HAL_GPIO_EXTI_CLEAR_IT(EXTI9_5_IRQn);
	omega = 0;
	Vy = 0;
	Vx = 0;
	// 编码器脉冲数
	Encoder_A = 0;
	Encoder_B = 0;
	Encoder_C = 0;
	Encoder_D = 0;
	memset(&PID_data_A, 0, sizeof(PID)); // 清空结构体成员数据
	memset(&PID_data_B, 0, sizeof(PID)); // 清空结构体成员数据
	memset(&PID_data_C, 0, sizeof(PID)); // 清空结构体成员数据
	memset(&PID_data_D, 0, sizeof(PID)); // 清空结构体成员数据
}
// 创建任务

uint8_t Motor_Thread_Start(StaticTask_t *PThread, StackType_t *P_Stack, int L_Stack)
{

	if (NULL != xTaskCreateStatic(Motor_Thread,
								  "motor_mod",
								  L_Stack,
								  NULL,
								  (osPriority_t)osPriorityNormal,
								  P_Stack,
								  PThread))
	{

		return 1;
	}
	return 0;
}

// 累计编码值
void IQRReadA_Motor(GPIO_TypeDef *Motor_GPIO_1,
					uint16_t Motor_PIN_1,
					GPIO_TypeDef *Motor_GPIO_2,
					uint16_t Motor_PIN_2,
					int *Encoder)
{
	if (GPIO_PIN_SET == HAL_GPIO_ReadPin(Motor_GPIO_1, Motor_PIN_1))
	{
		if (GPIO_PIN_SET == HAL_GPIO_ReadPin(Motor_GPIO_2, Motor_PIN_2)) // 在2引脚高电平时触发下降沿
		{
			*Encoder = (*Encoder) + 1;
		}
		else
		{
			// 在2为低电平时触发下降沿
			*Encoder = (*Encoder) - 1;
		}
	}
	else
	{
		if (GPIO_PIN_SET == HAL_GPIO_ReadPin(Motor_GPIO_2, Motor_PIN_2)) // 在2引脚高电平时触发下降沿
		{
			*Encoder = (*Encoder) - 1;
		}
		else
		{
			// 在2为低电平时触发下降沿
			*Encoder = (*Encoder) + 1;
		}
	}
}
void IQRReadB_Motor(GPIO_TypeDef *Motor_GPIO_1,
					uint16_t Motor_PIN_1,
					GPIO_TypeDef *Motor_GPIO_2,
					uint16_t Motor_PIN_2,
					int *Encoder)
{
	if (GPIO_PIN_SET == HAL_GPIO_ReadPin(Motor_GPIO_1, Motor_PIN_1)) // 上升沿
	{
		return;
	}
	else // 下降沿
	{

		if (GPIO_PIN_SET == HAL_GPIO_ReadPin(Motor_GPIO_2, Motor_PIN_2)) // 在2引脚高电平时触发下降沿
		{
			*Encoder = (*Encoder) - 1;
		}
		else
		{
			// 在2为低电平时触发下降沿
			*Encoder = (*Encoder) + 1;
		}
	}
}

// 外部中断，跟新编码值
/*
编码器A1	PB12
编码器A2	PB13
编码器B1	PA0
编码器B2	PA1
编码器C1	PA4
编码器C2	PA5
编码器D1	PB14
编码器D2	PB15
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	portDISABLE_INTERRUPTS();
	switch (GPIO_Pin)
	{
	case GPIO_PIN_0: // B1 PA0
	{
		// if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))//下降沿
		IQRReadA_Motor(GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1, &Encoder_B);

		break;
	}
	case GPIO_PIN_1: // B2 PA1
	{
		//			if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1))//下降沿
		//			IQRReadB_Motor(GPIOA,GPIO_PIN_1,GPIOA,GPIO_PIN_0,&Encoder_B);
		break;
	}
	case GPIO_PIN_4: // C1 PA4
	{
		// if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4))//下降沿
		IQRReadA_Motor(GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5, &Encoder_C);
		break;
	}
	case GPIO_PIN_5: // C2 PA5
	{
		//			if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5))//下降沿
		//			IQRReadB_Motor(GPIOA,GPIO_PIN_5,GPIOA,GPIO_PIN_4,&Encoder_C);
		break;
	}

	case GPIO_PIN_12: // A1 PB12
	{
		// if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12))//下降沿
		IQRReadA_Motor(GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, &Encoder_A);
		break;
	}
	case GPIO_PIN_13: // A2 PB13
	{
		//			if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13))//下降沿
		//			IQRReadB_Motor(GPIOB,GPIO_PIN_13,GPIOB,GPIO_PIN_12,&Encoder_A);
		break;
	}

	case GPIO_PIN_14: // D1 PB14
	{
		// if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14))//下降沿
		IQRReadA_Motor(GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, &Encoder_D);
		break;
	}
	case GPIO_PIN_15: // D2 PB15
	{
		//			if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15))//下降沿
		//			IQRReadB_Motor(GPIOB,GPIO_PIN_15,GPIOB,GPIO_PIN_14,&Encoder_D);
		break;
	}
	default:
		break;
	}
	portENABLE_INTERRUPTS();
}
