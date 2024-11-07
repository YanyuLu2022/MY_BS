#include "stm32f1xx_hal.h"
#include "PID.H"

/**
  * @brief  PID相关参数的初始化
  * @param  PID的结构体指针
*/

void PID_Init(void * P,float Velocity_Kp,float Velocity_Ki,float Velocity_Kd,float limit_value)
{
	PID *p = P;
    p->Kp = Velocity_Kp;
    p->Ki = Velocity_Ki;
    p->Kd = Velocity_Kd;
    p->last_error = 0;
    p->prev_error = 0;
    p->limit = limit_value;
    p->Velocity_return = 0;
}
/**
  * @brief  PID运算
  * @param  targetSpeed 目标速度
  * @param  targetSpeed 当前速度
  * @param  PID结构体
*/
void PID_Formula(float targetSpeed, float currentSpeed, void *P) 
{
    PID *p = P;
    float error = targetSpeed - currentSpeed; // 计算误差

    // 增量式PID公式
    p->Velocity_return += p->Kp * (error - p->last_error) 
                        + p->Ki * error 
                        + p->Kd * (error - 2 * p->last_error + p->prev_error);

    // 更新误差值
    p->prev_error = p->last_error; 
    p->last_error = error;

    // 输出限幅
    if (p->Velocity_return > p->limit) p->Velocity_return = p->limit;
    if (p->Velocity_return < -p->limit) p->Velocity_return = -p->limit;
}

