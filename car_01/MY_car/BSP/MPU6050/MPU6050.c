#include "mpu6050.h"
#include "i2c.h"
#include <math.h>
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h"               // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core
#include "cmsis_os.h"


#include "my_usart.h"
//****************************************
// 定义MPU6050内部地址
//****************************************

static char MPU6050_Queue_name[MPU6050Queue_Name_Len] = MPU6050Queue_Name;
static MPU6050_Struct MPU6050_Date = {0};//速度更新结构体
 
#define	MPU6050_SMPLRT_DIV		0x19  // 陀螺仪采样率，典型值：0x07(125Hz)
#define	MPU6050_CONFIG			0x1A  // 低通滤波频率，典型值：0x06(5Hz)
#define	MPU6050_GYRO_CONFIG		0x1B  // 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	MPU6050_ACCEL_CONFIG	0x1C  // 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B //电源管理，典型值：0x00(正常启用)
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75 //IIC地址寄存器(默认数值0x68，只读)



#define MPU6050_I2C_ADDR     0xD0
#define MPU6050_TIMEOUT     500



static I2C_HandleTypeDef *g_pHI2C_MPU6050 = &hi2c1;

static QueueHandle_t g_xQueueMPU6050;//陀螺仪队列句柄
//读取I2C
int MPU6050_ReadRegister(uint8_t reg, uint8_t *pdata)
{
	return HAL_I2C_Mem_Read(g_pHI2C_MPU6050, MPU6050_I2C_ADDR, reg, 1, pdata, 1, MPU6050_TIMEOUT);
}
//写I2C
static int MPU6050_WriteRegister(uint8_t reg, uint8_t data)
{
    uint8_t tmpbuf[2];

    tmpbuf[0] = reg;
    tmpbuf[1] = data;
    
    return HAL_I2C_Master_Transmit(g_pHI2C_MPU6050, MPU6050_I2C_ADDR, tmpbuf, 2, MPU6050_TIMEOUT);
}

/**
  * 函数功能：线性标度变换
  * 入口参数：Sample_Value: 采样回来的原始数值 
  * 入口参数：URV：         量程上限      
  * 入口参数：LRV：         量程下限
  * 返 回 值：变换后的数据
  */
float Scale_Transform(float Sample_Value, float URV, float LRV)
{
    float Data;             //定义用来保存变换后的数据变量
    float Value_L = -32767.0; //定义采样值下限变量   MPU6050寄存器是16位的，最高位是符号位，
    float Value_U = 32767.0;  //定义采样值上限变量   所以寄存器输出范围是-7FFF~7FFF,对应十进制-32767~32767
    
    /* 公式：当前数据 =（采样值 - 采样值下限）/（采样值上限 - 采样值下限）*（量程上限 - 量程下限）+ 量程下限     */
    Data = (Sample_Value - Value_L) / (Value_U - Value_L) * (URV - LRV) + LRV;
           
    return Data;
}
 
/**
  * 函数功能：读取转换后的陀螺仪数值
  * 入口参数：Gyro_Value：陀螺仪转换后保存的变量
  * 返 回 值：无
  */
void MPU6050_GetGyro_Value(float * x,float * y,float * o)
{      

   
    /*开始转换陀螺仪值*/
    *x = Scale_Transform( *x, 2000.0, -2000.0);  //转换X轴
    *y = Scale_Transform( *y, 2000.0, -2000.0);  //转换Y轴
    *o = Scale_Transform( *o, 2000.0, -2000.0);  //转换Z轴
    
}

int MPU6050_Init(void)
{
	g_xQueueMPU6050 =  xQueueCreate(MPU6050_QUEUELEN,sizeof(MPU6050_Struct));//创建队列
	MPU6050_Date.str_name = MPU6050_Queue_name;
    return 0;
}
//读取MPU6050数据
int MPU6050_ReadData(int16_t *pAccX, int16_t *pAccY, int16_t *pAccZ, int16_t *pGyroX, int16_t *pGyroY, int16_t *pGyroZ)
{
	uint8_t datal, datah;
    int err = 0;
	
	err |= MPU6050_ReadRegister(MPU6050_ACCEL_XOUT_H, &datah);
	err |= MPU6050_ReadRegister(MPU6050_ACCEL_XOUT_L, &datal);
	if(pAccX)
        *pAccX = (datah << 8) | datal;
	
	err |= MPU6050_ReadRegister(MPU6050_ACCEL_YOUT_H, &datah);
	err |= MPU6050_ReadRegister(MPU6050_ACCEL_YOUT_L, &datal);
	if(pAccY)
        *pAccY = (datah << 8) | datal;
	
	err |= MPU6050_ReadRegister(MPU6050_ACCEL_ZOUT_H, &datah);
	err |= MPU6050_ReadRegister(MPU6050_ACCEL_ZOUT_L, &datal);
	if(pAccZ)
        *pAccZ = (datah << 8) | datal;


	err |= MPU6050_ReadRegister(MPU6050_GYRO_XOUT_H, &datah);
	err |= MPU6050_ReadRegister(MPU6050_GYRO_XOUT_L, &datal);
	if(pGyroX)
        *pGyroX = (datah << 8) | datal;

	
	err |= MPU6050_ReadRegister(MPU6050_GYRO_YOUT_H, &datah);
	err |= MPU6050_ReadRegister(MPU6050_GYRO_YOUT_L, &datal);
	if(pGyroY)
        *pGyroY = (datah << 8) | datal;
	
	err |= MPU6050_ReadRegister(MPU6050_GYRO_ZOUT_H, &datah);
	err |= MPU6050_ReadRegister(MPU6050_GYRO_ZOUT_L, &datal);
	if(pGyroZ)
        *pGyroZ = (datah << 8) | datal;

    return err;	
}


//数据转换
void MPU6050_ParseDate(int32_t AccX, int32_t AccY, int32_t AccZ, int32_t GyroX, int32_t GyroY, int32_t GyroZ,MPU6050_Struct *MPUdate)
{
	if (MPUdate)
	{
		MPUdate->AccX = (int32_t)(acos((double)((double)(AccX) / 16384.0)) * 57.29577);
		MPUdate->AccY = (int32_t)(acos((double)((double)(AccY) / 16384.0)) * 57.29577);
		MPUdate->AccZ = (int32_t)(acos((double)((double)(AccZ) / 16384.0)) * 57.29577);		
	}
	
}
void MPU6050_Thread(void *argument)
{
    int16_t AccZ;
	float g_Z;
	uint8_t red;
//	static int16_t AccO;
	MPU6050_WriteRegister(MPU6050_PWR_MGMT_1, 0x00);	//解除休眠状态
	MPU6050_WriteRegister(MPU6050_PWR_MGMT_2, 0x00);
	MPU6050_WriteRegister(MPU6050_SMPLRT_DIV, 0x09);
	MPU6050_WriteRegister(MPU6050_CONFIG, 0x06);
	MPU6050_WriteRegister(MPU6050_GYRO_CONFIG, 0x18);
	MPU6050_WriteRegister(MPU6050_ACCEL_CONFIG, 0x18);
	while(1)
	{
		
//获取互斥量
		red = MPU6050_ReadData(NULL, NULL, NULL,NULL,NULL ,&AccZ );

//释放互斥量   
		if(0 == red)
		{
			g_Z = AccZ;
			
			MPU6050_GetGyro_Value(0,0,&g_Z);
			g_Z = (g_Z +3.6) * 0.3;
			if(((g_Z > 0.1) && (g_Z < 1)))
			{
				g_Z = 1;
			}else if(((g_Z < -0.1) && (g_Z > -1)))
			{
				g_Z = -1;			
			}
			MPU6050_Date.GyroZ = (int16_t)(g_Z);
//			UsartPrintf(&huart1,1000,"%d\r\n",MPU6050_Date.GyroZ);			
			xQueueSend(g_xQueueMPU6050,&MPU6050_Date,0);
			
		}
     
		vTaskDelay(10);
		
	}
}
QueueHandle_t RetQueueMPU6050(void)//返回句柄
{ 
	return g_xQueueMPU6050;
}

uint8_t MPU6050_Thread_Start( StaticTask_t * PThread,StackType_t * P_Stack,int L_Stack)
{
	if(NULL !=xTaskCreateStatic(MPU6050_Thread,
											"MPU6050",
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
