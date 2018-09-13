#include "PointH.h"

extern MotorControlType MotorControlDEF;
extern u8 JOY_DATA[USART1_RX_NUM];												//存放摇杆数据
extern float Remote_Set_Angle;														//遥控伺服角度
extern Whole_PositionType Whole_PositionDEF;						  //全场定位结构体

int main(void)
{
	Systick_Configuration();														//初始化滴答时钟，延时计数
	NVIC_Configuration();																//优先级配置
	CAN1_Configuration();                               //CAN1初始化
  Chassis_MOTOR_Init();																//底盘电机配置初始化
	UART1_Configuration();															//USART1配置
	UART2_Configuration();															//USART2配置
	TIM7_Configuration();																//Time7初始化，定时发送串口
	//LED_Configuration();																//LED初始化，放在CAN初始化后面，对CAN初始化有干扰，原因未知
	//Key_All_Init();																			//所有按键初始化
	OLED_Init();
	while(1)
	{
		 oled_show();
		 Chassis_Algorithm_Model();											//底盘电机模型	
	
	}
}




