#include "PointH.h"

extern MotorControlType MotorControlDEF;
extern u8 JOY_DATA[USART1_RX_NUM];												//���ҡ������
extern float Remote_Set_Angle;														//ң���ŷ��Ƕ�
extern Whole_PositionType Whole_PositionDEF;						  //ȫ����λ�ṹ��

int main(void)
{
	Systick_Configuration();														//��ʼ���δ�ʱ�ӣ���ʱ����
	NVIC_Configuration();																//���ȼ�����
	CAN1_Configuration();                               //CAN1��ʼ��
  Chassis_MOTOR_Init();																//���̵�����ó�ʼ��
	UART1_Configuration();															//USART1����
	UART2_Configuration();															//USART2����
	TIM7_Configuration();																//Time7��ʼ������ʱ���ʹ���
	//LED_Configuration();																//LED��ʼ��������CAN��ʼ�����棬��CAN��ʼ���и��ţ�ԭ��δ֪
	//Key_All_Init();																			//���а�����ʼ��
	OLED_Init();
	while(1)
	{
		 oled_show();
		 Chassis_Algorithm_Model();											//���̵��ģ��	
	
	}
}




