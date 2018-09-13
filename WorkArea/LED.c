#include "LED.h"
extern USART1_Read_TYPE USART1_Read_DEF;


void LED_Configuration(void)																	//LED初始化，放在CAN初始化后面，对CAN初始化有干扰，原因未知
{
  GPIO_InitTypeDef  GPIO_InitStructure;	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_10;				
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
  GPIO_ResetBits(GPIOB,GPIO_Pin_15);					
  GPIO_ResetBits(GPIOB,GPIO_Pin_14);
  GPIO_ResetBits(GPIOB,GPIO_Pin_13);
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);	
	
	if(USART1_Read_DEF.slam_remote_change==SLAM)
	{
			LED1=1;							//亮
			LED2=0;							//灭
	}
	else if(USART1_Read_DEF.slam_remote_change==REMOTE)
	{
			LED1=0;							//灭
			LED2=1;							//亮
	}
}







