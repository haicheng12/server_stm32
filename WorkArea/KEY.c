#include "KEY.h"


void Key_Patrol_Init(void)			//巡逻按键初始化
{
	GPIO_InitTypeDef  GPIO_InitStructure;	
	
	AFIO->MAPR&=~(7<<24);
	AFIO->MAPR|=2<<24;				//关闭JITA，开启SW
	
  RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
  GPIO_ResetBits(GPIOA,GPIO_Pin_15);
	
}



void Key_All_Init(void)					//所有按键初始化
{
	Key_Patrol_Init();
}



