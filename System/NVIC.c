#include "NVIC.h"


void NVIC_Configuration(void)												//ϵͳ���ȼ����亯��
{
  NVIC_InitTypeDef NVIC_InitStructure; 

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//�жϷ���2 ��λ��ռ��0-3��  ��λ��Ӧ��0-3��=

	
	//******************************************************	
	
//	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;  
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	//******************************************************	
//	
//	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	//*******************************************************
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	//*****************************************************
//	
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);

////	//*****************************************************
////	
////	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
////  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
////  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
////  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////  NVIC_Init(&NVIC_InitStructure);
////	
////	//*****************************************************
//	
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);

//  //*******************************************************
	
		NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn; 	
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	


		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
 /*���ȼ����飺���ȼ�������Ϊ�˸���ռʽ���ȼ�����Ӧ���ȼ����ж����ȼ��Ĵ����ĸ���λ���������ռ��λ��
               ��ͬһ�����������ȼ�����ֻ���趨һ�� */

 /*��ռ���ȼ�: ��һ���ж�����ִ�ж���һ�����и�����ռ���ȼ����жϵ���ʱ����ռ���ȼ��ߵľͻ���ռCPU����Ȩ
 			         ��ʱ�����жϾ�����Ƕ���жϹ�ϵ��������ͬ��ռ���ȼ����ж�û��Ƕ�׹�ϵ*/

 /*��Ӧ���ȼ����������ȼ�����������ռ���ȼ���ͬ���ж�ͬʱ������������Ӧ���ȼ��ߵ��ж����Ȼ��CPU�Ĵ���Ȩ
               ����жϲ���ͬʱ�������������Ҫ�ȴ�����ִ�е��жϴ�������ܻ����Ӧ*/

}



