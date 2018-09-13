#include "Time.h"	

void TIM7_Configuration(void)//����ѭ���¼���ʱ��ʼ��
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

  TIM_TimeBaseStructure.TIM_Period = TIMPeriod-1;
  TIM_TimeBaseStructure.TIM_Prescaler =720-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;       
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
  TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure);
   
  TIM_UpdateRequestConfig(TIM7,TIM_UpdateSource_Regular);
  TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM7,ENABLE);

}


void TIM7_IRQHandler(void)//�ڲ��ײ�����	   
{
		USART1_DataSend();
	  TIM_ClearITPendingBit(TIM7,TIM_FLAG_Update);			//�жϱ�־λ���
}

