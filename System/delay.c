#include "delay.h"

void Systick_Configuration(void)
{
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8	9M
}

//////��ʱд��ӿ�

void SysDelay_ms(u16 nms)//72M������,nms<=1864(2^24/9000)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*Sys_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL=0x01;           //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
}

void SysDelay_us(u32 nus)//72M������,nus<=2^24
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*Sys_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01;       //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}
