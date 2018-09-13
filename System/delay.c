#include "delay.h"

void Systick_Configuration(void)
{
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8	9M
}

//////延时写入接口

void SysDelay_ms(u16 nms)//72M条件下,nms<=1864(2^24/9000)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*Sys_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01;           //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
}

void SysDelay_us(u32 nus)//72M条件下,nus<=2^24
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*Sys_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01;       //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}
