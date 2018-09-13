#ifndef _Time_H_
#define _Time_H_

#include "PointH.h"

#define TIMPeriod	     1000



void TIM7_Configuration(void);       //周期循环事件定时初始化
void TIM7_IRQHandler(void);					 //Time7中断函数


#endif



