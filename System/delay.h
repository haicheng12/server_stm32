#ifndef _delay_h_
#define _delay_h_
#include "stm32f10x.h"

#define Sys_us  72/8            
#define Sys_ms  Sys_us*1000


void Systick_Configuration(void);
void SysDelay_ms(u16 nms);//72M条件下,nms<=1864(2^24/9000)
void SysDelay_us(u32 nus);//72M条件下,nus<=2^24

#endif

