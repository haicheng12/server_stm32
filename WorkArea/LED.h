#ifndef _LED_h_
#define _LED_h_
#include "PointH.h"


#define LED1  PBout(15)	
#define LED2  PBout(14)
#define LED3  PBout(13)

#define Patrol_LED PBout(10)


void LED_Configuration(void);								//LED初始化，放在CAN初始化后面，对CAN初始化有干扰，原因未知



#endif




