#include "show.h"
#include "PointH.h"

extern MotorControlType MotorControlDEF;
extern posture posture1;
void oled_show(void)
{
	//==============================================//
	if(posture1.ActVal[0]<-0.001)
	{
		OLED_ShowString(0,00,"- ");
		OLED_ShowNumber(2,00,abs(posture1.ActVal[0]),5,12);
	}
	else if(posture1.ActVal[0]<0.001&&posture1.ActVal[0]>-0.001)
	{
		OLED_ShowString(0,00,"   0");
	}
	else if(posture1.ActVal[0]>=0.001)
	{
		OLED_ShowString(0,00,"  ");
		OLED_ShowNumber(2,00,posture1.ActVal[0],5,12);
	}
	//==============================================//
	if(posture1.ActVal[3]<-0.1)
	{
		OLED_ShowString(0,20,"- ");
		OLED_ShowNumber(2,20,abs(posture1.ActVal[3]),5,12);
	}
	else if(posture1.ActVal[3]<0.1&&posture1.ActVal[3]>-0.1)
	{
		OLED_ShowString(0,20,"   0");
	}
	else if(posture1.ActVal[3]>=0.1)
	{
		OLED_ShowString(0,20,"  ");
		OLED_ShowNumber(2,20,posture1.ActVal[3],5,12);
	}
	//==============================================//
	if(posture1.ActVal[4]<-0.1)
	{
		OLED_ShowString(0,40,"- ");
		OLED_ShowNumber(2,40,abs(posture1.ActVal[4]),5,12);
	}
	else if(posture1.ActVal[4]<0.1&&posture1.ActVal[4]>-0.1)
	{
		OLED_ShowString(0,40,"   0");
	}
	else if(posture1.ActVal[4]>=0.1)
	{
		OLED_ShowString(0,40,"  ");
		OLED_ShowNumber(2,40,posture1.ActVal[4],5,12);
	}
	//==============================================//

		//=============Ë¢ÐÂ=======================//
		OLED_Refresh_Gram();	
}
