#include "Algorithm.h"

/************************extern*************************/

extern u8 JOY_DATA[USART1_RX_NUM];							//���ҡ������
extern u8 PAD_DATA[USART1_RX_NUM];							//���PAD����
extern u8 UART1_TxBuffer[USART1_TX_NUM];

extern USART1_Read_TYPE USART1_Read_DEF;

/************************extern*************************/

/************************FLAHS**************************/


PID_AbsoluteType Point_Angle_PID;										//�㵽��ĽǶ�PID

float Remote_Set_Angle=0;														//ң���ŷ��Ƕ�
	

Whole_PositionType Whole_PositionDEF;															//ȫ����λ�ṹ��
ACC_DEC_TYPE ACC_DEC_TYPEDEF;																			//���μӼ����㷨�Ż�





//����ʽPID�㷨
void PID_AbsoluteMode(PID_AbsoluteType* PID)
{

 if(PID->kp      < 0)    PID->kp      = -PID->kp;
 if(PID->ki      < 0)    PID->ki      = -PID->ki;
 if(PID->kd      < 0)    PID->kd      = -PID->kd;
 if(PID->errILim < 0)    PID->errILim = -PID->errILim;

 PID->errP = PID->errNow;  //��ȡ���ڵ�������kp����

 PID->errI += PID->errNow; //�����֣�����ki����

 if(PID->errILim != 0)	   //΢�����޺�����
 {
  if(     PID->errI >  PID->errILim)    PID->errI =  PID->errILim;
  else if(PID->errI < -PID->errILim)    PID->errI = -PID->errILim;
 }
 
 PID->errD = PID->errNow - PID->errOld;//���΢�֣�����kd����

 PID->errOld = PID->errNow;	//�������ڵ����
 
 PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//�������ʽPID���

}


MotorControlType MotorControlDEF;											//���̵������



void Chassis_MOTOR_Init(void)													//���̵��ģʽ��ʼ��
{
	
	SysDelay_ms(1000);
	CAN_RoboModule_DRV_Reset(0,0);                      //��1��1�����������и�λ
	SysDelay_ms(100);                                  //���͸�λָ������ʱ����Ҫ�У��ȴ���������λ��ϡ�
	CAN_RoboModule_DRV_Config(0,0,100,0);               //����Ϊ1s����һ������
	SysDelay_ms(100);
	CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Mode);  //ѡ������ٶȻ�ģʽ
	SysDelay_ms(500);
	
	MotorControlDEF.Vx=0;
	MotorControlDEF.Vy=0;
	MotorControlDEF.Vspin=0;
	
}


/***********************
����ִ��λ��
	main������
***********************/
float slam_vx,slam_vy,slam_vz;
void Chassis_Algorithm_Model(void)										//���̵���ٶȷ���ģ��
{

   //����ȫ�����˶�ģ��
	
	 
	
	 MotorControlDEF.MotorSpeedOne =   MotorControlDEF.Vx																				  -	 MotorControlDEF.Vspin;	
	 MotorControlDEF.MotorSpeedTwo =  -MotorControlDEF.Vx*sin(Pi/6)+MotorControlDEF.Vy*cos(Pi/6)  -  MotorControlDEF.Vspin;
	 MotorControlDEF.MotorSpeedThr =  -MotorControlDEF.Vx*sin(Pi/6)-MotorControlDEF.Vy*cos(Pi/6)  -  MotorControlDEF.Vspin; 
	
	 
	 CAN_RoboModule_DRV_Velocity_Mode(0,1,4900,MotorControlDEF.MotorSpeedOne);
	 CAN_RoboModule_DRV_Velocity_Mode(0,2,4900,MotorControlDEF.MotorSpeedTwo);
	 CAN_RoboModule_DRV_Velocity_Mode(0,3,4900,MotorControlDEF.MotorSpeedThr);
	
}





/*************************
����ִ��λ��
	USART1��DMA�жϡ�
*************************/
PID_AbsoluteType Remote_Angle_PID;										//ң�ؽǶ�PID

void Remote_Analysis(void)														//ң�����ݽ���
{
		static s16 Pre_spin=0;
		s16 Set_speed;
		u16 Accelerate=5;
		if(USART1_Read_DEF.slam_remote_change==REMOTE)						//ң���ٶȽ���
		{
			MotorControlDEF.Vy    =  USART1_Read_DEF.remote_Vy;		//X,Y�����ٶ�
			MotorControlDEF.Vx    =  USART1_Read_DEF.remote_Vx;
			
			
			Set_speed =  USART1_Read_DEF.remote_Vz;								//�����ٶȻ�ȡ
			if(Set_speed!=0||Pre_spin!=0)																//�ж��Ƿ�����ѡ���ٶ��Ƿ�Ϊ0
			{
					if(abs(Set_speed-Pre_spin)>Accelerate)									//�����Ӽ��ٹ���
					{
						if((Set_speed-Pre_spin)>0)
							Pre_spin=Pre_spin+Accelerate;
						else
							Pre_spin=Pre_spin-Accelerate;
					}
					else
					{
						Pre_spin=Set_speed;
					}
					MotorControlDEF.Vspin=Pre_spin;
					if(Pre_spin==0)
					{
						Remote_Set_Angle=Whole_PositionDEF.Z_Angle;
					}
			}
			else
			{
					Remote_Angle_PID.errNow=Whole_PositionDEF.Z_Angle-Remote_Set_Angle;					//�����Ƕ��ŷ�
					Remote_Angle_PID.kp=60;
					Remote_Angle_PID.ki=0;
					Remote_Angle_PID.kd=30;
					Remote_Angle_PID.errILim=200;
					PID_AbsoluteMode(&Remote_Angle_PID);
					MotorControlDEF.Vspin=Remote_Angle_PID.ctrOut;
					if(MotorControlDEF.Vspin>300)
					{
						MotorControlDEF.Vspin=300;
					}
					else if(MotorControlDEF.Vspin<-300)
					{
						MotorControlDEF.Vspin=-300;	
					}	
		  }
		}
		
}

/*************************
����ִ��λ��
	USART2��DMA�жϡ�
*************************/

union
{
	u8 DATA_8[24];
	float DATA_F[6];
}Whole_Position_Union;

void Whole_Position(u8 *DATA_Ptr)										//����ȫ����λ���ݣ�����DATA_PtrΪ����ָ��
{
		u8 Circle;
		if(DATA_Ptr[0]==0x0D&&DATA_Ptr[1]==0x0A&&DATA_Ptr[26]==0x0A&&DATA_Ptr[27]==0x0D)
		{
			for(Circle=0;Circle<24;Circle++)
			{
				Whole_Position_Union.DATA_8[Circle]=DATA_Ptr[Circle+2];
			}
			Whole_PositionDEF.Z_Angle 		 				=   Whole_Position_Union.DATA_F[0]+Whole_PositionDEF.C_Z_Angle;
			Whole_PositionDEF.X_Angle		   				=   Whole_Position_Union.DATA_F[1];
			Whole_PositionDEF.Y_Angle 				 		=   Whole_Position_Union.DATA_F[2];
			Whole_PositionDEF.X_Coordinate 				= -((Whole_Position_Union.DATA_F[3])*sin(Radian45+Whole_PositionDEF.C_Z_Angle) + (Whole_Position_Union.DATA_F[4])*cos(Radian45+Whole_PositionDEF.C_Z_Angle))+Whole_PositionDEF.CX_Coordinate;
			Whole_PositionDEF.Y_Coordinate 				=   (Whole_Position_Union.DATA_F[3])*cos(Radian45+Whole_PositionDEF.C_Z_Angle) - (Whole_Position_Union.DATA_F[4])*sin(Radian45+Whole_PositionDEF.C_Z_Angle)+Whole_PositionDEF.CY_Coordinate;
			Whole_PositionDEF.Angle_Acceleration  =   Whole_Position_Union.DATA_F[5];
			Whole_PositionDEF.Statu								=   OK;															//ȫ����λ���ݽ�����ȷ��־λ
			LED3=1;
		}
		else
		{
			Whole_PositionDEF.Statu								=   UNFinished;
			LED3=0;
		}
}


/*************************
����ִ��λ��
	USART2��DMA�жϡ�
*************************/

 u8 Point_TO_Point(float SPoint_X,float SPoint_Y,float FPoint_X,float FPoint_Y,u16 Speed,float Spin_Angle,u16 Stage,u8 Stage_Next)				//�㵽���λ���ŷ�
{
		static u16 PStatu=0;
		static u16 Pre_speed=0;
		u16 Accelerate=5;
		float Length,Servo_length;
		u8 Finish_MARK=UNFinished;
		if(PStatu==Stage)
		{
				if(Speed<32767)
				{ 
						/****************************Vx Vy�ٶȷ���**********************************/
						
					
					  /*********************************�����ŷ�***********************************/
						Length=sqrt(pow(FPoint_Y-SPoint_Y,2)+pow(FPoint_X-SPoint_X,2));
						Servo_length=Speed/2*(Speed*0.0001);
						if(Length<Servo_length)
						{
							ACC_DEC_TYPEDEF.START_Place=Servo_length;
							ACC_DEC_TYPEDEF.START_Speed=Speed;
							ACC_DEC_TYPEDEF.STOP_Place=0;
							ACC_DEC_TYPEDEF.STOP_Speed=0;
							ACC_DEC_TYPEDEF.PRESENT_Place=Length;
							ACC_DEC_servo(&ACC_DEC_TYPEDEF);
							Pre_speed=-ACC_DEC_TYPEDEF.PRESENT_Speed;
							if(Length<10)
							{
								Pre_speed=0;
								Finish_MARK=OK;
								if(PStatu==Stage&&Stage_NEXT==0x08&&Stage_Next!=Stage_STOP)
								{
									PStatu++;
								}
							}
						}
						 /*********************************�����ŷ�***********************************/
						else
						{
						/********************************�����ŷ�**********************************/
							
							if(abs(Speed-Pre_speed)>Accelerate)
							{
								if((Speed-Pre_speed)>0)
									Pre_speed=Pre_speed+Accelerate;
								else
									Pre_speed=Pre_speed-Accelerate;
							}
							else
							{
									Pre_speed=Speed;
							}

						/********************************�����ŷ�**********************************/
						}
						
						
						MotorControlDEF.Vx=   Pre_speed*(FPoint_X-SPoint_X)/Length*cos(Whole_PositionDEF.Z_Angle*Pi/180) + Pre_speed*(FPoint_Y-SPoint_Y)/Length*sin(Whole_PositionDEF.Z_Angle*Pi/180);
						MotorControlDEF.Vy= - Pre_speed*(FPoint_X-SPoint_X)/Length*sin(Whole_PositionDEF.Z_Angle*Pi/180) + Pre_speed*(FPoint_Y-SPoint_Y)/Length*cos(Whole_PositionDEF.Z_Angle*Pi/180);
					
						/****************************Vx Vy�ٶȷ���**********************************/
						
						
						
						/****************************Vspin�ٶȷ���**********************************/
						
						Point_Angle_PID.errNow=Whole_PositionDEF.Z_Angle-Spin_Angle;
						Point_Angle_PID.kp=60;
						Point_Angle_PID.ki=0;
						Point_Angle_PID.kd=30;
						Point_Angle_PID.errILim=200;
						PID_AbsoluteMode(&Point_Angle_PID);
						MotorControlDEF.Vspin=Point_Angle_PID.ctrOut;
						if(MotorControlDEF.Vspin>900)
						{
							MotorControlDEF.Vspin=900;
						}
						else if(MotorControlDEF.Vspin<-900)
						{
							MotorControlDEF.Vspin=-900;
						}
						
						
						/****************************Vspin�ٶȷ���**********************************/
						
				}
		}
		else if(Stage==0)
		{
			PStatu=0;
		}
		return Finish_MARK;
}


void ACC_DEC_servo(ACC_DEC_TYPE *ACC_DEC_TYPEDEF)																	//���μӼ����ٶ��ŷ�
{
	ACC_DEC_TYPEDEF->PRESENT_Speed=sqrt((double)(ACC_DEC_TYPEDEF->START_Place - ACC_DEC_TYPEDEF->STOP_Place)*((double)ACC_DEC_TYPEDEF->START_Place*ACC_DEC_TYPEDEF->STOP_Speed*ACC_DEC_TYPEDEF->STOP_Speed - (double)ACC_DEC_TYPEDEF->STOP_Place*ACC_DEC_TYPEDEF->START_Speed*ACC_DEC_TYPEDEF->START_Speed + (double)ACC_DEC_TYPEDEF->PRESENT_Place*ACC_DEC_TYPEDEF->START_Speed*ACC_DEC_TYPEDEF->START_Speed - (double)ACC_DEC_TYPEDEF->PRESENT_Place*ACC_DEC_TYPEDEF->STOP_Speed*ACC_DEC_TYPEDEF->STOP_Speed))/(ACC_DEC_TYPEDEF->START_Place - ACC_DEC_TYPEDEF->STOP_Place);
	if(ACC_DEC_TYPEDEF->STOP_Place-ACC_DEC_TYPEDEF->PRESENT_Place>0)
	{
		ACC_DEC_TYPEDEF->PRESENT_Speed=fabs(ACC_DEC_TYPEDEF->PRESENT_Speed);
	}
	else if(ACC_DEC_TYPEDEF->STOP_Place-ACC_DEC_TYPEDEF->PRESENT_Place<0)
	{
		ACC_DEC_TYPEDEF->PRESENT_Speed=-fabs(ACC_DEC_TYPEDEF->PRESENT_Speed);
	}
	else
	{
		ACC_DEC_TYPEDEF->PRESENT_Speed=ACC_DEC_TYPEDEF->STOP_Speed;
	}
}
float Vx,Vy,Vz;

void Algorithm_1(float x,float y,float z)      //m/s
{
   	  Vx = x;
			Vy = y;
	    Vz = z;
//	  Vx =  (x*60)/Value_radius;
//	  Vy =  (y*60)/Value_radius;
//	  Vz =  (z*60)/Value_radius;
//	
	  
		MotorControlDEF.MotorSpeedOne  	= 	-Vy+l_value*Vz;     //Vy�����ٶ�,
		MotorControlDEF.MotorSpeedTwo 	=  	Vx*V_VALUE+0.5f*Vy+l_value*Vz;
	  MotorControlDEF.MotorSpeedThr 	= 	-Vx*V_VALUE+0.5f*Vy+l_value*Vz;
  
	 CAN_RoboModule_DRV_Velocity_Mode(1,1,4900,MotorControlDEF.MotorSpeedOne);
	 CAN_RoboModule_DRV_Velocity_Mode(1,2,4900,MotorControlDEF.MotorSpeedTwo);
	 CAN_RoboModule_DRV_Velocity_Mode(1,3,4900,MotorControlDEF.MotorSpeedThr);

}

