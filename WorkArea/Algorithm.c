#include "Algorithm.h"

/************************extern*************************/

extern u8 JOY_DATA[USART1_RX_NUM];							//存放摇杆数据
extern u8 PAD_DATA[USART1_RX_NUM];							//存放PAD数据
extern u8 UART1_TxBuffer[USART1_TX_NUM];

extern USART1_Read_TYPE USART1_Read_DEF;

/************************extern*************************/

/************************FLAHS**************************/


PID_AbsoluteType Point_Angle_PID;										//点到点的角度PID

float Remote_Set_Angle=0;														//遥控伺服角度
	

Whole_PositionType Whole_PositionDEF;															//全场定位结构体
ACC_DEC_TYPE ACC_DEC_TYPEDEF;																			//梯形加减速算法优化





//绝对式PID算法
void PID_AbsoluteMode(PID_AbsoluteType* PID)
{

 if(PID->kp      < 0)    PID->kp      = -PID->kp;
 if(PID->ki      < 0)    PID->ki      = -PID->ki;
 if(PID->kd      < 0)    PID->kd      = -PID->kd;
 if(PID->errILim < 0)    PID->errILim = -PID->errILim;

 PID->errP = PID->errNow;  //读取现在的误差，用于kp控制

 PID->errI += PID->errNow; //误差积分，用于ki控制

 if(PID->errILim != 0)	   //微分上限和下限
 {
  if(     PID->errI >  PID->errILim)    PID->errI =  PID->errILim;
  else if(PID->errI < -PID->errILim)    PID->errI = -PID->errILim;
 }
 
 PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制

 PID->errOld = PID->errNow;	//保存现在的误差
 
 PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出

}


MotorControlType MotorControlDEF;											//底盘电机参数



void Chassis_MOTOR_Init(void)													//底盘电机模式初始化
{
	
	SysDelay_ms(1000);
	CAN_RoboModule_DRV_Reset(0,0);                      //对1组1号驱动器进行复位
	SysDelay_ms(100);                                  //发送复位指令后的延时必须要有，等待驱动器复位完毕。
	CAN_RoboModule_DRV_Config(0,0,100,0);               //配置为1s传回一次数据
	SysDelay_ms(100);
	CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Mode);  //选择进入速度环模式
	SysDelay_ms(500);
	
	MotorControlDEF.Vx=0;
	MotorControlDEF.Vy=0;
	MotorControlDEF.Vspin=0;
	
}


/***********************
函数执行位置
	main函数。
***********************/
float slam_vx,slam_vy,slam_vz;
void Chassis_Algorithm_Model(void)										//底盘电机速度分配模型
{

   //三轮全向轮运动模型
	
	 
	
	 MotorControlDEF.MotorSpeedOne =   MotorControlDEF.Vx																				  -	 MotorControlDEF.Vspin;	
	 MotorControlDEF.MotorSpeedTwo =  -MotorControlDEF.Vx*sin(Pi/6)+MotorControlDEF.Vy*cos(Pi/6)  -  MotorControlDEF.Vspin;
	 MotorControlDEF.MotorSpeedThr =  -MotorControlDEF.Vx*sin(Pi/6)-MotorControlDEF.Vy*cos(Pi/6)  -  MotorControlDEF.Vspin; 
	
	 
	 CAN_RoboModule_DRV_Velocity_Mode(0,1,4900,MotorControlDEF.MotorSpeedOne);
	 CAN_RoboModule_DRV_Velocity_Mode(0,2,4900,MotorControlDEF.MotorSpeedTwo);
	 CAN_RoboModule_DRV_Velocity_Mode(0,3,4900,MotorControlDEF.MotorSpeedThr);
	
}





/*************************
函数执行位置
	USART1的DMA中断。
*************************/
PID_AbsoluteType Remote_Angle_PID;										//遥控角度PID

void Remote_Analysis(void)														//遥控数据解析
{
		static s16 Pre_spin=0;
		s16 Set_speed;
		u16 Accelerate=5;
		if(USART1_Read_DEF.slam_remote_change==REMOTE)						//遥控速度解析
		{
			MotorControlDEF.Vy    =  USART1_Read_DEF.remote_Vy;		//X,Y方向速度
			MotorControlDEF.Vx    =  USART1_Read_DEF.remote_Vx;
			
			
			Set_speed =  USART1_Read_DEF.remote_Vz;								//自旋速度获取
			if(Set_speed!=0||Pre_spin!=0)																//判断是否有自选的速度是否为0
			{
					if(abs(Set_speed-Pre_spin)>Accelerate)									//自旋加减速过程
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
					Remote_Angle_PID.errNow=Whole_PositionDEF.Z_Angle-Remote_Set_Angle;					//自旋角度伺服
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
函数执行位置
	USART2的DMA中断。
*************************/

union
{
	u8 DATA_8[24];
	float DATA_F[6];
}Whole_Position_Union;

void Whole_Position(u8 *DATA_Ptr)										//解析全场定位数据，传入DATA_Ptr为数据指针
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
			Whole_PositionDEF.Statu								=   OK;															//全场定位数据接收正确标志位
			LED3=1;
		}
		else
		{
			Whole_PositionDEF.Statu								=   UNFinished;
			LED3=0;
		}
}


/*************************
函数执行位置
	USART2的DMA中断。
*************************/

 u8 Point_TO_Point(float SPoint_X,float SPoint_Y,float FPoint_X,float FPoint_Y,u16 Speed,float Spin_Angle,u16 Stage,u8 Stage_Next)				//点到点的位置伺服
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
						/****************************Vx Vy速度分配**********************************/
						
					
					  /*********************************减速伺服***********************************/
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
						 /*********************************减速伺服***********************************/
						else
						{
						/********************************加速伺服**********************************/
							
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

						/********************************加速伺服**********************************/
						}
						
						
						MotorControlDEF.Vx=   Pre_speed*(FPoint_X-SPoint_X)/Length*cos(Whole_PositionDEF.Z_Angle*Pi/180) + Pre_speed*(FPoint_Y-SPoint_Y)/Length*sin(Whole_PositionDEF.Z_Angle*Pi/180);
						MotorControlDEF.Vy= - Pre_speed*(FPoint_X-SPoint_X)/Length*sin(Whole_PositionDEF.Z_Angle*Pi/180) + Pre_speed*(FPoint_Y-SPoint_Y)/Length*cos(Whole_PositionDEF.Z_Angle*Pi/180);
					
						/****************************Vx Vy速度分配**********************************/
						
						
						
						/****************************Vspin速度分配**********************************/
						
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
						
						
						/****************************Vspin速度分配**********************************/
						
				}
		}
		else if(Stage==0)
		{
			PStatu=0;
		}
		return Finish_MARK;
}


void ACC_DEC_servo(ACC_DEC_TYPE *ACC_DEC_TYPEDEF)																	//梯形加减速速度伺服
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
	  
		MotorControlDEF.MotorSpeedOne  	= 	-Vy+l_value*Vz;     //Vy是线速度,
		MotorControlDEF.MotorSpeedTwo 	=  	Vx*V_VALUE+0.5f*Vy+l_value*Vz;
	  MotorControlDEF.MotorSpeedThr 	= 	-Vx*V_VALUE+0.5f*Vy+l_value*Vz;
  
	 CAN_RoboModule_DRV_Velocity_Mode(1,1,4900,MotorControlDEF.MotorSpeedOne);
	 CAN_RoboModule_DRV_Velocity_Mode(1,2,4900,MotorControlDEF.MotorSpeedTwo);
	 CAN_RoboModule_DRV_Velocity_Mode(1,3,4900,MotorControlDEF.MotorSpeedThr);

}

