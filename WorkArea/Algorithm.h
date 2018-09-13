#ifndef _Algorithm_h_
#define _Algorithm_h_
#include "PointH.h"

/******************Control Model***********************/

#define REMOTE	1					//手柄遥控
#define SLAM    0					//自动

/******************Control Model***********************/

#define ON 1
#define OFF 0

#define CHECK_OUT_A_NONE     0X888       //ps CHECK_OUT角度修正的方法

#define Remote_Key_UP  0
#define Remote_Key_DOWN  1

#define Remote_Set_Max	10

#define OK 0x08

#define UNFinished 0

#define Stage_NEXT 0x08	
#define Stage_STOP 0



#define Pi      3.141592653589											

#define Radian45   0.785398163397




typedef struct 
{
 /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
 float kp;     //比例系数
 float ki;     //积分系数
 float kd;     //微分系数
 float errILim;//误差积分上限
 
 float errNow;//当前的误差
 float ctrOut;//控制量输出
 
 /*PID算法内部变量，其值不能修改*/
 float errOld;
 float errP;
 float errI;
 float errD;
 
}PID_AbsoluteType;


typedef struct
{
  s16 Vx;																						//Vx速度
	s16 Vy;																						//Vy速度
  s16 Vspin;																				//自旋速度

	s16 MotorSpeedOne;																//电机1分配速度
	s16 MotorSpeedTwo;																//电机2分配速度
	s16 MotorSpeedThr;																//电机3分配速度
	
}MotorControlType;																	//底盘电机参数


typedef struct
{
	
	float Z_Angle;																		//修正后的世界坐标
	float X_Angle;																		
	float Y_Angle;																		
	float X_Coordinate;																//修正后的世界坐标
	float Y_Coordinate;																//修正后的世界坐标
	float Angle_Acceleration;
	
	float	CX_Coordinate;															//校正的数据
	float	CY_Coordinate;															//校正的数据
	float C_Z_Angle;																	//校正的数据
		
	u8 Statu;																						//全场定位数据就绪标识
	
}Whole_PositionType;																  //全场定位数据


typedef struct
{
	s32 START_Place;		//伺服的起始位置
	s16 START_Speed;		//伺服的起始速度
	
	s32 STOP_Place;			//伺服的结束位置
	s16 STOP_Speed;			//伺服的结束速度

	s32 PRESENT_Place;	//伺服的当前位置
	s16 PRESENT_Speed;	//伺服的当前速度
}ACC_DEC_TYPE;
#define   zhouchang       0.47728
#define  Value_radius     0.076
#define  V_VALUE           (sqrt(3)/2.f)
#define  l_value  		    0.26


void Algorithm_1(float x,float y,float z);
void Chassis_MOTOR_Init(void);												//底盘电机模式初始化
void Chassis_Algorithm_Model(void);										//底盘电机速度分配模型
void Whole_Position(u8 *DATA_Ptr);										//解析全场定位数据，传入DATA_Ptr为数据指针
void Remote_Analysis(void);														//遥控数据解析
u8 Point_TO_Point(float SPoint_X,float SPoint_Y,float FPoint_X,float FPoint_Y,u16 Speed,float Spin_Angle,u16 Stage,u8 Stage_Next);				//点到点的位置伺服
void ACC_DEC_servo(ACC_DEC_TYPE *ACC_DEC_TYPEDEF);		//梯形加减速速度伺服


#endif


