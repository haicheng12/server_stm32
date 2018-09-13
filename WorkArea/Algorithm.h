#ifndef _Algorithm_h_
#define _Algorithm_h_
#include "PointH.h"

/******************Control Model***********************/

#define REMOTE	1					//�ֱ�ң��
#define SLAM    0					//�Զ�

/******************Control Model***********************/

#define ON 1
#define OFF 0

#define CHECK_OUT_A_NONE     0X888       //ps CHECK_OUT�Ƕ������ķ���

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
 /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
 float kp;     //����ϵ��
 float ki;     //����ϵ��
 float kd;     //΢��ϵ��
 float errILim;//����������
 
 float errNow;//��ǰ�����
 float ctrOut;//���������
 
 /*PID�㷨�ڲ���������ֵ�����޸�*/
 float errOld;
 float errP;
 float errI;
 float errD;
 
}PID_AbsoluteType;


typedef struct
{
  s16 Vx;																						//Vx�ٶ�
	s16 Vy;																						//Vy�ٶ�
  s16 Vspin;																				//�����ٶ�

	s16 MotorSpeedOne;																//���1�����ٶ�
	s16 MotorSpeedTwo;																//���2�����ٶ�
	s16 MotorSpeedThr;																//���3�����ٶ�
	
}MotorControlType;																	//���̵������


typedef struct
{
	
	float Z_Angle;																		//���������������
	float X_Angle;																		
	float Y_Angle;																		
	float X_Coordinate;																//���������������
	float Y_Coordinate;																//���������������
	float Angle_Acceleration;
	
	float	CX_Coordinate;															//У��������
	float	CY_Coordinate;															//У��������
	float C_Z_Angle;																	//У��������
		
	u8 Statu;																						//ȫ����λ���ݾ�����ʶ
	
}Whole_PositionType;																  //ȫ����λ����


typedef struct
{
	s32 START_Place;		//�ŷ�����ʼλ��
	s16 START_Speed;		//�ŷ�����ʼ�ٶ�
	
	s32 STOP_Place;			//�ŷ��Ľ���λ��
	s16 STOP_Speed;			//�ŷ��Ľ����ٶ�

	s32 PRESENT_Place;	//�ŷ��ĵ�ǰλ��
	s16 PRESENT_Speed;	//�ŷ��ĵ�ǰ�ٶ�
}ACC_DEC_TYPE;
#define   zhouchang       0.47728
#define  Value_radius     0.076
#define  V_VALUE           (sqrt(3)/2.f)
#define  l_value  		    0.26


void Algorithm_1(float x,float y,float z);
void Chassis_MOTOR_Init(void);												//���̵��ģʽ��ʼ��
void Chassis_Algorithm_Model(void);										//���̵���ٶȷ���ģ��
void Whole_Position(u8 *DATA_Ptr);										//����ȫ����λ���ݣ�����DATA_PtrΪ����ָ��
void Remote_Analysis(void);														//ң�����ݽ���
u8 Point_TO_Point(float SPoint_X,float SPoint_Y,float FPoint_X,float FPoint_Y,u16 Speed,float Spin_Angle,u16 Stage,u8 Stage_Next);				//�㵽���λ���ŷ�
void ACC_DEC_servo(ACC_DEC_TYPE *ACC_DEC_TYPEDEF);		//���μӼ����ٶ��ŷ�


#endif


