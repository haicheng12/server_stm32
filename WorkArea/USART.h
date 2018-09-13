#ifndef _USART_h_
#define _USART_h_
#include "PointH.h"

#define USART1_TX_NUM 16
#define USART1_RX_NUM 41

#define USART2_TX_NUM 5
#define USART2_RX_NUM 28




typedef union
{
	float data_f[9];
	u8 data_8[USART1_RX_NUM];
}USART_MID_TYPE;

typedef union
{
 uint8_t data[24];
 float ActVal[6];
}posture;


typedef struct
{
	float Correct_x;
	float Correct_y;
	float Correct_z;
	
	float slam_Vx;
	float slam_Vy;
	float slam_Vz;
	
	float remote_Vx;
	float remote_Vy;
	float remote_Vz;
	
	u8 slam_remote_change;
}USART1_Read_TYPE;




void UART1_Configuration(void);
void USART1_DataSend(void);								//USART1   DMA∑¢ÀÕ
void UART2_Configuration(void);           //USART2≈‰÷√
void USART2_DataSend(void);							  //USART2   DMA∑¢ÀÕ


#endif




