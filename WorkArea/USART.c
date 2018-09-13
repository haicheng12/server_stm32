#include "USART.h"

USART_MID_TYPE USART_MID_Read;
USART_MID_TYPE USART_MID_Write;

USART1_Read_TYPE USART1_Read_DEF;

/*************************extern*********************************/

extern MotorControlType MotorControlDEF;											//���̵������
extern u8 Control_Model_Statu;																//����ģʽ״̬��ǣ�REMOTE��AUTO
extern u8 AUTO_PRE_AREA,AUTO_AIM_AREA;											  //��ǰ��������Ŀ����������
extern Whole_PositionType Whole_PositionDEF;									//ȫ����λ�ṹ��
/*************************extern*********************************/


union
{
	u8 DATA_8[12];
	float DATA_F[3];
}USART1_POSITION_DATA;


/*****************************USART1����*****************************/

u8 JOY_DATA[USART1_RX_NUM];																				//���ҡ������
u8 PAD_DATA[USART1_RX_NUM];																				//���PAD����
u8 UART1_TxBuffer[USART1_TX_NUM];																	//����1���ձ���

void UART1_Configuration(void)																		//����1����			����λ��ͨѶ
{
  GPIO_InitTypeDef GPIO_InitStructure;														
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	 
	RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1,    ENABLE); 			//����DMAʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
	////////////////////DMA����/////////////////////////
	DMA_DeInit(DMA1_Channel4);
	
	DMA_InitStructure.DMA_BufferSize          = USART1_TX_NUM;
	DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralDST;      //����Ϊ�����յ�
	DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority            = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M                 = DMA_M2M_Disable;
	
	DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)&(USART1->DR);    //�����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr      = (uint32_t)UART1_TxBuffer;   //UART1���ݷ��ͻ�����  
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel4, DISABLE);
	
	////////////////////DMA����////////////////////////////////////
	
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&(USART1->DR);	    		//����DMAԴ���ڴ��ַ&�������ݼĴ�����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr 			= (uint32_t)USART_MID_Read.data_8;			//�ڴ��ַ(Ҫ����ı�����ָ��)
	DMA_InitStructure.DMA_DIR 								= DMA_DIR_PeripheralSRC ;						//���򣺴����赽�ڴ�		
	DMA_InitStructure.DMA_BufferSize 					= USART1_RX_NUM;												//�����СDMA_BufferSize=SENDBUFF_SIZE*/	    
	DMA_InitStructure.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable; 				//�����ַ����	
	DMA_InitStructure.DMA_MemoryInc 					= DMA_MemoryInc_Enable;							//�ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_Byte;			//�������ݵ�λ	
	DMA_InitStructure.DMA_MemoryDataSize			= DMA_MemoryDataSize_Byte;	 				//�ڴ����ݵ�λ 8bit
	DMA_InitStructure.DMA_Mode 								= DMA_Mode_Circular ;	 							//DMAģʽ��ѭ��
	DMA_InitStructure.DMA_Priority 						= DMA_Priority_VeryHigh;  					//���ȼ����ǳ���	
	DMA_InitStructure.DMA_M2M 								= DMA_M2M_Disable;									//��ֹ�ڴ浽�ڴ�Ĵ���   
	DMA_Init(DMA1_Channel5, &DMA_InitStructure); 	   														  //����DMA1��5ͨ��
	DMA_Cmd (DMA1_Channel5,ENABLE);									//ʹ��DMA
	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);  	//����DMA������ɺ�����ж�

	/**********************************************************************************/
	
  USART_DeInit(USART1); 
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART1, &USART_InitStructure); 
	
  USART_DMACmd(USART1, USART_DMAReq_Tx,  ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx,  ENABLE);
  
	USART_Cmd(USART1, ENABLE);  

}


union
{
	u32 DATA_32;
	u8 DATA_8[4];																//����У��
}USART_SEND_CRC;															//����CRCУ��

void USART1_DataSend(void)										//USART1   DMA����			16��λ
{
	u8 CIR1,CIR2;
	DMA_Cmd(DMA1_Channel4, DISABLE);
	
	
	USART_MID_Write.data_f[0]=Whole_PositionDEF.X_Coordinate;
	USART_MID_Write.data_f[1]=Whole_PositionDEF.Y_Coordinate;
	USART_MID_Write.data_f[2]=Whole_PositionDEF.Z_Angle;
	USART_SEND_CRC.DATA_32=0;
	for(CIR2=0;CIR2<8;CIR2++)
	{
		for(CIR1=0;CIR1<USART1_TX_NUM-4;CIR1++)
		{
			USART_SEND_CRC.DATA_32=((USART_MID_Write.data_8[CIR1]>>CIR2)&0x01)*(CIR1+1)*(CIR2+1)+USART_SEND_CRC.DATA_32;			//У��
		}
	}
	
	for(CIR1=0;CIR1<12;CIR1++)
	{
		UART1_TxBuffer[CIR1]=USART_MID_Write.data_8[CIR1];
	}
	for(CIR1=12;CIR1<16;CIR1++)
	{
		UART1_TxBuffer[CIR1]=USART_SEND_CRC.DATA_8[CIR1-12];
	}
	
	DMA1_Channel4->CNDTR = USART1_TX_NUM;							  //���͸���
	DMA_Cmd(DMA1_Channel4, ENABLE);  
	
}


union
{
	u32 DATA_32;
	u8 DATA_8[4];																//����У��
}USART_READ_CRC;															//����CRCУ��

void DMA1_Channel5_IRQHandler(void)										//����1�жϽ��ܣ�����λ��ͨѶ�ӿ�		   41	
{	
//�ж��Ƿ�ΪDMA��������ж�
	 u8 CIR1,CIR2;
   if(DMA_GetFlagStatus(DMA1_FLAG_TC5)==SET) 
   {  
		    DMA_ClearFlag(DMA1_FLAG_TC5); 
		    DMA_ClearITPendingBit(DMA1_IT_TC5);
		    USART_READ_CRC.DATA_32=0;
				for(CIR2=0;CIR2<8;CIR2++)
				{
					for(CIR1=0;CIR1<USART1_TX_NUM-4;CIR1++)
					{
						USART_READ_CRC.DATA_32=((USART_MID_Read.data_8[CIR1]>>CIR2)&0x01)*(CIR1+1)*(CIR2+1)+USART_READ_CRC.DATA_32;
					}
				}
				USART1_Read_DEF.Correct_x							=USART_MID_Read.data_f[0]*1000;
				USART1_Read_DEF.Correct_y							=USART_MID_Read.data_f[1]*1000;
				USART1_Read_DEF.Correct_z							=USART_MID_Read.data_f[2]*180/Pi;
				USART1_Read_DEF.slam_Vx								=USART_MID_Read.data_f[3]*1000;
				USART1_Read_DEF.slam_Vy								=USART_MID_Read.data_f[4]*1000;
				USART1_Read_DEF.slam_Vz								=USART_MID_Read.data_f[5]*1000;
				USART1_Read_DEF.remote_Vx							=USART_MID_Read.data_f[6];
				USART1_Read_DEF.remote_Vy							=USART_MID_Read.data_f[7];
				USART1_Read_DEF.remote_Vz							=USART_MID_Read.data_f[8];
				USART1_Read_DEF.slam_remote_change		=USART_MID_Read.data_8[36];					
				if(USART1_Read_DEF.slam_remote_change==SLAM)             //0 slam     1 remote
				{
					MotorControlDEF.Vx=USART1_Read_DEF.slam_Vx;
					MotorControlDEF.Vy=USART1_Read_DEF.slam_Vy;
					MotorControlDEF.Vspin=USART1_Read_DEF.slam_Vz;
				}
				else
				{
					Remote_Analysis();
				}
	 }
}




/*****************************USART2����*****************************/


u8 UART2_RxBuffer[USART2_RX_NUM];				//DMA��������
u8 UART2_TxBuffer[USART2_TX_NUM];


void UART2_Configuration(void)					//����2���ã���ȫ����λͨѶ
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,  ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
  USART_DeInit(USART2); 
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_RXNE,  ENABLE);

	USART_Cmd(USART2, ENABLE);  

}
posture posture1;
int run=0;
int timede=0;
float pos_x=0;
float pos_y=0;
float zangle=0;
float xangle=0;
float yangle=0;
float w_z=0;
void USART2_IRQHandler(void)
{	 
	static uint8_t ch;

	static uint8_t count=0;
	static uint8_t i=0;

	if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit( USART2,USART_IT_RXNE);
		ch=USART_ReceiveData(USART2);
		 switch(count)
		 {
			 case 0:
				 if(ch==0x0d)
					 count++;
				 else
					 count=0;
				 break;
			 case 1:
				 if(ch==0x0a)
				 {
					 i=0;
					 count++;
				 }
				 else if(ch==0x0d);
				 else
					 count=0;
				 break;
			 case 2:
				 posture1.data[i]=ch;
			   i++;
			   if(i>=24)
				 {
					 i=0;
					 count++;
				 }
				 break;
			 case 3:
				 if(ch==0x0a)
					 count++;
				 else
					 count=0;
				 break;
			 case 4:
				 if(ch==0x0d)
				 {
  				 Whole_PositionDEF.Z_Angle=posture1.ActVal[0];          //Z_angle
	  		   xangle=posture1.ActVal[1];
		  	   yangle=posture1.ActVal[2];
			     Whole_PositionDEF.X_Coordinate =posture1.ActVal[4];    //Y_location
			     Whole_PositionDEF.Y_Coordinate =posture1.ActVal[3];    //Y_location
			     w_z   =posture1.ActVal[5];
					 timede=run;
					 run=0;
				 }
			   count=0;
				 break;
			 
			 default:
				 count=0;
			   break;		 
		 }
	 }
}



