/**
ʹ��ע������:
    1.��tjc_usart_hmi.c��tjc_usart_hmi.h �ֱ��빤��
    2.����Ҫʹ�õĺ������ڵ�ͷ�ļ������ #include "tjc_usart_hmi.h"
    3.ʹ��ǰ�뽫 HAL_UART_Transmit_IT() ���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
    3.TJCPrintf��printf�÷�һ��

*/

#include "main.h"
//#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "tjc_usart_hmi.h"
#include <stddef.h>
#include <usart.h>
#include <time.h>
#include <LobotServoController.h>
#include <stdlib.h>
int usarflag;

extern uint8_t usart1_rx_completed_flag;
extern uint8_t position1[32];
extern float L_Target_Speed;
extern float R_Target_Speed;
#define STR_LENGTH 100
typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Lenght;
    uint8_t  Ring_data[RINGBUFF_LEN];
}RingBuff_t;

RingBuff_t ringBuff;	//����һ��ringBuff�Ļ�����
uint8_t RxBuff[1];



/********************************************************
��������  	TJCPrintf
���ߣ�    	wwd
���ڣ�    	2022.10.08
���ܣ�    	�򴮿ڴ�ӡ����,ʹ��ǰ�뽫USART_SCREEN_write���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
���������		�ο�printf
����ֵ�� 		��ӡ�����ڵ�����
�޸ļ�¼��
**********************************************************/

void TJCPrintf(const char *str, ...)
{


	uint8_t end = 0xff;
	char buffer[STR_LENGTH+1];  // ���ݳ���
	uint8_t i = 0;
	va_list arg_ptr;
	va_start(arg_ptr, str);
	vsnprintf(buffer, STR_LENGTH+1, str, arg_ptr);
	va_end(arg_ptr);
	while ((i < STR_LENGTH) && (i < strlen(buffer)))
	{
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)(buffer) + i++, 1);
		while(huart3.gState != HAL_UART_STATE_READY);	//�ȴ��������
	}

	HAL_UART_Transmit_IT(&huart3, &end, 1);			//���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
	while(huart3.gState != HAL_UART_STATE_READY);	//�ȴ��������
	HAL_UART_Transmit_IT(&huart3, &end, 1);			//���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
	while(huart3.gState != HAL_UART_STATE_READY);	//�ȴ��������
	HAL_UART_Transmit_IT(&huart3, &end, 1);			//���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
	while(huart3.gState != HAL_UART_STATE_READY);	//�ȴ��������

}
//this function is very important

void jiaozhun(){
//	  time_t currentTime;
//    struct tm *localTime;

//    // ??????
//    currentTime = time(NULL);

//    // ????????????
//    localTime = localtime(&currentTime);
	  TJCPrintf("rtc0=%d\xff\xff\xff",2004);
    TJCPrintf("rtc1=%02d\xff\xff\xff",04);
    TJCPrintf("rtc2=%02d\xff\xff\xff",15);
	  TJCPrintf("rtc3=%02d\xff\xff\xff",20);
    TJCPrintf("rtc4=%02d\xff\xff\xff",58);
    TJCPrintf("rtc5=%02d\xff\xff\xff",32);
	



}
uint16_t show(){
		TJCPrintf("zaizhi.n0.val=%d\xff\xff\xff",u(0));
		TJCPrintf("zaizhi.n1.val=%d\xff\xff\xff",u(1));
		TJCPrintf("zaizhi.n2.val=%d\xff\xff\xff",u(2));
	  TJCPrintf("zaizhi.n3.val=%d\xff\xff\xff",u(3));
	  TJCPrintf("zaizhi.n4.val=%d\xff\xff\xff",u(4));
	  TJCPrintf("zaizhi.n5.val=%d\xff\xff\xff",u(5));
	
	
}	
/********************************************************
��������  	HAL_UART_RxCpltCallback
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	���ڽ����ж�,�����յ�������д�뻷�λ�����
���������
����ֵ�� 		void
�޸ļ�¼��
**********************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1)	// �ж������ĸ����ڴ������ж�
	{
		writeRingBuff(RxBuff[0]);
		HAL_UART_Receive_IT(&huart1,RxBuff,1);
     if( ringBuff.Lenght>20)
		 {
			 usart1_rx_completed_flag=1;
		 }
	}
}



/********************************************************
��������  	initRingBuff
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	��ʼ�����λ�����
���������
����ֵ�� 		void
�޸ļ�¼��
**********************************************************/
void initRingBuff(void)
{
  //��ʼ�������Ϣ
  ringBuff.Head = 0;
  ringBuff.Tail = 0;
  ringBuff.Lenght = 0;
}



/********************************************************
��������  	writeRingBuff
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	�����λ�����д������
���������
����ֵ�� 		void
�޸ļ�¼��
**********************************************************/
void writeRingBuff(uint8_t data)
{
  if(ringBuff.Lenght >= RINGBUFF_LEN) //�жϻ������Ƿ�����
  {
    return ;
  }
  ringBuff.Ring_data[ringBuff.Tail]=data;
  ringBuff.Tail = (ringBuff.Tail+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
  ringBuff.Lenght++;

}




/********************************************************
��������  	deleteRingBuff
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	ɾ�����ڻ���������Ӧ���ȵ�����
���������		Ҫɾ���ĳ���
����ֵ�� 		void
�޸ļ�¼��
**********************************************************/
void deleteRingBuff(uint16_t size)
{
	if(size >= ringBuff.Lenght)
	{
	    initRingBuff();
	    return;
	}
	for(int i = 0; i < size; i++)
	{

		if(ringBuff.Lenght == 0)//�жϷǿ�
		{
		initRingBuff();
		return;
		}
		ringBuff.Head = (ringBuff.Head+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
		ringBuff.Lenght--;

	}

}



/********************************************************
��������  	read1BFromRingBuff
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	�Ӵ��ڻ�������ȡ1�ֽ�����
���������		position:��ȡ��λ��
����ֵ�� 		����λ�õ�����(1�ֽ�)
�޸ļ�¼��
**********************************************************/
uint8_t read1BFromRingBuff(uint16_t position)
{
	uint16_t realPosition = (ringBuff.Head + position) % RINGBUFF_LEN;

	return ringBuff.Ring_data[realPosition];
}




/********************************************************
��������  	getRingBuffLenght
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	��ȡ���ڻ���������������
���������
����ֵ�� 		���ڻ���������������
�޸ļ�¼��
**********************************************************/
uint16_t getRingBuffLenght()
{
	return ringBuff.Lenght;
}


/********************************************************
��������  	isRingBuffOverflow
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	�жϻ��λ������Ƿ�����
���������
����ֵ�� 		1:���λ��������� , 2:���λ�����δ��
�޸ļ�¼��
**********************************************************/
uint8_t isRingBuffOverflow()
{
	return ringBuff.Lenght == RINGBUFF_LEN;
}

void judge1(void){//��һ�ν�������
	while (usize>=8){
		if(u(0)!=0x05||u(7)!=0x07)
		  {
			  udelete(1);
		  }

		else
		  {
			  break;
		  }
	  }	
 //serial port and begin receiving
		if(u(0)==0x05&&u(7)==0x07){
	position1[0]=change(u(1));
	position1[1]=change(u(2));
	position1[2]=change(u(3));
	position1[3]=change(u(4));
	position1[4]=change(u(5));
	position1[5]=change(u(6));

}
}
void judge2(void){
		while (usize>=8){
		if(u(0)!=0x05||u(7)!=0x07)//����Ϊ���жϽ����Ƿ�������
		  {
			  udelete(1);
		  }
		else
		  {
			  break;
		  }
	  }	
 //serial port and begin receiving
		if(u(0)==0x05&&u(7)==0x07){
  position1[6]=change(u(1));
	position1[7]=change(u(2));
	position1[8]=change(u(3));
	position1[9]=change(u(4));
	position1[10]=change(u(5));
	position1[11]=change(u(6));
}
}
void judge3(void){
	WHILEE:		while (usart1_rx_completed_flag!=1){
		
	  }	
		
		while (usart1_rx_completed_flag==1){
		if(u(0)!=0x05||u(19)!=0x07)
		  {
			  udelete(1);
				usart1_rx_completed_flag=0;
				goto WHILEE;
		  }

		else
		  {
			  break;
		  }
	  }	
 //serial port and begin receiving
		if(u(0)==0x05&&u(19)==0x07){
	position1[0]=change(u(1));
	position1[1]=change(u(2));
	position1[2]=change(u(3));
	position1[3]=change(u(4));
	position1[4]=change(u(5));
	position1[5]=change(u(6));
  position1[6]=change(u(7));
	position1[7]=change(u(8));
	position1[8]=change(u(9));
	position1[9]=change(u(10));
	position1[10]=change(u(11));
	position1[11]=change(u(12));
	position1[12]=change(u(13));
	position1[13]=change(u(14));
	position1[14]=change(u(15));
	position1[15]=change(u(16));
	position1[16]=change(u(17));
	position1[17]=change(u(18));
}
}
