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
#include <stdlib.h>
#include "JY901S.h"
#include "Emm_v5.h"
#include "LobotServoController.h"
int usarflag;
int usarflag1;
extern uint8_t receive_cross_flag;//���ڽ��հ�ɫʮ�ֵı�־
extern uint8_t color;
extern uint8_t xgewei;
extern uint8_t xshibai;
extern uint8_t ygewei;
extern uint8_t yshibai;
extern uint16_t xl,yl,xr,yr;
extern uint16_t xc;
#define STR_LENGTH 100
typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Lenght;
    uint8_t  Ring_data[RINGBUFF_LEN];
}RingBuff_t;

RingBuff_t ringBuff;	//����һ��ringBuff�Ļ�����
RingBuff_t ringBuff1;//ringBuff1�����ڴ���x,y�����
uint8_t RxBuff[1];
uint8_t RxBuff_1[1];//RxBuff_1�����ڴ���x,y�����

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
		HAL_UART_Transmit_IT(&huart5, (uint8_t *)(buffer) + i++, 1);
		while(huart5.gState != HAL_UART_STATE_READY);	//�ȴ��������
	}

	HAL_UART_Transmit_IT(&huart5, &end, 1);			//���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
	while(huart5.gState != HAL_UART_STATE_READY);	//�ȴ��������
	HAL_UART_Transmit_IT(&huart5, &end, 1);			//���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
	while(huart5.gState != HAL_UART_STATE_READY);	//�ȴ��������
	HAL_UART_Transmit_IT(&huart5, &end, 1);			//���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
	while(huart5.gState != HAL_UART_STATE_READY);	//�ȴ��������

}
//this function is very important

void jiaozhun(){
	  TJCPrintf("rtc0=%d\xff\xff\xff",2004);
    TJCPrintf("rtc1=%02d\xff\xff\xff",04);
    TJCPrintf("rtc2=%02d\xff\xff\xff",15);
	  TJCPrintf("rtc3=%02d\xff\xff\xff",20);
    TJCPrintf("rtc4=%02d\xff\xff\xff",58);
    TJCPrintf("rtc5=%02d\xff\xff\xff",32);
}
void show(void){
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
extern uint8_t Rxdata;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART3)//����JY901Sģ��
	{
		HAL_UART_Receive_IT(&huart3,&Rxdata,1);
		uart3_read_data(Rxdata);	//��������
	}
  if(huart->Instance==USART1){//����K210���ص�����
			//���հ�ɫʮ�������������һ��
		  writeRingBuff(RxBuff[0]);
		  HAL_UART_Receive_IT(&huart1,RxBuff,1);
	}
	if(huart->Instance==UART4){//���ղ������������
		writeRingBuff1(RxBuff_1[0]);
		HAL_UART_Receive_IT(&huart4,RxBuff_1,1);
		
	
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
void initRingBuff1(void)
{
  //��ʼ�������Ϣ
  ringBuff1.Head = 0;
  ringBuff1.Tail = 0;
  ringBuff1.Lenght = 0;
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
void writeRingBuff1(uint8_t data)
{
  if(ringBuff1.Lenght >= RINGBUFF_LEN) //�жϻ������Ƿ�����
  {
    return ;
  }
  ringBuff1.Ring_data[ringBuff1.Tail]=data;
  ringBuff1.Tail = (ringBuff1.Tail+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
  ringBuff1.Lenght++;

}
/**
@Brief ������k210����ָ��
@Param num
@Retval NULL
*/
void sendnumber(uint32_t number)
{
    char buffer1[12]; // �㹻�洢32λ�������ַ�����ʾ���������ź���ֹ��'\0'��
	  sprintf(buffer1, "%d", number); // ������ת��Ϊ�ַ���
    HAL_UART_Transmit_IT(&huart1, ( uint8_t*)buffer1, strlen(buffer1)); // �����ַ���
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
void deleteRingBuff_k(uint16_t size1)
{
	if(size1 >= ringBuff1.Lenght)
	{
	    initRingBuff1();
	    return;
	}
	for(int i = 0; i < size1; i++)
	{

		if(ringBuff1.Lenght == 0)//�жϷǿ�
		{
		initRingBuff1();
		return;
		}
		ringBuff1.Head = (ringBuff1.Head+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
		ringBuff1.Lenght--;

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
uint8_t read1BFromRingBuff_1(uint16_t position)//���ڴ���x,y����
{
	uint16_t realPosition = (ringBuff1.Head + position) % RINGBUFF_LEN;

	return ringBuff1.Ring_data[realPosition];
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
uint16_t getRingBuffLenght_1(){
	return ringBuff1.Lenght;
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
uint8_t isRingBuffOverflow1(){
	return ringBuff1.Lenght==RINGBUFF_LEN;
}
	
	
	
void judge(){
	while (usize>=8){
		if(u(0)!=0x01||u(7)!=0x02)
		  {
			  udelete(1);
		  }

		else
		  {
			  break;
		  }
	  }	
 //serial port and begin receiving
		if(u(0)==0x01&&u(7)==0x02){
			if(u(1)==0x05){
				xgewei=abs(change(u(2)));
				xshibai=abs(change(u(3)));
				xc=xshibai*10+xgewei;
				//bottom wheels run forward
}
		  if(u(1)==0x01){
				xgewei=abs(change(u(2)));
				xshibai=abs(change(u(3)));
				ygewei=abs(change(u(4)));
				yshibai=abs(change(u(5)));
				xl=xshibai*10+xgewei;
				yl=yshibai*10+ygewei;
			
			}
			if(u(1)==0x02){
				xgewei=abs(change(u(2)));
				xshibai=abs(change(u(3)));
				ygewei=abs(change(u(4)));
				yshibai=abs(change(u(5)));
				xr=xshibai*10+xgewei;
				yr=yshibai*10+ygewei;
			}
			if(u(1)==0x03){
				xgewei=abs(change(u(2)));
				xshibai=abs(change(u(3)));
				ygewei=abs(change(u(4)));
				yshibai=abs(change(u(5)));
				color=change(u(6));
				xl=xgewei+xshibai*10;
				yl=ygewei+yshibai*10;
				
			
			}
			if(u(1)==0x04){
				xgewei=abs(change(u(2)));
				xshibai=abs(change(u(3)));
				ygewei=abs(change(u(4)));
				yshibai=abs(change(u(5)));
				xr=xgewei+xshibai*10;
				yr=ygewei+yshibai*10;
				color=change(u(6));
			
	
			}
}
}

