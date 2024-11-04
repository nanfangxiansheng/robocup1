/**
使用注意事项:
    1.将tjc_usart_hmi.c和tjc_usart_hmi.h 分别导入工程
    2.在需要使用的函数所在的头文件中添加 #include "tjc_usart_hmi.h"
    3.使用前请将 HAL_UART_Transmit_IT() 这个函数改为你的单片机的串口发送单字节函数
    3.TJCPrintf和printf用法一样

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
extern uint8_t receive_cross_flag;//用于接收白色十字的标志
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

RingBuff_t ringBuff;	//创建一个ringBuff的缓冲区
RingBuff_t ringBuff1;//ringBuff1是用于传输x,y坐标的
uint8_t RxBuff[1];
uint8_t RxBuff_1[1];//RxBuff_1是用于传输x,y坐标的

/********************************************************
函数名：  	TJCPrintf
作者：    	wwd
日期：    	2022.10.08
功能：    	向串口打印数据,使用前请将USART_SCREEN_write这个函数改为你的单片机的串口发送单字节函数
输入参数：		参考printf
返回值： 		打印到串口的数量
修改记录：
**********************************************************/

void TJCPrintf(const char *str, ...)
{


	uint8_t end = 0xff;
	char buffer[STR_LENGTH+1];  // 数据长度
	uint8_t i = 0;
	va_list arg_ptr;
	va_start(arg_ptr, str);
	vsnprintf(buffer, STR_LENGTH+1, str, arg_ptr);
	va_end(arg_ptr);
	while ((i < STR_LENGTH) && (i < strlen(buffer)))
	{
		HAL_UART_Transmit_IT(&huart5, (uint8_t *)(buffer) + i++, 1);
		while(huart5.gState != HAL_UART_STATE_READY);	//等待发送完毕
	}

	HAL_UART_Transmit_IT(&huart5, &end, 1);			//这个函数改为你的单片机的串口发送单字节函数
	while(huart5.gState != HAL_UART_STATE_READY);	//等待发送完毕
	HAL_UART_Transmit_IT(&huart5, &end, 1);			//这个函数改为你的单片机的串口发送单字节函数
	while(huart5.gState != HAL_UART_STATE_READY);	//等待发送完毕
	HAL_UART_Transmit_IT(&huart5, &end, 1);			//这个函数改为你的单片机的串口发送单字节函数
	while(huart5.gState != HAL_UART_STATE_READY);	//等待发送完毕

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
函数名：  	HAL_UART_RxCpltCallback
作者：
日期：    	2022.10.08
功能：    	串口接收中断,将接收到的数据写入环形缓冲区
输入参数：
返回值： 		void
修改记录：
**********************************************************/
extern uint8_t Rxdata;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART3)//用于JY901S模块
	{
		HAL_UART_Receive_IT(&huart3,&Rxdata,1);
		uart3_read_data(Rxdata);	//处理数据
	}
  if(huart->Instance==USART1){//接收K210传回的数据
			//接收白色十字坐标仅仅接收一次
		  writeRingBuff(RxBuff[0]);
		  HAL_UART_Receive_IT(&huart1,RxBuff,1);
	}
	if(huart->Instance==UART4){//接收步进电机的数据
		writeRingBuff1(RxBuff_1[0]);
		HAL_UART_Receive_IT(&huart4,RxBuff_1,1);
		
	
	} 	
}



/********************************************************
函数名：  	initRingBuff
作者：
日期：    	2022.10.08
功能：    	初始化环形缓冲区
输入参数：
返回值： 		void
修改记录：
**********************************************************/
void initRingBuff(void)
{
  //初始化相关信息
  ringBuff.Head = 0;
  ringBuff.Tail = 0;
  ringBuff.Lenght = 0;
}
void initRingBuff1(void)
{
  //初始化相关信息
  ringBuff1.Head = 0;
  ringBuff1.Tail = 0;
  ringBuff1.Lenght = 0;
}



/********************************************************
函数名：  	writeRingBuff
作者：
日期：    	2022.10.08
功能：    	往环形缓冲区写入数据
输入参数：
返回值： 		void
修改记录：
**********************************************************/
void writeRingBuff(uint8_t data)
{
  if(ringBuff.Lenght >= RINGBUFF_LEN) //判断缓冲区是否已满
  {
    return ;
  }
  ringBuff.Ring_data[ringBuff.Tail]=data;
  ringBuff.Tail = (ringBuff.Tail+1)%RINGBUFF_LEN;//防止越界非法访问
  ringBuff.Lenght++;

}
void writeRingBuff1(uint8_t data)
{
  if(ringBuff1.Lenght >= RINGBUFF_LEN) //判断缓冲区是否已满
  {
    return ;
  }
  ringBuff1.Ring_data[ringBuff1.Tail]=data;
  ringBuff1.Tail = (ringBuff1.Tail+1)%RINGBUFF_LEN;//防止越界非法访问
  ringBuff1.Lenght++;

}
/**
@Brief 用于向k210发送指令
@Param num
@Retval NULL
*/
void sendnumber(uint32_t number)
{
    char buffer1[12]; // 足够存储32位整数的字符串表示（包括负号和终止符'\0'）
	  sprintf(buffer1, "%d", number); // 将整数转换为字符串
    HAL_UART_Transmit_IT(&huart1, ( uint8_t*)buffer1, strlen(buffer1)); // 发送字符串
}



/********************************************************
函数名：  	deleteRingBuff
作者：
日期：    	2022.10.08
功能：    	删除串口缓冲区中相应长度的数据
输入参数：		要删除的长度
返回值： 		void
修改记录：
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

		if(ringBuff.Lenght == 0)//判断非空
		{
		initRingBuff();
		return;
		}
		ringBuff.Head = (ringBuff.Head+1)%RINGBUFF_LEN;//防止越界非法访问
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

		if(ringBuff1.Lenght == 0)//判断非空
		{
		initRingBuff1();
		return;
		}
		ringBuff1.Head = (ringBuff1.Head+1)%RINGBUFF_LEN;//防止越界非法访问
		ringBuff1.Lenght--;

	}

}


/********************************************************
函数名：  	read1BFromRingBuff
作者：
日期：    	2022.10.08
功能：    	从串口缓冲区读取1字节数据
输入参数：		position:读取的位置
返回值： 		所在位置的数据(1字节)
修改记录：
**********************************************************/
uint8_t read1BFromRingBuff(uint16_t position)
{
	uint16_t realPosition = (ringBuff.Head + position) % RINGBUFF_LEN;

	return ringBuff.Ring_data[realPosition];
}
uint8_t read1BFromRingBuff_1(uint16_t position)//用于传输x,y坐标
{
	uint16_t realPosition = (ringBuff1.Head + position) % RINGBUFF_LEN;

	return ringBuff1.Ring_data[realPosition];
}



/********************************************************
函数名：  	getRingBuffLenght
作者：
日期：    	2022.10.08
功能：    	获取串口缓冲区的数据数量
输入参数：
返回值： 		串口缓冲区的数据数量
修改记录：
**********************************************************/
uint16_t getRingBuffLenght()
{
	return ringBuff.Lenght;
}
uint16_t getRingBuffLenght_1(){
	return ringBuff1.Lenght;
}


/********************************************************
函数名：  	isRingBuffOverflow
作者：
日期：    	2022.10.08
功能：    	判断环形缓冲区是否已满
输入参数：
返回值： 		1:环形缓冲区已满 , 2:环形缓冲区未满
修改记录：
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

