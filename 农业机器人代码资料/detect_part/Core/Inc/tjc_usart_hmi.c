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

RingBuff_t ringBuff;	//创建一个ringBuff的缓冲区
uint8_t RxBuff[1];



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
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)(buffer) + i++, 1);
		while(huart3.gState != HAL_UART_STATE_READY);	//等待发送完毕
	}

	HAL_UART_Transmit_IT(&huart3, &end, 1);			//这个函数改为你的单片机的串口发送单字节函数
	while(huart3.gState != HAL_UART_STATE_READY);	//等待发送完毕
	HAL_UART_Transmit_IT(&huart3, &end, 1);			//这个函数改为你的单片机的串口发送单字节函数
	while(huart3.gState != HAL_UART_STATE_READY);	//等待发送完毕
	HAL_UART_Transmit_IT(&huart3, &end, 1);			//这个函数改为你的单片机的串口发送单字节函数
	while(huart3.gState != HAL_UART_STATE_READY);	//等待发送完毕

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
函数名：  	HAL_UART_RxCpltCallback
作者：
日期：    	2022.10.08
功能：    	串口接收中断,将接收到的数据写入环形缓冲区
输入参数：
返回值： 		void
修改记录：
**********************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1)	// 判断是由哪个串口触发的中断
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

void judge1(void){//第一次接收数据
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
		if(u(0)!=0x05||u(7)!=0x07)//这是为了判断接收是否有问题
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
