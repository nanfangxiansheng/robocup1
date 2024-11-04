#include "jy901s.h"
#include <string.h>
#include "usart.h"
#include "gpio.h"
#include "NRF24L01.h"
struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;
extern uint8_t Rxdata;
extern float gyro_data;
extern float gyroz,gyroz_last;
extern float anglez;
extern float gyro_datax;
extern float gyrox,gyrox_last;
extern float anglex;
extern float gyro_datay;
extern float gyroy,gyroy_last,angley;
extern int zyaw;
extern int yroll;
extern int xpitch;
char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//进入加速度校准模式
char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//保存当前配置

char MAGNETICCALAM[5] = {0XFF,0XAA,0X01,0X07,0X00};
char SAVEMAGNETICCALAM[5] = {0XFF,0XAA,0X00,0X00,0X00};

//用串口3给JY模块发送指令
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<5;i++){
	
	UART3_send_char(cmd[i]);
	}
	
}



void uart3_read_data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[256];
	static unsigned char ucRxCount = 0;	
	
	
	ucRxBuffer[ucRxCount++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) 				//数据头
	{
		ucRxCount=0;
		return;
	}
	if (ucRxCount<11) {return;}			//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
		}
		ucRxCount=0;	//清空缓存区
	}
}

void JY901S_init(void){
	HAL_UART_Receive_IT(&huart3,&Rxdata,1);

	sendcmd(ACCCALSW);delay_ms(100); //加速度计校准
	sendcmd(SAVACALSW);delay_ms(100);//保存当前配置

	sendcmd(MAGNETICCALAM);	delay_ms(100);   //磁力计校准
	sendcmd(SAVEMAGNETICCALAM);delay_ms(100);//保存当前配置

}
/**
@Brief 重启JY901S且清零其示数
@Param NULL
@Retval NULL
*/
void JY901S_RESET(void){
	HAL_GPIO_WritePin(JY_CON_GPIO_Port,JY_CON_Pin,GPIO_PIN_SET);
	delay_ms(5000);
	HAL_GPIO_WritePin(JY_CON_GPIO_Port,JY_CON_Pin,GPIO_PIN_RESET);
	JY901S_init();
	delay_ms(1000);
	gyro_data=0;
	gyroz=0;
	gyroz_last=0;
	anglez=0;
	gyro_datax=0;gyrox=0;
	gyrox_last=0;
	gyro_datay=0;
	gyroy=0;
	gyroy_last=0;
	anglex=0;
	angley=0;
	anglez=0;
	zyaw=0;
	xpitch=0;
	yroll=0;
	delay_ms(5000);
	
	
	

}
