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
char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//������ٶ�У׼ģʽ
char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//���浱ǰ����

char MAGNETICCALAM[5] = {0XFF,0XAA,0X01,0X07,0X00};
char SAVEMAGNETICCALAM[5] = {0XFF,0XAA,0X00,0X00,0X00};

//�ô���3��JYģ�鷢��ָ��
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
	
	
	ucRxBuffer[ucRxCount++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) 				//����ͷ
	{
		ucRxCount=0;
		return;
	}
	if (ucRxCount<11) {return;}			//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
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
		ucRxCount=0;	//��ջ�����
	}
}

void JY901S_init(void){
	HAL_UART_Receive_IT(&huart3,&Rxdata,1);

	sendcmd(ACCCALSW);delay_ms(100); //���ٶȼ�У׼
	sendcmd(SAVACALSW);delay_ms(100);//���浱ǰ����

	sendcmd(MAGNETICCALAM);	delay_ms(100);   //������У׼
	sendcmd(SAVEMAGNETICCALAM);delay_ms(100);//���浱ǰ����

}
/**
@Brief ����JY901S��������ʾ��
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
