/*******************************************************************************
* 文件名： LobotServoController.c
* 作者： 深圳乐幻索尔科技
* 日期：20160806
* LSC系列舵机控制板二次开发示例
*******************************************************************************/
#include "stm32f1xx_hal.h"
#include "LobotServoController.h"
#include <stdarg.h>
#include <string.h>
#include "stdio.h"
#include "stdlib.h"
#include "NRF24L01.h"
#include "tjc_usart_hmi.h"
#include "tim.h"
#define GET_LOW_BYTE(A) ((uint8_t)(A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//宏函数 获得A的高八位

extern bool isUartRxCompleted;
extern uint8_t position1[32];
uint8_t LobotTxBuf[128];  //发送缓存
uint8_t LobotRxBuf[16];
uint16_t batteryVolt;
uint8_t moveflag;//用于小车行进时候的标志位
uint32_t movei;//用于小车行进时候的计数
/*********************************************************************************
 * Function:  moveServo
 * Description： 控制单个舵机转动
 * Parameters:   sevoID:舵机ID，Position:目标位置,Time:转动时间
                    舵机ID取值:0<=舵机ID<=31,Time取值: Time > 0
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
	if (servoID > 31 || !(Time > 0)) {  //舵机ID不能打于31,可根据对应控制板修改
		return;
	}
	LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;    //填充帧头
	LobotTxBuf[2] = 8;
	LobotTxBuf[3] = CMD_SERVO_MOVE;           //数据长度=要控制舵机数*3+5，此处=1*3+5//填充舵机移动指令
	LobotTxBuf[4] = 1;                        //要控制的舵机个数
	LobotTxBuf[5] = GET_LOW_BYTE(Time);       //取得时间的低八位
	LobotTxBuf[6] = GET_HIGH_BYTE(Time);      //取得时间的高八位
	LobotTxBuf[7] = servoID;                  //舵机ID
	LobotTxBuf[8] = GET_LOW_BYTE(Position);   //取得目标位置的低八位
	LobotTxBuf[9] = GET_HIGH_BYTE(Position);  //取得目标位置的高八位

	uartWriteBuf(LobotTxBuf, 10);
}

/*********************************************************************************
 * Function:  moveServosByArray
 * Description： 控制多个舵机转动
 * Parameters:   servos[]:舵机结体数组，Num:舵机个数,Time:转动时间
                    0 < Num <= 32,Time > 0
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time)
{
	uint8_t index = 7;
	uint8_t i = 0;

	if (Num < 1 || Num > 32 || !(Time > 0)) {
		return;                                          //舵机数不能为零和大与32，时间不能为零
	}
	LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //填充帧头
	LobotTxBuf[2] = Num * 3 + 5;                       //数据长度 = 要控制舵机数*3+5
	LobotTxBuf[3] = CMD_SERVO_MOVE;                    //填充舵机移动指令
	LobotTxBuf[4] = Num;                               //要控制的舵机个数
	LobotTxBuf[5] = GET_LOW_BYTE(Time);                //取得时间的低八位
	LobotTxBuf[6] = GET_HIGH_BYTE(Time);               //取得时间的高八位

	for (i = 0; i < Num; i++) {                        //循环填充舵机ID和对应目标位置
		LobotTxBuf[index++] = servos[i].ID;              //填充舵机ID
		LobotTxBuf[index++] = GET_LOW_BYTE(servos[i].Position); //填充目标位置低八位
		LobotTxBuf[index++] = GET_HIGH_BYTE(servos[i].Position);//填充目标位置高八位
	}

	uartWriteBuf(LobotTxBuf, LobotTxBuf[2] + 2);             //发送
}

/*********************************************************************************
 * Function:  moveServos
 * Description： 控制多个舵机转动
 * Parameters:   Num:舵机个数,Time:转动时间,...:舵机ID,转动角，舵机ID,转动角度 如此类推
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServos(uint8_t Num, uint16_t Time, ...)
{
	uint8_t index = 7;
	uint8_t i = 0;
	uint16_t temp;
	va_list arg_ptr;  //

	va_start(arg_ptr, Time); //取得可变参数首地址
	if (Num < 1 || Num > 32) {
		return;               //舵机数不能为零和大与32，时间不能小于0
	}
	LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //填充帧头
	LobotTxBuf[2] = Num * 3 + 5;                //数据长度 = 要控制舵机数 * 3 + 5
	LobotTxBuf[3] = CMD_SERVO_MOVE;             //舵机移动指令
	LobotTxBuf[4] = Num;                        //要控制舵机数
	LobotTxBuf[5] = GET_LOW_BYTE(Time);         //取得时间的低八位
	LobotTxBuf[6] = GET_HIGH_BYTE(Time);        //取得时间的高八位

	for (i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
		temp = va_arg(arg_ptr, int);//可参数中取得舵机ID
		LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
		temp = va_arg(arg_ptr, int);  //可变参数中取得对应目标位置
		LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp)); //填充目标位置低八位
		LobotTxBuf[index++] = GET_HIGH_BYTE(temp);//填充目标位置高八位
	}

	va_end(arg_ptr);  //置空arg_ptr

	uartWriteBuf(LobotTxBuf, LobotTxBuf[2] + 2);    //发送
}


/*********************************************************************************
 * Function:  runActionGroup
 * Description： 运行指定动作组
 * Parameters:   NumOfAction:动作组序号, Times:执行次数
 * Return:       无返回
 * Others:       Times = 0 时无限循环
 **********************************************************************************/
void runActionGroup(uint8_t numOfAction, uint16_t Times)
{
	LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;  //填充帧头
	LobotTxBuf[2] = 5;                      //数据长度，数据帧除帧头部分数据字节数，此命令固定为5
	LobotTxBuf[3] = CMD_ACTION_GROUP_RUN;   //填充运行动作组命令
	LobotTxBuf[4] = numOfAction;            //填充要运行的动作组号
	LobotTxBuf[5] = GET_LOW_BYTE(Times);    //取得要运行次数的低八位
	LobotTxBuf[6] = GET_HIGH_BYTE(Times);   //取得要运行次数的高八位

	uartWriteBuf(LobotTxBuf, 7);            //发送
}

/*********************************************************************************
 * Function:  stopActiongGroup
 * Description： 停止动作组运行
 * Parameters:   Speed: 目标速度
 * Return:       无返回
 * Others:
 **********************************************************************************/
void stopActionGroup(void)
{
	LobotTxBuf[0] = FRAME_HEADER;     //填充帧头
	LobotTxBuf[1] = FRAME_HEADER;
	LobotTxBuf[2] = 2;                //数据长度，数据帧除帧头部分数据字节数，此命令固定为2
	LobotTxBuf[3] = CMD_ACTION_GROUP_STOP;   //填充停止运行动作组命令

	uartWriteBuf(LobotTxBuf, 4);      //发送
}
/*********************************************************************************
 * Function:  setActionGroupSpeed
 * Description： 设定指定动作组的运行速度
 * Parameters:   NumOfAction: 动作组序号 , Speed:目标速度
 * Return:       无返回
 * Others:
 **********************************************************************************/
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed)
{
	LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;   //填充帧头
	LobotTxBuf[2] = 5;                       //数据长度，数据帧除帧头部分数据字节数，此命令固定为5
	LobotTxBuf[3] = CMD_ACTION_GROUP_SPEED;  //填充设置动作组速度命令
	LobotTxBuf[4] = numOfAction;             //填充要设置的动作组号
	LobotTxBuf[5] = GET_LOW_BYTE(Speed);     //获得目标速度的低八位
	LobotTxBuf[6] = GET_HIGH_BYTE(Speed);    //获得目标熟读的高八位

	uartWriteBuf(LobotTxBuf, 7);             //发送
}

/*********************************************************************************
 * Function:  setAllActionGroupSpeed
 * Description： 设置所有动作组的运行速度
 * Parameters:   Speed: 目标速度
 * Return:       无返回
 * Others:
 **********************************************************************************/
void setAllActionGroupSpeed(uint16_t Speed)
{
	setActionGroupSpeed(0xFF, Speed);  //调用动作组速度设定，组号为0xFF时设置所有组的速度
}

/*********************************************************************************
 * Function:  getBatteryVoltage
 * Description： 发送获取电池电压命令
 * Parameters:   Timeout：重试次数
 * Return:       无返回
 * Others:
 **********************************************************************************/
void getBatteryVoltage(void)
{
//	uint16_t Voltage = 0;
	LobotTxBuf[0] = FRAME_HEADER;  //填充帧头
	LobotTxBuf[1] = FRAME_HEADER;
	LobotTxBuf[2] = 2;             //数据长度，数据帧除帧头部分数据字节数，此命令固定为2
	LobotTxBuf[3] = CMD_GET_BATTERY_VOLTAGE;  //填充获取电池电压命令

	uartWriteBuf(LobotTxBuf, 4);   //发送
}

void receiveHandle()
{
	//可以根据二次开发手册添加其他指令
	if (isUartRxCompleted) {
		isUartRxCompleted = FALSE;
		switch (LobotRxBuf[3]) {
		case CMD_GET_BATTERY_VOLTAGE: //获取电压
			batteryVolt = (((uint16_t)(LobotRxBuf[5])) << 8) | (LobotRxBuf[4]);
			break;
		default:
			break;
		}
	}
}
/*
mode1->middle,ID1->253
mode2->left,ID1->385
mode3->right,ID1->125
*/
void mode1(void){
	moveServo(1,253,500);
	delay_ms(1000);
	special_delay();
	judge1();
	udelete(8);
}
void mode2(void){
	delay_ms(500);
	sendnumber(2);
	special_delay();
	judge2();
	udelete(8);
}
void mode3(void){
	judge3();
	tianchong();
	
	NRF24L01_SendBuf(position1);
	udelete(20);
}
void sendnumber(uint32_t number)
{
    char buffer[12]; // 足够存储32位整数的字符串表示（包括负号和终止符'\0'）
	  sprintf(buffer, "%d", number); // 将整数转换为字符串
    HAL_UART_Transmit_IT(&huart1, ( uint8_t*)buffer, strlen(buffer)); // 发送字符串
}
int change(int hexnum){
	char hexString[10]; 
  sprintf(hexString, "%X", hexnum);
	long decimalNum = strtol(hexString, NULL, 16);
	return decimalNum;
}
void color_judge(uint8_t i,uint8_t k){
	if(i==1){
		TJCPrintf("page0.n%d.bco=2016\xff\xff\xff",k);
	}
	else if(i==2){
		TJCPrintf("page0.n%d.bco=1055\xff\xff\xff",k);
	}
	else if(i==3){
		TJCPrintf("page0.n%d.bco=63488\xff\xff\xff",k);
	}
}
void show_hanqing(void){
	color_judge(position1[6],0);
	color_judge(position1[7],1);
	color_judge(position1[8],2);
	color_judge(position1[9],3);
	color_judge(position1[10],4);
	color_judge(position1[11],5);
	color_judge(position1[0],6);
	color_judge(position1[1],7);
	color_judge(position1[2],8);
	color_judge(position1[3],9);
	color_judge(position1[4],10);
	color_judge(position1[5],11);
	color_judge(position1[12],12);
	color_judge(position1[13],13);
	color_judge(position1[14],14);
	color_judge(position1[15],15);
	color_judge(position1[16],16);
	color_judge(position1[17],17);


}
void delay_ms(int32_t nms) 

 {

  int32_t temp; 

  SysTick->LOAD = 8000*nms; 

  SysTick->VAL=0X00;//清空计数器 

  SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源 

  do 

  { 

       temp=SysTick->CTRL;//读取当前倒计数值 

  }

     while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达 

     

     SysTick->CTRL=0x00; //关闭计数器 

     SysTick->VAL =0X00; //清空计数器 

 }
void special_delay(void){//用于小车行进时候的延时函数
	moveflag=1;
	movei=0;
	for(movei=0;movei<20;movei++){
		delay_ms(200);
	
	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim2){
		if(moveflag==1){
			movei++;
		  if(movei>=150){
				moveflag=0;
				movei=0;
		}
		}
	}
}
void tianchong(void){
	uint8_t i;
	for(i=0;i<18;i++){
		if(position1[i]==0){
			position1[i]=1;
		
		}
	
	}


}