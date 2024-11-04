#include "NRF24L01.h"

 #include "tjc_usart_hmi.h"// Device header
extern uint8_t rj;//用于接收传输的旱情信息
extern uint8_t line1[6];
extern uint8_t line2[6];
extern uint8_t line3[6];
extern uint8_t Buf[32];
#include "NRF24L01_Ins.h"
#include "stm32f1xx_hal.h"                  // Device header

/***************************************************************************

主要函数使用说明：

NRF24L01_Pin_Init()		引脚初始化, 无参数，无返回值，一般不调用，初始化直接调用NRF24L01_Init()即可

*---------------------------------------------------------------------------*
NRF24L01_Write_Reg(uint8_t Reg, uint8_t Value)		写NRF24L01寄存器

参数：Reg, 寄存器地址
	  Value：要写入的数据
返回值：状态值
*---------------------------------------------------------------------------*
NRF24L01_Read_Reg(uint8_t Reg)		读NRF24L01寄存器

参数：Reg：寄存器地址
返回值：状态值
*---------------------------------------------------------------------------*
NRF24L01_Read_Buf(uint8_t Reg, uint8_t *Buf, uint8_t Len)	一次读NRF24L01寄存器多个字节

参数：Reg：寄存器地址；*Buf：寄存器字节读出后存储的数组；Len：要读出的字节个数
返回值：状态值
*---------------------------------------------------------------------------*
NRF24L01_Write_Buf(uint8_t Reg, uint8_t *Buf, uint8_t Len)		一次写NRF24L01寄存器多个字节

参数：Reg：寄存器地址；*Buf：寄存器写入字节的存放数组；Len：要读出的字节个数
返回值：状态值
*---------------------------------------------------------------------------*
NRF24L01_GetRxBuf(uint8_t *Buf)		读出接收到的数据，为多个字节

参数：*Buf多个字节存放的数组
返回值：状态值
*---------------------------------------------------------------------------*
NRF24L01_SendTxBuf(uint8_t *Buf)	发送多个字节

参数：*Buf要发送的字节存放的数组
返回值：状态值
*---------------------------------------------------------------------------*
NRF24L01_Check(void)				检验NRF24L01是否存在

返回值：1：不存在，0：存在
*---------------------------------------------------------------------------*
NRF24L01_Init()						NRF24L01初始化，包含检验NRF24L01是否存在、收发配置初始化，初始化直接调用该函数即可

*---------------------------------------------------------------------------*
NRF24L01_RT_Init(void) 				NRF24L01收发配置初始化
*---------------------------------------------------------------------------*
NRF24L01_SendBuf(uint8_t *Buf)		NRF24L01发送多个字节数据

参数：要发送字节存放的数组
*---------------------------------------------------------------------------*
uint8_t NRF24L01_Get_Value_Flag()	获取中断标志，一旦接收到数据，返回0

返回值：0：接收到数据；1：未接收到数据

**************************     By:普普通通Ordinary    **********************************/


/***************    记得三连支持一下哦！QAQ  *******************************************/


#define TX_ADR_WIDTH    5     //5字节地址宽度
#define RX_ADR_WIDTH    5     //5字节地址宽度
#define TX_PLOAD_WIDTH  32    //32字节有效数据宽度
#define RX_PLOAD_WIDTH  32    //32字节有效数据宽度

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xFF, 0xFF, 0xFF, 0xFF, 0xEE}; 
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xFF, 0xFF, 0xFF, 0xFF, 0xEE}; 


void W_SS(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (GPIO_PinState)BitValue);
}

void W_CE(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (GPIO_PinState)BitValue);
}

void W_SCK(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (GPIO_PinState)BitValue);
}

void W_MOSI(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (GPIO_PinState)BitValue);
}

uint8_t R_MISO(void)
{
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
}

uint8_t R_IRQ(void)
{
	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
}

uint8_t SPI_SwapByte(uint8_t Byte)  
{
	uint8_t i;
	for(i = 0; i < 8; i ++) 
	{
		if((uint8_t)(Byte & 0x80) == 0x80)
		{
			W_MOSI(1);
		} 		
		else 
		{
			W_MOSI(0);
		}			
		Byte = (Byte << 1);			
		W_SCK(1);					
		Byte |= R_MISO();	        
		W_SCK(0);					
	}
	return Byte;
}
uint8_t NRF24L01_Write_Reg(uint8_t Reg, uint8_t Value)
{
	uint8_t Status;

	W_SS(0);                  
  	Status = SPI_SwapByte(Reg);
	SPI_SwapByte(Value);	
	W_SS(1);                 

	return Status;
}

uint8_t NRF24L01_Read_Reg(uint8_t Reg)
{
 	uint8_t Value;

	W_SS(0);              
  	SPI_SwapByte(Reg);		
	Value = SPI_SwapByte(NOP);
	W_SS(1);             	

	return Value;
}




uint8_t NRF24L01_Read_Buf(uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
	uint8_t Status, i;
	W_SS(0);                    	
	Status =SPI_SwapByte(Reg);		
 	for(i = 0; i < Len; i ++)
	{
		Buf[i] = SPI_SwapByte(NOP);	
	}
	W_SS(1);                 		
	return Status;        			
}

uint8_t NRF24L01_Write_Buf(uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
	uint8_t Status, i;
	W_SS(0); 
	Status = SPI_SwapByte(Reg);		
	for(i = 0; i < Len; i ++)
	{
		SPI_SwapByte(*Buf ++); 		
	}
	W_SS(1); 
	return Status;          		
}							  					   

uint8_t NRF24L01_GetRxBuf(uint8_t *Buf)
{
	uint8_t State;
	State = NRF24L01_Read_Reg(STATUS);  			
	NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, State);
	if(State & RX_OK)								
	{                                                       
		W_CE(1);											
		NRF24L01_Read_Buf(RD_RX_PLOAD, Buf, RX_PLOAD_WIDTH);
		NRF24L01_Write_Reg(FLUSH_RX, NOP);					
		W_CE(1);				
		Delay_us(150);
		return 0; 
	}	   
	return 1;
}
uint8_t NRF24L01_SendTxBuf(uint8_t *Buf)
{
	uint8_t State;
   
	W_CE(0);												
  	NRF24L01_Write_Buf(WR_TX_PLOAD, Buf, TX_PLOAD_WIDTH);	
 	W_CE(1);												
	while(R_IRQ() == 1);									
	State = NRF24L01_Read_Reg(STATUS);  					
	NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, State); 		
	if(State&MAX_TX)				 						
	{
		NRF24L01_Write_Reg(FLUSH_TX, NOP);					
		return MAX_TX; 
	}
	if(State & TX_OK)	
	{
		return TX_OK;
	}
	return NOP;	
}

uint8_t NRF24L01_Check(void)
{
	uint8_t check_in_buf[5] = {0x11 ,0x22, 0x33, 0x44, 0x55};
	uint8_t check_out_buf[5] = {0x00};

	W_SCK(0);
	W_SS(1);   
	W_CE(0);	

	NRF24L01_Write_Buf(nRF_WRITE_REG + TX_ADDR, check_in_buf, 5);

	NRF24L01_Read_Buf(nRF_READ_REG + TX_ADDR, check_out_buf, 5);

	if((check_out_buf[0] == 0x11) && (check_out_buf[1] == 0x22) && (check_out_buf[2] == 0x33) && (check_out_buf[3] == 0x44) && (check_out_buf[4] == 0x55))
	{
		return 0;
	}
	else 
	{
		return 1;
	}
}			


void NRF24L01_RT_Init(void) 
{	
	W_CE(0);		  
  	NRF24L01_Write_Reg(nRF_WRITE_REG+RX_PW_P0, RX_PLOAD_WIDTH);
	NRF24L01_Write_Reg(FLUSH_RX, NOP);									
  	NRF24L01_Write_Buf(nRF_WRITE_REG + TX_ADDR, (uint8_t*)TX_ADDRESS, TX_ADR_WIDTH);
  	NRF24L01_Write_Buf(nRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH);   
  	NRF24L01_Write_Reg(nRF_WRITE_REG + EN_AA, 0x01);     
  	NRF24L01_Write_Reg(nRF_WRITE_REG + EN_RXADDR, 0x01); 
  	NRF24L01_Write_Reg(nRF_WRITE_REG + SETUP_RETR, 0x1A);
  	NRF24L01_Write_Reg(nRF_WRITE_REG + RF_CH, 0);        
  	NRF24L01_Write_Reg(nRF_WRITE_REG + RF_SETUP, 0x0F);  
  	NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG, 0x0F);    
	W_CE(1);									  
}

void NRF24L01_Init()
{
	while(NRF24L01_Check());
	NRF24L01_RT_Init();
	
}

void NRF24L01_SendBuf(uint8_t *Buf)
{
	W_CE(0);									
	NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG, 0x0E);   
	W_CE(1);
	Delay_us(15);
	NRF24L01_SendTxBuf(Buf);						
	W_CE(0);
	NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG, 0x0F);
	W_CE(1);	
}

uint8_t NRF24L01_Get_Value_Flag(void)
{
	return R_IRQ();
}

void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
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


void judge_receive(void){
		line1[0]=Buf[0];
		line1[1]=Buf[1];
		line1[2]=Buf[2];
		line1[3]=Buf[3];
		line1[4]=Buf[4];
		line1[5]=Buf[5];
		line2[0]=Buf[6];
		line2[1]=Buf[7];
		line2[2]=Buf[8];
		line2[3]=Buf[9];
		line2[4]=Buf[10];
		line2[5]=Buf[11];	
		line3[0]=Buf[12];
		line3[1]=Buf[13];
		line3[2]=Buf[14];
		line3[3]=Buf[15];
		line3[4]=Buf[16];
		line3[5]=Buf[17];
}
/**
@Brief  用于清除接收旱情信息的缓冲区
@Param  NULL
@Retval NULL
*/
void Buf_clear1(void){
	uint8_t i;
	for(i=0;i<31;i++){
		Buf[i]=0;
	}


}
void hanqing_init(void){
		line1[0]=0;
		line1[1]=0;
		line1[2]=0;
		line1[3]=0;
		line1[4]=0;
		line1[5]=0;
		line2[0]=0;
		line2[1]=0;
		line2[2]=0;
		line2[3]=0;
		line2[4]=0;
		line2[5]=0;
		line3[0]=0;
		line3[1]=0;
		line3[2]=0;
		line3[3]=0;
		line3[4]=0;
		line3[5]=0;
}
void modify(void){//用于特色情况下的紧急调整
//	if(Buf[0]!=9){//此时出现了问题
//		line1[0]=line1[1];
//		line1[1]=line1[2];
//		line1[2]=line1[3];
//		line1[3]=line1[4];
//		line1[4]=line1[5];
//		line1[5]=line2[0];
//		line2[0]=line2[1];
//		line2[1]=line2[2];
//		line2[2]=line2[3];
//		line2[3]=line2[4];
//		line2[4]=line2[5];
//	  line2[5]=line3[0];
//		line3[0]=line3[1];
//		line3[1]=line3[2];
//		line3[2]=line3[3];
//		line3[3]=line3[4];
//		line3[4]=line3[5];
//		line3[5]=Buf[19];
//	}
	if(line1[0]!=1&&line1[0]!=2&&line1[0]!=3&&line1[1]!=0&&line1[2]!=0&&line1[3]!=0&&line1[4]!=0&&line1[5]!=0){
		line1[0]=line1[1];
		line1[1]=line1[2];
		line1[2]=line1[3];
		line1[3]=line1[4];
		line1[4]=line1[5];
		line1[5]=line2[0];
		line2[0]=line2[1];
		line2[1]=line2[2];
		line2[2]=line2[3];
		line2[3]=line2[4];
		line2[4]=line2[5];
	  line2[5]=line3[0];
		line3[0]=line3[1];
		line3[1]=line3[2];
		line3[2]=line3[3];
		line3[3]=line3[4];
		line3[4]=line3[5];
		line3[5]=2;
	
	
	}


}
void show_hanqing(void){//在TJC串口屏上显示旱情信息
//		TJCPrintf("zaizhi.n0.val=%d\xff\xff\xff",line2[0]);
//		TJCPrintf("zaizhi.n1.val=%d\xff\xff\xff",line2[1]);
//		TJCPrintf("zaizhi.n2.val=%d\xff\xff\xff",line2[2]);
//	  TJCPrintf("zaizhi.n3.val=%d\xff\xff\xff",line2[3]);
//	  TJCPrintf("zaizhi.n4.val=%d\xff\xff\xff",line2[4]);
//	  TJCPrintf("zaizhi.n5.val=%d\xff\xff\xff",line2[5]);
//		TJCPrintf("zaizhi.n6.val=%d\xff\xff\xff",line1[0]);
//		TJCPrintf("zaizhi.n7.val=%d\xff\xff\xff",line1[1]);
//		TJCPrintf("zaizhi.n8.val=%d\xff\xff\xff",line1[2]);
//	  TJCPrintf("zaizhi.n9.val=%d\xff\xff\xff",line1[3]);
//	  TJCPrintf("zaizhi.n10.val=%d\xff\xff\xff",line1[4]);
//	  TJCPrintf("zaizhi.n11.val=%d\xff\xff\xff",line1[5]);
//		TJCPrintf("zaizhi.n12.val=%d\xff\xff\xff",line3[0]);
//		TJCPrintf("zaizhi.n13.val=%d\xff\xff\xff",line3[1]);
//		TJCPrintf("zaizhi.n14.val=%d\xff\xff\xff",line3[2]);
//	  TJCPrintf("zaizhi.n15.val=%d\xff\xff\xff",line3[3]);
//	  TJCPrintf("zaizhi.n16.val=%d\xff\xff\xff",line3[4]);
//	  TJCPrintf("zaizhi.n17.val=%d\xff\xff\xff",line3[5]);
	color_judge(line2[0],0);
	color_judge(line2[1],1);
	color_judge(line2[2],2);
	color_judge(line2[3],3);
	color_judge(line2[4],4);
	color_judge(line2[5],5);
	color_judge(line1[0],6);
	color_judge(line1[1],7);
	color_judge(line1[2],8);
	color_judge(line1[3],9);
	color_judge(line1[4],10);
	color_judge(line1[5],11);
	color_judge(line3[0],12);
	color_judge(line3[1],13);
	color_judge(line3[2],14);
	color_judge(line3[3],15);
	color_judge(line3[4],16);
	color_judge(line3[5],17);
}
void color_judge(uint8_t i,uint8_t k){
	if(i==1){
		TJCPrintf("zaizhi.n%d.bco=2016\xff\xff\xff",k);
	}
	else if(i==2){
		TJCPrintf("zaizhi.n%d.bco=1055\xff\xff\xff",k);
	}
	else if(i==3){
		TJCPrintf("zaizhi.n%d.bco=63488\xff\xff\xff",k);
	}
}
//line1->B
//line2->A
//line3->C
