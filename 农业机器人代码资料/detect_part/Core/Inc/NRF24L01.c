# include "NRF24L01.h"
#include "NRF24L01_Ins.h"
#include "stm32f1xx_hal.h"                  // Device header

/***************************************************************************

��Ҫ����ʹ��˵����

NRF24L01_Pin_Init()		���ų�ʼ��, �޲������޷���ֵ��һ�㲻���ã���ʼ��ֱ�ӵ���NRF24L01_Init()����

*---------------------------------------------------------------------------*
NRF24L01_Write_Reg(uint8_t Reg, uint8_t Value)		дNRF24L01�Ĵ���

������Reg, �Ĵ�����ַ
	  Value��Ҫд�������
����ֵ��״ֵ̬
*---------------------------------------------------------------------------*
NRF24L01_Read_Reg(uint8_t Reg)		��NRF24L01�Ĵ���

������Reg���Ĵ�����ַ
����ֵ��״ֵ̬
*---------------------------------------------------------------------------*
NRF24L01_Read_Buf(uint8_t Reg, uint8_t *Buf, uint8_t Len)	һ�ζ�NRF24L01�Ĵ�������ֽ�

������Reg���Ĵ�����ַ��*Buf���Ĵ����ֽڶ�����洢�����飻Len��Ҫ�������ֽڸ���
����ֵ��״ֵ̬
*---------------------------------------------------------------------------*
NRF24L01_Write_Buf(uint8_t Reg, uint8_t *Buf, uint8_t Len)		һ��дNRF24L01�Ĵ�������ֽ�

������Reg���Ĵ�����ַ��*Buf���Ĵ���д���ֽڵĴ�����飻Len��Ҫ�������ֽڸ���
����ֵ��״ֵ̬
*---------------------------------------------------------------------------*
NRF24L01_GetRxBuf(uint8_t *Buf)		�������յ������ݣ�Ϊ����ֽ�

������*Buf����ֽڴ�ŵ�����
����ֵ��״ֵ̬
*---------------------------------------------------------------------------*
NRF24L01_SendTxBuf(uint8_t *Buf)	���Ͷ���ֽ�

������*BufҪ���͵��ֽڴ�ŵ�����
����ֵ��״ֵ̬
*---------------------------------------------------------------------------*
NRF24L01_Check(void)				����NRF24L01�Ƿ����

����ֵ��1�������ڣ�0������
*---------------------------------------------------------------------------*
NRF24L01_Init()						NRF24L01��ʼ������������NRF24L01�Ƿ���ڡ��շ����ó�ʼ������ʼ��ֱ�ӵ��øú�������

*---------------------------------------------------------------------------*
NRF24L01_RT_Init(void) 				NRF24L01�շ����ó�ʼ��
*---------------------------------------------------------------------------*
NRF24L01_SendBuf(uint8_t *Buf)		NRF24L01���Ͷ���ֽ�����

������Ҫ�����ֽڴ�ŵ�����
*---------------------------------------------------------------------------*
uint8_t NRF24L01_Get_Value_Flag()	��ȡ�жϱ�־��һ�����յ����ݣ�����0

����ֵ��0�����յ����ݣ�1��δ���յ�����

**************************     By:����ͨͨOrdinary    **********************************/


/***************    �ǵ�����֧��һ��Ŷ��QAQ  *******************************************/


#define TX_ADR_WIDTH    5     //5�ֽڵ�ַ���
#define RX_ADR_WIDTH    5     //5�ֽڵ�ַ���
#define TX_PLOAD_WIDTH  32    //32�ֽ���Ч���ݿ��
#define RX_PLOAD_WIDTH  32    //32�ֽ���Ч���ݿ��

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

uint8_t NRF24L01_Get_Value_Flag()
{
	return R_IRQ();
}
// ע��:nus��ֵ,��Ҫ����798915us(���ֵ��2^24/fac_us {fac_us=21})
void Delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD = nus * 72; // ʱ����أ�fac_us��ʱ��Ƶ���йأ����磺168Mhzʱ
                                  // fac_us=168��168/168M=1us
    SysTick->VAL = 0x00;                      // ��ռ�����
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // ��ʼ����
    do
    {
        temp = SysTick->CTRL; // ��ȡ���Ƽ�״̬�Ĵ�����ֵ

    } while ((temp & 0x01) && !(temp & (1 << 16))); // �ȴ�ʱ�䵽�ʹ����λ16Ϊ0��δ�Ƶ�0��
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;      // �رռ�����
    SysTick->VAL = 0X00;                            // ��ռ�����
}
