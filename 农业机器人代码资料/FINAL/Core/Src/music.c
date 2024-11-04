#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "music.h"
#include "NRF24L01.h"
#include "Emm_v5.h"
/**
 * @brief  初始化GPIO
 * @param  无
 * @retval 无
 */
void JQ8900_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE(); // 使能GPIOB时钟

    // 配置PB10为开漏输出模式
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief  发送字节函数
 * @param  字节数据
 * @retval 无
 */
void SendData(uint8_t Data)
{
    uint8_t m;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // PB10置高电平
    HAL_Delay(1); // 延时1ms

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // PB10置低电平
    HAL_Delay(4); // 延时4ms

    for (m = 0; m < 8; m++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        if (Data & 0x01)
        {
            HAL_Delay(1); // 延时1ms
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
            HAL_Delay(21); // 延时21ms
        }
        else
        {
            HAL_Delay(21); // 延时21ms
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
            HAL_Delay(1); // 延时1ms
        }
        Data >>= 1;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
}

/**
 * @brief  选曲播放函数
 * @param  曲目号Num_L(0x__), Num_H(0x__)
 * @param  xms 延时时间（毫秒）
 * @retval 无
 */
void Play_Spe_Song(uint8_t Num_L, uint8_t Num_H, uint32_t xms)
{
    SendData(0x0A);
    if (Num_H == 0)
    {
        SendData(Num_L);
        HAL_Delay(xms); // 根据音频时长进行调整的延时
    }
    else
    {
        SendData(Num_L);
        SendData(Num_H);
        HAL_Delay(xms); // 根据音频时长进行调整的延时
    }
    SendData(0x0B);
}
void play1(void){
	HAL_GPIO_WritePin(JQ_C4_GPIO_Port,JQ_C4_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(JQ_C2_GPIO_Port,JQ_C2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(JQ_C3_GPIO_Port,JQ_C3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(JQ_C1_GPIO_Port,JQ_C1_Pin,GPIO_PIN_RESET);
	delay_ms(100);
	HAL_GPIO_WritePin(JQ_C1_GPIO_Port,JQ_C1_Pin,GPIO_PIN_SET);
	delay_ms(2000);
	delay_ms(1000);
}
void play3(void){
	HAL_GPIO_WritePin(JQ_C4_GPIO_Port,JQ_C4_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(JQ_C2_GPIO_Port,JQ_C2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(JQ_C1_GPIO_Port,JQ_C1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(JQ_C3_GPIO_Port,JQ_C3_Pin,GPIO_PIN_RESET);
	delay_ms(100);
	HAL_GPIO_WritePin(JQ_C3_GPIO_Port,JQ_C3_Pin,GPIO_PIN_SET);
	delay_ms(1500);
	delay_ms(500);
}
void play2(void){
	HAL_GPIO_WritePin(JQ_C4_GPIO_Port,JQ_C4_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(JQ_C1_GPIO_Port,JQ_C1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(JQ_C3_GPIO_Port,JQ_C3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(JQ_C2_GPIO_Port,JQ_C2_Pin,GPIO_PIN_RESET);
	delay_ms(100);
	HAL_GPIO_WritePin(JQ_C2_GPIO_Port,JQ_C2_Pin,GPIO_PIN_SET);
	delay_ms(1500);
	delay_ms(500);


}
