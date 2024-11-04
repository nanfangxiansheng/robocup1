#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "music.h"
#include "NRF24L01.h"
#include "Emm_v5.h"
/**
 * @brief  ��ʼ��GPIO
 * @param  ��
 * @retval ��
 */
void JQ8900_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE(); // ʹ��GPIOBʱ��

    // ����PB10Ϊ��©���ģʽ
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief  �����ֽں���
 * @param  �ֽ�����
 * @retval ��
 */
void SendData(uint8_t Data)
{
    uint8_t m;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // PB10�øߵ�ƽ
    HAL_Delay(1); // ��ʱ1ms

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // PB10�õ͵�ƽ
    HAL_Delay(4); // ��ʱ4ms

    for (m = 0; m < 8; m++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        if (Data & 0x01)
        {
            HAL_Delay(1); // ��ʱ1ms
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
            HAL_Delay(21); // ��ʱ21ms
        }
        else
        {
            HAL_Delay(21); // ��ʱ21ms
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
            HAL_Delay(1); // ��ʱ1ms
        }
        Data >>= 1;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
}

/**
 * @brief  ѡ�����ź���
 * @param  ��Ŀ��Num_L(0x__), Num_H(0x__)
 * @param  xms ��ʱʱ�䣨���룩
 * @retval ��
 */
void Play_Spe_Song(uint8_t Num_L, uint8_t Num_H, uint32_t xms)
{
    SendData(0x0A);
    if (Num_H == 0)
    {
        SendData(Num_L);
        HAL_Delay(xms); // ������Ƶʱ�����е�������ʱ
    }
    else
    {
        SendData(Num_L);
        SendData(Num_H);
        HAL_Delay(xms); // ������Ƶʱ�����е�������ʱ
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
