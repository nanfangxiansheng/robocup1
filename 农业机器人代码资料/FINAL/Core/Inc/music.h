#ifndef _MUSIC_H
#define _MUSIC_H
#include "stm32f1xx_hal.h"
void JQ8900_Init(void);
void SendData(uint8_t Data);
void Play_Spe_Song(uint8_t Num_L,uint8_t Num_H,uint32_t xms);
void play1(void);//��΢�ɺ�
void play2(void );//һ��ɺ�
void play3(void );//���ظɺ�
#endif