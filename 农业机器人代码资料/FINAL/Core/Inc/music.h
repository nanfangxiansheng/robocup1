#ifndef _MUSIC_H
#define _MUSIC_H
#include "stm32f1xx_hal.h"
void JQ8900_Init(void);
void SendData(uint8_t Data);
void Play_Spe_Song(uint8_t Num_L,uint8_t Num_H,uint32_t xms);
void play1(void);//轻微干旱
void play2(void );//一般干旱
void play3(void );//严重干旱
#endif