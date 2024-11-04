#ifndef __TJCUSARTHMI_H_
#define __TJCUSARTHMI_H_


#include <stdio.h>
#include <stm32f1xx_hal.h>
/**
	打印到屏幕串口
*/
void TJCPrintf(const char *cmd, ...);
void initRingBuff(void);
void initRingBuff1(void);
void writeRingBuff(uint8_t data);
void deleteRingBuff(uint16_t size);
void deleteRingBuff_k(uint16_t size1);
void writeRingBuff1(uint8_t data);
uint16_t getRingBuffLenght(void);
uint16_t getRingBuffLenght_1(void);
uint8_t read1BFromRingBuff(uint16_t position);
uint8_t read1BFromRingBuff_1(uint16_t position);
uint8_t isRingBuffOverflow1();
uint8_t isRingBuffOverflow();

void show(void);//show sent order
void jiaozhun(void);//calibrate the clock
#define RINGBUFF_LEN	(500)     //定义最大接收字节数 500

#define usize getRingBuffLenght()
#define gsize getRingBuffLenght_1()
#define code_c() initRingBuff()
#define udelete(x) deleteRingBuff(x)
#define gdelete(x1) deleteRingBuff_k(x1)
#define u(x) read1BFromRingBuff(x)
#define g(x) read1BFromRingBuff_1(x)
void judge(void);
extern uint8_t RxBuff[1];
extern uint8_t RxBuff_1[1];
extern int usarflag;
void sendnumber(uint32_t number);
#endif
