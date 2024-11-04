#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "usart.h"
#include "stdio.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"
/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

#define		ABS(x)		((x) > 0 ? (x) : -(x)) 


typedef enum {
	S_VER   = 0,			/* 读取固件版本和对应的硬件版本 */
	S_RL    = 1,			/* 读取读取相电阻和相电感 */
	S_PID   = 2,			/* 读取PID参数 */
	S_VBUS  = 3,			/* 读取总线电压 */
	S_CPHA  = 5,			/* 读取相电流 */
	S_ENCL  = 7,			/* 读取经过线性化校准后的编码器值 */
	S_TPOS  = 8,			/* 读取电机目标位置角度 */
	S_VEL   = 9,			/* 读取电机实时转速 */
	S_CPOS  = 10,			/* 读取电机实时位置角度 */
	S_PERR  = 11,			/* 读取电机位置误差角度 */
	S_FLAG  = 13,			/* 读取使能/到位/堵转状态标志位 */
	S_Conf  = 14,			/* 读取驱动参数 */
	S_State = 15,			/* 读取系统状态参数 */
	S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
}SysParams_t;


/**********************************************************
*** 注意：每个函数的参数的具体说明，请查阅对应函数的注释说明
**********************************************************/
void ALL_INIT(void);//初始化的主要代码
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr); // 将当前位置清零
void Emm_V5_Reset_Clog_Pro(uint8_t addr); // 解除堵转保护
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s); // 读取参数
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode); // 发送命令修改开环/闭环控制模式
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF); // 电机使能控制
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF); // 速度模式控制
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF); // 位置模式控制
void Emm_V5_Stop_Now(uint8_t addr, bool snF); // 让电机立即停止运动
void Emm_V5_Synchronous_motion(uint8_t addr); // 触发多机同步开始运动
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF); // 设置挡圈回零的零点位置
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // 修改回零参数
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF); // 发送命令触发回零
void Emm_V5_Origin_Interrupt(uint8_t addr); // 强制中断并退出回零
void STEP_Turn_1(void);//小车转弯
void STEP_Turn_2(void);//小车在第二个转弯处的转弯
void STEP_Turn_3(void);//小车在第二行末尾的转弯
void STEP_Turn_4(void);//小车在第三行开头的转弯
void STEP_Turn_5(void);//小车在第三行末尾的转弯
void STEP_Turn_6(void);//小车在第四行开头的转弯
void vel_move(uint16_t distance,uint8_t s1,uint8_t s2,uint8_t s3,uint8_t s4);//速度环控制前进
void bujin_zheng(float quanshu);//步进正转，向上
void bujin_fan(float quanshu);//步进反转，向下
void HUIDU_read(void);//读取灰度传感器的值
void jiaoshui(void);//浇水灌溉模块
void move_forward(uint32_t distance,uint32_t s1,uint32_t s2,uint32_t s3,uint32_t s4);//小车前进固定距离的函数
void line1_left(void);//第一行左侧盆栽处机械臂位置
void line1_right(void);//第一行右侧盆栽处机械臂位置
void line_fold(void);//机械臂弯曲的函数
void line1_middle(void);//使得机械臂处于中间的位置
void line2_left(void);//第二行左侧盆栽机械臂位置
void line2_right(void);//第二行右侧盆栽机械臂位置
void special_delay(void);//小车行进期间的特殊延时
void line1_act(void);//第一行小车对应的函数
void line2_act(void);//第二行小车对应函数
void line3_act(void);//第三行小车对应函数
void line4_act(void);//第四行小车对应函数
void line1_config(uint8_t li);//第一行旱情判断对应的浇水和播报函数
void line2_config(uint8_t li);//第二行旱情对应的浇水和播报函数
void line3_config(uint8_t li);//第三行旱情对应的浇水和播报函数
void line4_config(uint8_t i);//第四行对应的浇水和播报函数
void line1_choose(uint8_t ID);//第一行对应位置的综合函数
void line2_choose(uint8_t ID);//第二行对应位置的综合函数
void line3_choose(uint8_t ID);//第三行对应位置的综合函数
void line4_choose();//第四行对应位置的综合函数
void STEP_Stop(void);//用于使得小车立即停止
float change_angle(float angle);//角度转化
float change_angle_1(float angle);//角度转化

void line3_left(void);//第三行左侧的机械臂方案
void line3_right(void);//第三行右侧的机械臂方案
void line4_left(void);//第四行左侧的机械臂方案
void line4_right(void);//第四行右侧的机械臂方案
void caluclate3l(uint16_t x,uint16_t y);//第三行左侧，左侧以小车前进方向为正方向来判断
void caluclate3r(uint16_t x,uint16_t y);//第三行右侧
void caluclate4l(uint16_t x,uint16_t y);//第四行左侧
void caluclate4r(uint16_t x,uint16_t y);//第四行右侧
void modify_lines(void);//根据白色十字纠正的函数
void servo_edit(uint16_t x1,uint16_t x2,uint16_t x3);//舵机位置控制
uint8_t turnangle_set(uint8_t angle);//提前设定的角度传参函数
void line_watch(void);//小车处于纠正模式时候的机械臂位置
float float_abs(float k);//用于取得浮点数的绝对值
void watch(uint8_t address,uint8_t direction,uint8_t acc);//用于监管前进函数防止电机犯病的函数
void watch_together(void);//用于监管看护一齐运行的函数  
void watch_stop(uint8_t ID);//监管小车停止的函数
void watch_vel(uint8_t ID,uint8_t dir);//监督小车以速度模式运行的函数
void left_modify(void);//小车向左微调
void right_modify(void);//小车向右微调
void small_modify(uint8_t mode);//微小调整来使得车方向为正向
int change_angle_modify(int z);//用于调整的特殊角度变换函数

#endif
