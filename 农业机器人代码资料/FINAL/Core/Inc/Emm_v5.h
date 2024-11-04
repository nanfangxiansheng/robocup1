#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "usart.h"
#include "stdio.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"
/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

#define		ABS(x)		((x) > 0 ? (x) : -(x)) 


typedef enum {
	S_VER   = 0,			/* ��ȡ�̼��汾�Ͷ�Ӧ��Ӳ���汾 */
	S_RL    = 1,			/* ��ȡ��ȡ���������� */
	S_PID   = 2,			/* ��ȡPID���� */
	S_VBUS  = 3,			/* ��ȡ���ߵ�ѹ */
	S_CPHA  = 5,			/* ��ȡ����� */
	S_ENCL  = 7,			/* ��ȡ�������Ի�У׼��ı�����ֵ */
	S_TPOS  = 8,			/* ��ȡ���Ŀ��λ�ýǶ� */
	S_VEL   = 9,			/* ��ȡ���ʵʱת�� */
	S_CPOS  = 10,			/* ��ȡ���ʵʱλ�ýǶ� */
	S_PERR  = 11,			/* ��ȡ���λ�����Ƕ� */
	S_FLAG  = 13,			/* ��ȡʹ��/��λ/��ת״̬��־λ */
	S_Conf  = 14,			/* ��ȡ�������� */
	S_State = 15,			/* ��ȡϵͳ״̬���� */
	S_ORG   = 16,     /* ��ȡ���ڻ���/����ʧ��״̬��־λ */
}SysParams_t;


/**********************************************************
*** ע�⣺ÿ�������Ĳ����ľ���˵��������Ķ�Ӧ������ע��˵��
**********************************************************/
void ALL_INIT(void);//��ʼ������Ҫ����
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr); // ����ǰλ������
void Emm_V5_Reset_Clog_Pro(uint8_t addr); // �����ת����
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s); // ��ȡ����
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode); // ���������޸Ŀ���/�ջ�����ģʽ
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF); // ���ʹ�ܿ���
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF); // �ٶ�ģʽ����
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF); // λ��ģʽ����
void Emm_V5_Stop_Now(uint8_t addr, bool snF); // �õ������ֹͣ�˶�
void Emm_V5_Synchronous_motion(uint8_t addr); // �������ͬ����ʼ�˶�
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF); // ���õ�Ȧ��������λ��
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // �޸Ļ������
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF); // �������������
void Emm_V5_Origin_Interrupt(uint8_t addr); // ǿ���жϲ��˳�����
void STEP_Turn_1(void);//С��ת��
void STEP_Turn_2(void);//С���ڵڶ���ת�䴦��ת��
void STEP_Turn_3(void);//С���ڵڶ���ĩβ��ת��
void STEP_Turn_4(void);//С���ڵ����п�ͷ��ת��
void STEP_Turn_5(void);//С���ڵ�����ĩβ��ת��
void STEP_Turn_6(void);//С���ڵ����п�ͷ��ת��
void vel_move(uint16_t distance,uint8_t s1,uint8_t s2,uint8_t s3,uint8_t s4);//�ٶȻ�����ǰ��
void bujin_zheng(float quanshu);//������ת������
void bujin_fan(float quanshu);//������ת������
void HUIDU_read(void);//��ȡ�Ҷȴ�������ֵ
void jiaoshui(void);//��ˮ���ģ��
void move_forward(uint32_t distance,uint32_t s1,uint32_t s2,uint32_t s3,uint32_t s4);//С��ǰ���̶�����ĺ���
void line1_left(void);//��һ��������Դ���е��λ��
void line1_right(void);//��һ���Ҳ����Դ���е��λ��
void line_fold(void);//��е�������ĺ���
void line1_middle(void);//ʹ�û�е�۴����м��λ��
void line2_left(void);//�ڶ���������Ի�е��λ��
void line2_right(void);//�ڶ����Ҳ����Ի�е��λ��
void special_delay(void);//С���н��ڼ��������ʱ
void line1_act(void);//��һ��С����Ӧ�ĺ���
void line2_act(void);//�ڶ���С����Ӧ����
void line3_act(void);//������С����Ӧ����
void line4_act(void);//������С����Ӧ����
void line1_config(uint8_t li);//��һ�к����ж϶�Ӧ�Ľ�ˮ�Ͳ�������
void line2_config(uint8_t li);//�ڶ��к����Ӧ�Ľ�ˮ�Ͳ�������
void line3_config(uint8_t li);//�����к����Ӧ�Ľ�ˮ�Ͳ�������
void line4_config(uint8_t i);//�����ж�Ӧ�Ľ�ˮ�Ͳ�������
void line1_choose(uint8_t ID);//��һ�ж�Ӧλ�õ��ۺϺ���
void line2_choose(uint8_t ID);//�ڶ��ж�Ӧλ�õ��ۺϺ���
void line3_choose(uint8_t ID);//�����ж�Ӧλ�õ��ۺϺ���
void line4_choose();//�����ж�Ӧλ�õ��ۺϺ���
void STEP_Stop(void);//����ʹ��С������ֹͣ
float change_angle(float angle);//�Ƕ�ת��
float change_angle_1(float angle);//�Ƕ�ת��

void line3_left(void);//���������Ļ�е�۷���
void line3_right(void);//�������Ҳ�Ļ�е�۷���
void line4_left(void);//���������Ļ�е�۷���
void line4_right(void);//�������Ҳ�Ļ�е�۷���
void caluclate3l(uint16_t x,uint16_t y);//��������࣬�����С��ǰ������Ϊ���������ж�
void caluclate3r(uint16_t x,uint16_t y);//�������Ҳ�
void caluclate4l(uint16_t x,uint16_t y);//���������
void caluclate4r(uint16_t x,uint16_t y);//�������Ҳ�
void modify_lines(void);//���ݰ�ɫʮ�־����ĺ���
void servo_edit(uint16_t x1,uint16_t x2,uint16_t x3);//���λ�ÿ���
uint8_t turnangle_set(uint8_t angle);//��ǰ�趨�ĽǶȴ��κ���
void line_watch(void);//С�����ھ���ģʽʱ��Ļ�е��λ��
float float_abs(float k);//����ȡ�ø������ľ���ֵ
void watch(uint8_t address,uint8_t direction,uint8_t acc);//���ڼ��ǰ��������ֹ��������ĺ���
void watch_together(void);//���ڼ�ܿ���һ�����еĺ���  
void watch_stop(uint8_t ID);//���С��ֹͣ�ĺ���
void watch_vel(uint8_t ID,uint8_t dir);//�ලС�����ٶ�ģʽ���еĺ���
void left_modify(void);//С������΢��
void right_modify(void);//С������΢��
void small_modify(uint8_t mode);//΢С������ʹ�ó�����Ϊ����
int change_angle_modify(int z);//���ڵ���������Ƕȱ任����

#endif
