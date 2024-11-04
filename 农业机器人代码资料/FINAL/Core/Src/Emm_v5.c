#include "Emm_V5.h"
#include "gpio.h"
#include "tim.h"
#include "LobotServoController.h"
#include "music.h"
#include "JY901S.h"
#include "tjc_usart_hmi.h"
#include "stdlib.h"
#include "NRF24L01.h"
#include "math.h"//���ڼ������Ǻ���
#include "usart.h"

/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/
extern float BUJIN_CIRCLE;//�������Ȧ��
extern uint8_t line1[6];//ÿ�еĺ�����Ϣ��ʾ
extern uint8_t line2[6];
extern uint8_t line3[6];
extern uint16_t BUJIN_1;//��������
float gyro_data;
float gyroz,gyroz_last;
float anglez;
float gyro_datax;
float gyrox,gyrox_last;
float anglex;
float gyro_datay;
float gyroy,gyroy_last,angley;
extern struct STime		stcTime;
extern struct SAcc 		stcAcc;
extern struct SGyro 		stcGyro;
extern struct SAngle 	stcAngle;
extern struct SMag 		stcMag;
extern struct SDStatus stcDStatus;
extern struct SPress 	stcPress;
extern struct SLonLat 	stcLonLat;
extern struct SGPSV 		stcGPSV;
extern struct SQ       stcQ;
int yroll;//y������
int zyaw;//z��ƫ����
int xpitch;//x��ƫ��
uint8_t turnflag1=0;//����a ����ת���־
uint8_t turnflag2=0;//���ǵڶ���ת�䴦�ı�־
uint8_t turnflag3=0;//�����ڶ�������ʱ��Ĺ����־
uint8_t turnflag4=0;//������������ʼʱ��Ĺ����־
uint8_t turnflag5=0;//�����ڵ���������ʱ��Ĺ����־
uint8_t turnflag6=0;//�����ڵ�������ʼʱ��Ĺ����־
uint8_t stopflag1=0;//�����ڵ�һ��ĩβ����ʱ��Ľ�����־
uint8_t stopflag2=0;//�����ڵڶ��п�ͷ����ʱ��Ľ�����־
uint8_t stopflag3=0;//�����ڵڶ���ĩβ����ʱ��Ľ�����־
uint8_t stopflag4=0;//�����ڵ����п�ͷ����ʱ��Ľ�����־
uint8_t stopflag5=0;//�����ڵ�����ĩβ����ʱ��Ľ�����־
uint8_t stopflag6=0;//�����ڵ����п�ͷ����ʱ��Ľ�����־
extern int k;//���ǵڶ���ת��ǰ��zyaw����ֵ
int x1;//�ո��ߵ��ڶ��е�ʱ��xpitch�ļ���ֵ
int x2;//�����ϵ�ʱ��xpitch�ļ���ֵ
int y1;//�ڵ�һ�������ϵ�ʱ���yroll�ļ���ֵ
int y2;//�ڵڶ������ϵ�ʱ���yroll�ļ���ֵ
int z1;//�ڵ�һ��ĩ��ת��ǰzyaw�ļ���ֵ
int z3;//���ǵڶ���ĩβʱ��zyaw�ļ���ֵ
int z4;//���ǵ����п�ͷzyaw�ļ���ֵ
int z5;//���ǵ�����ĩβʱ��zyaw�ļ���ֵ
int z6;//���ǵ����п�ͷʱ��ת��ǰzyaw�ļ���ֵ
uint8_t on_poflag1=0;//��һ������ʱ��ı�־λ
uint8_t down_poflag1=0;//��һ������ʱ��ı�־λ
uint8_t on_poflag2=0;//�ڶ�������ʱ��ı�־λ
uint8_t down_poflag2=0;//�ڶ�������ʱ��ı�־λ
uint8_t xgewei=0;
uint8_t xshibai=0;
uint8_t ygewei=0;
uint8_t yshibai=0;
uint8_t color=0;//�����ڵ����е�ʶ���н�����ɫ��Ϣ
uint8_t modifyflag=0;//����a����c���Ľ�������
uint8_t modifyflag1=0;//����b����d���Ľ�������,ʵ��û���õ�
extern uint8_t Rxdata;
extern uint32_t s1;//��������
extern uint32_t s2;
extern uint32_t s3;
extern uint32_t s4;
uint16_t xl,yl,xr,yr=0;
uint32_t poi=0;//����������ʱ��ĵ�ƽ����
uint32_t poj=0;//�����ڵڶ������µ�ʱ��ĵ�ƽ����
uint8_t moveflag=0;//����С���н�ʱ��ı�־λ
uint32_t movei=0;//����С���н�ʱ��ļ���
uint16_t xc=0;//xc��ʶ���ɫʮ�ֵ�ʱ�򷵻صĺ�����
uint8_t yc=0;//yc��ʶ���ɫʮ�ֵ�ʱ�򷵻ص�������,ʵ���ϲ���
uint32_t distance=0;//ʶ����ֺͻ�е�����ĵľ���
int32_t dydistance=0;//��ʾʶ���ɫʮ�ֺ���������ƶ�����ľ���
uint8_t spy_po_flag=0;//�����м�⵽�º�ı�־λ����������1
uint32_t roll_1=0;//������Щ������С��ǰ��ʱ��������ȫ�ֱ���,������ǰ�ֵ�
uint32_t roll_2=0;//������ǰ�ֵ�
uint32_t roll_3=0;//�����Һ��ֵ�
uint32_t roll_4=0;//��������ֵ�
uint32_t rolls1=0;//�����ڽ���������
uint32_t rolls2=0;
uint32_t rolls3=0;
uint32_t rolls4=0;
uint8_t spy_down_po_flag=0;//��ʾ������֮����������ǰ����ʽ�ĺ���
uint8_t judge_flag=0;//��ʾ�жϻض��Ĳ����������
uint32_t collect[4]={0,0,0,0};

void ALL_INIT(void){
	HAL_UART_Receive_IT(&huart1,RxBuff,1);//����k210���жϣ�baud=115200
	HAL_UART_Receive_IT(&huart4,RxBuff_1,1);//����ZDT����������жϣ�baud=115200
	HAL_UART_Receive_IT(&huart3,&Rxdata,1);//����JY901S���ڽ��յ��ж�
	JY901S_init();//��ʼ��������
	HAL_TIM_Base_Start_IT(&htim4);//������������ж�
	HAL_TIM_Base_Start_IT(&htim6);//����JY901S�Ķ�ʱ�����ж�
	HAL_TIM_Base_Start_IT(&htim2);//�����ܵĴ����ֵ��ж�
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//TIM_C1->���� 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,7);//���ý�ˮ��ˮ���٣�TIM_C4->��ˮ	

}
/**
  * @brief    ����ǰλ������
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x0A;                       // ������
  cmd[2] =  0x6D;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    �����ת����
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x0E;                       // ������
  cmd[2] =  0x52;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    ��ȡϵͳ����
  * @param    addr  �������ַ
  * @param    s     ��ϵͳ��������
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[i] = addr; ++i;                   // ��ַ

  switch(s)                             // ������
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;//��ȡʵʱλ��
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, i);
}

/**
  * @brief    �޸Ŀ���/�ջ�����ģʽ
  * @param    addr     �������ַ
  * @param    svF      ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  * @param    ctrl_mode������ģʽ����Ӧ��Ļ�ϵ�P_Pul�˵�����0�ǹر������������ţ�1�ǿ���ģʽ��2�Ǳջ�ģʽ��3����En�˿ڸ���Ϊ��Ȧ��λ�����������ţ�Dir�˿ڸ���Ϊ��λ����ߵ�ƽ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x46;                       // ������
  cmd[2] =  0x69;                       // ������
  cmd[3] =  svF;                        // �Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  cmd[4] =  ctrl_mode;                  // ����ģʽ����Ӧ��Ļ�ϵ�P_Pul�˵�����0�ǹر������������ţ�1�ǿ���ģʽ��2�Ǳջ�ģʽ��3����En�˿ڸ���Ϊ��Ȧ��λ�����������ţ�Dir�˿ڸ���Ϊ��λ����ߵ�ƽ����
  cmd[5] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 6);
}

/**
  * @brief    ʹ���źſ���
  * @param    addr  �������ַ
  * @param    state ��ʹ��״̬     ��trueΪʹ�ܵ����falseΪ�رյ��
  * @param    snF   �����ͬ����־ ��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xF3;                       // ������
  cmd[2] =  0xAB;                       // ������
  cmd[3] =  (uint8_t)state;             // ʹ��״̬
  cmd[4] =  snF;                        // ���ͬ���˶���־
  cmd[5] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 6);
}

/**
  * @brief    �ٶ�ģʽ
  * @param    addr�������ַ
  * @param    dir ������       ��0ΪCW������ֵΪCCW
  * @param    vel ���ٶ�       ����Χ0 - 5000RPM
  * @param    acc �����ٶ�     ����Χ0 - 255��ע�⣺0��ֱ������
  * @param    snF �����ͬ����־��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xF6;                       // ������
  cmd[2] =  dir;                        // ����
  cmd[3] =  (uint8_t)(vel >> 8);        // �ٶ�(RPM)��8λ�ֽ�
  cmd[4] =  (uint8_t)(vel >> 0);        // �ٶ�(RPM)��8λ�ֽ�
  cmd[5] =  acc;                        // ���ٶȣ�ע�⣺0��ֱ������
  cmd[6] =  snF;                        // ���ͬ���˶���־
  cmd[7] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 8);
}

/**
  * @brief    λ��ģʽ
  * @param    addr�������ַ
  * @param    dir ������        ��0ΪCW������ֵΪCCW
  * @param    vel ���ٶ�(RPM)   ����Χ0 - 5000RPM
  * @param    acc �����ٶ�      ����Χ0 - 255��ע�⣺0��ֱ������
  * @param    clk ��������      ����Χ0- (2^32 - 1)��
  * @param    raF ����λ/���Ա�־��falseΪ����˶���trueΪ����ֵ�˶�
  * @param    snF �����ͬ����־ ��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // װ������
  cmd[0]  =  addr;                      // ��ַ
  cmd[1]  =  0xFD;                      // ������
  cmd[2]  =  dir;                       // ����
  cmd[3]  =  (uint8_t)(vel >> 8);       // �ٶ�(RPM)��8λ�ֽ�
  cmd[4]  =  (uint8_t)(vel >> 0);       // �ٶ�(RPM)��8λ�ֽ� 
  cmd[5]  =  acc;                       // ���ٶȣ�ע�⣺0��ֱ������
  cmd[6]  =  (uint8_t)(clk >> 24);      // ������(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // ������(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // ������(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // ������(bit0  - bit7 )
  cmd[10] =  raF;                       // ��λ/���Ա�־��falseΪ����˶���trueΪ����ֵ�˶�
  cmd[11] =  snF;                       // ���ͬ���˶���־��falseΪ�����ã�trueΪ����
  cmd[12] =  0x6B;                      // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 13);
}

/**
  * @brief    ����ֹͣ�����п���ģʽ��ͨ�ã�
  * @param    addr  �������ַ
  * @param    snF   �����ͬ����־��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xFE;                       // ������
  cmd[2] =  0x98;                       // ������
  cmd[3] =  snF;                        // ���ͬ���˶���־
  cmd[4] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    ���ͬ���˶�
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xFF;                       // ������
  cmd[2] =  0x66;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    ���õ�Ȧ��������λ��
  * @param    addr  �������ַ
  * @param    svF   ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x93;                       // ������
  cmd[2] =  0x88;                       // ������
  cmd[3] =  svF;                        // �Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  cmd[4] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    �޸Ļ������
  * @param    addr  �������ַ
  * @param    svF   ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  * @param    o_mode ������ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  * @param    o_dir  �����㷽��0ΪCW������ֵΪCCW
  * @param    o_vel  �������ٶȣ���λ��RPM��ת/���ӣ�
  * @param    o_tm   �����㳬ʱʱ�䣬��λ������
  * @param    sl_vel ������λ��ײ������ת�٣���λ��RPM��ת/���ӣ�
  * @param    sl_ma  ������λ��ײ�������������λ��Ma��������
  * @param    sl_ms  ������λ��ײ������ʱ�䣬��λ��Ms�����룩
  * @param    potF   ���ϵ��Զ��������㣬falseΪ��ʹ�ܣ�trueΪʹ��
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32]={0};
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x4C;                       // ������
  cmd[2] =  0xAE;                       // ������
  cmd[3] =  svF;                        // �Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  cmd[4] =  o_mode;                     // ����ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  cmd[5] =  o_dir;                      // ���㷽��
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // �����ٶ�(RPM)��8λ�ֽ�
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // �����ٶ�(RPM)��8λ�ֽ� 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // ���㳬ʱʱ��(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // ���㳬ʱʱ��(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // ���㳬ʱʱ��(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // ���㳬ʱʱ��(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // ����λ��ײ������ת��(RPM)��8λ�ֽ�
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // ����λ��ײ������ת��(RPM)��8λ�ֽ� 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // ����λ��ײ���������(Ma)��8λ�ֽ�
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // ����λ��ײ���������(Ma)��8λ�ֽ� 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // ����λ��ײ������ʱ��(Ms)��8λ�ֽ�
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // ����λ��ײ������ʱ��(Ms)��8λ�ֽ�
  cmd[18] =  potF;                      // �ϵ��Զ��������㣬falseΪ��ʹ�ܣ�trueΪʹ��
  cmd[19] =  0x6B;                      // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 20);
}

/**
  * @brief    ��������
  * @param    addr   �������ַ
  * @param    o_mode ������ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  * @param    snF   �����ͬ����־��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x9A;                       // ������
  cmd[2] =  o_mode;                     // ����ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  cmd[3] =  snF;                        // ���ͬ���˶���־��falseΪ�����ã�trueΪ����
  cmd[4] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    ǿ���жϲ��˳�����
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x9C;                       // ������
  cmd[2] =  0x48;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  usart_SendCmd(cmd, 4);
}
/**
  * @brief    step motor init
  * @param    NULL
  * @retval   NULL
  */


uint8_t L1,L2,M,R1,R2;
void step_clear(){//����ջ��������ʾ���ĺ�����ʵ��û���õ�
	delay_ms(10);
	Emm_V5_Reset_CurPos_To_Zero(1);
	delay_ms(10);
	Emm_V5_Reset_CurPos_To_Zero(2);
	delay_ms(10);
	Emm_V5_Reset_CurPos_To_Zero(3);
	delay_ms(10);
	Emm_V5_Reset_CurPos_To_Zero(4);
	delay_ms(10);
}
void HUIDU_read(void){//��ȡ�Ҷȴ�������Ϣ
	L2=HAL_GPIO_ReadPin(HUIDU_1_GPIO_Port,HUIDU_1_Pin);
	L1=HAL_GPIO_ReadPin(HUIDU_2_GPIO_Port,HUIDU_2_Pin);
	M=HAL_GPIO_ReadPin(HUIDU_3_GPIO_Port,HUIDU_3_Pin);
	R1=HAL_GPIO_ReadPin(HUIDU_4_GPIO_Port,HUIDU_4_Pin);
	R2=HAL_GPIO_ReadPin(HUIDU_5_GPIO_Port,HUIDU_5_Pin);
}
void watch_vel(uint8_t ID,uint8_t direction){//Ĭ�ϰ����ٶ�40�����ٶ�50���˶�,�ල�ٶ�ģʽ�ĺ���
	Emm_V5_Vel_Control(ID,direction,30,50,1);
	delay_ms(30);
	uint8_t errcount=0;
	uint8_t errorflag=0;
	while (gsize>=4){
		if(g(0)!=0x01||g(3)!=0x6B)
		  {
			  gdelete(1);
		  }
		else
		  {
			  break;
		  }
	}
  if(g(0)==0x01&&g(3)==0x6B){
		if(errorflag==0){		
			if(g(1)!=0xF6||g(2)!=0x02){//��ʾû����ȷ����
			watch_vel(ID,direction);
			gdelete(4);
			errcount+=1;
			if(errcount>=5){
				errorflag=1;
			}
		}
		}

	}
	gdelete(4);
}
void STEP_Turn_1(void){	//�ڵ�һ��ת�䴦�Ķ�������
	delay_ms(30);
	//��16ϸ�ֵ�״̬�£�3200ת��һȦ��32000��ʮȦ
  watch_vel(1,0);
	watch_vel(2,0);
	watch_vel(3,0);
	watch_vel(4,0);
	watch_together();//��ʾ�ලһ���˶�
	turnflag1=1;
}
void STEP_Turn_2(void){//�ڵڶ��п�ʼ���Ĺ���
	delay_ms(30);
	//��16ϸ�ֵ�״̬�£�3200ת��һȦ��32000��ʮȦ
  watch_vel(1,0);
	watch_vel(2,0);
	watch_vel(3,0);
	watch_vel(4,0);
	watch_together();//��ʾ�ලһ���˶�
	turnflag2=1;
}
void STEP_Turn_3(void){//�ڵڶ��н�����ʱ��ʼ����
	delay_ms(30);
	watch_vel(1,1);
	watch_vel(2,1);
	watch_vel(3,1);
	watch_vel(4,1);
	watch_together();//��ʾ�ලһ���˶�
	turnflag3=1;
}
void STEP_Turn_4(void){//�ڵ����п�ʼ��ʱ��ʼ����
	delay_ms(30);
	watch_vel(1,1);
	watch_vel(2,1);
	watch_vel(3,1);
	watch_vel(4,1);
	watch_together();//��ʾ�ලһ���˶�
	turnflag4=1;
}
void STEP_Turn_5(void){//�ڵ����н�����ʱ��ʼ����
	delay_ms(30);
	watch_vel(1,0);
	watch_vel(2,0);
	watch_vel(3,0);
	watch_vel(4,0);
	watch_together();//��ʾ�ලһ���˶�
	turnflag5=1;
}
void STEP_Turn_6(void){//�ڵ����п�ʼ��ʱ��ʼ����
	delay_ms(30);
	watch_vel(1,0);
	watch_vel(2,0);
	watch_vel(3,0);
	watch_vel(4,0);
	watch_together();//��ʾ�ලһ���˶�
	turnflag6=1;
}
void watch_stop(uint8_t ID){//���С��ֹͣ�ĺ���
	Emm_V5_Stop_Now(ID,1);
	delay_ms(5);
	uint8_t errorcount=0;
	uint8_t errorflag=0;
	while (gsize>=4){
		if(g(0)!=0x01||g(3)!=0x6B)
		  {
			  gdelete(1);
		  }
		else
		  {
			  break;
		  }
	}
  if(g(0)==0x01&&g(3)==0x6B){
		if(errorflag==0){		
			if(g(1)!=0xFE||g(2)!=0x02){//��ʾû����ȷ����
			watch_stop(ID);
			gdelete(4);
			errorcount+=1;
				if(errorcount>=5){
					errorflag=1;
				}
		}
		}
	}
	gdelete(4);
	
}
void STEP_Stop(void){
	delay_ms(5);
	watch_stop(1);
	watch_stop(2);
	watch_stop(3);
	watch_stop(4);
	watch_together();

}
/**
@Brief ���ٶȻ����Ƶ�С���˶���ʵ���ϲ�����
@Param distance
@Retval NULL
*/
void vel_move(uint16_t distance,uint8_t s1,uint8_t s2,uint8_t s3,uint8_t s4){
	uint32_t a=0;//��ָ�Ƕ�
	uint32_t b=0;
	uint32_t c=0;
	uint32_t d=0;
	uint32_t e=0;
	uint8_t step_flag=1;
	delay_ms(50);
  Emm_V5_Vel_Control(1,1,60+s1,80,true);
	delay_ms(50);
	Emm_V5_Vel_Control(2,0,60+s2,80,true);
	delay_ms(50);
	Emm_V5_Vel_Control(3,1,60+s3,80,true);
	delay_ms(50);
  Emm_V5_Vel_Control(4,0,60+s4,80,true);
	delay_ms(50);
  Emm_V5_Synchronous_motion(0);//����ͬʱ�˶�
	delay_ms(50);
	initRingBuff1();
	while(step_flag==1){//26.37cm�ǳ��ֵ��ܳ�
		Emm_V5_Read_Sys_Params(1,S_CPOS);
		if(g(0)==0x01&&g(1)==0x36&&g(7)==0x6B){
			a=(change(g(6))+256*change(g(5))+65536*change(g(4))+16776960*change(g(3)))*360/65536;
			gdelete(8);
		}	
		delay_ms(5);
		Emm_V5_Read_Sys_Params(2,S_CPOS);		
		if(g(0)==0x01&&g(1)==0x36&&g(7)==0x6B){
			b=(change(g(6))+256*change(g(5))+65536*change(g(4))+16776960*change(g(3)))*360/65536;
			gdelete(8);
		}	
		delay_ms(5);
		Emm_V5_Read_Sys_Params(3,S_CPOS);
		if(g(0)==0x01&&g(1)==0x36&&g(7)==0x6B){
			c=(change(g(6))+256*change(g(5))+65536*change(g(4))+16776960*change(g(3)))*360/65536;
			gdelete(8);
		}	
		delay_ms(5);
		Emm_V5_Read_Sys_Params(4,S_CPOS);
		if(g(0)==0x01&&g(1)==0x36&&g(7)==0x6B){
			d=(change(g(6))+256*change(g(5))+65536*change(g(4))+16776960*change(g(3)))*360/65536;
			gdelete(8);
		}	
		e=(a+b+c+d)/4;
		if(e>(distance/(3.14*9.5))*360){
			STEP_Stop();	
			step_clear();//�Ƕ�����
			step_flag=0;
			break;
		}
	}
}
void bujin_zheng(float quanshu){//���Ƶ�ƽ������ת,�����˶�
	HAL_GPIO_WritePin(bujin_CON_GPIO_Port,bujin_CON_Pin,GPIO_PIN_SET);
	BUJIN_1=0;
	BUJIN_CIRCLE=quanshu;
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,5);
}
void bujin_fan(float quanshu){//���Ƶ�ƽ������ת�������˶�
	HAL_GPIO_WritePin(bujin_CON_GPIO_Port,bujin_CON_Pin,GPIO_PIN_RESET);
	BUJIN_1=0;
	BUJIN_CIRCLE=quanshu;
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,5);
}
/**
@Brief ��һ�й�ȱ�ʱ�ĺ���
@Param ��һ�еĻ�����
@Retval NULL
*/
void line1_choose(uint8_t ID){
	line1_middle();
	line1_left();
	line1_config(ID);
	line1_middle();
	line1_right();
	line1_config(ID);
	line1_middle();
	line_fold();
}
/**
@Brief �ڶ��й�ȱ�ʱ�ĺ���
@Param �ڶ��еĻ�����
@Retval NULL
*/
void line2_choose(uint8_t ID){
	line1_middle();
	line2_left();
	line2_config(ID);
	line1_middle();
	line2_right();
	line2_config(ID);
	line1_middle();
	line_fold();
}
/**
@Brief �����й�ȱ�ʱ�ĺ���
@Param �����еĻ�����
@Retval NULL
*/
void line3_choose(uint8_t ID){
	line1_middle();
	line3_left();
	line3_config(ID);
	line1_middle();
	line3_right();
	line3_config(ID);
	line1_middle();
	line_fold();
}
/**
@Brief �����й�ȱ�ʱ�ĺ���
@Param NULL
@Retval NULL
*/
void line4_choose(){
	line1_middle();
	line4_left();
	line1_middle();
	line4_right();
	line1_middle();
	line_fold();
}
//�������ɺ�����������
void special_delay(void){//����С���н�ʱ�����ͨ��ʱ����
	moveflag=1;
	movei=0;
	while(moveflag==1){
		delay_ms(10);
	}
}
void special_delay_2(void){//����k210�ȵ�������ʱ����
	uint8_t i;
	for(i=0;i<20;i++){
		delay_ms(200);
	
	}
}
//��С�����������ģ�һ���ǻ�е�����ģ�����һ���ǳ����ģ������漰��ˮ�����������û�е�����ģ��漰��ת�������ó�����
void line1_act(void){//С���ڵ�һ�е���Ӧ���ж�
	modifyflag=1;//�����Ծ�������
	move_forward(55,s1,s2,s3,s4);
	special_delay();//���ǿ�ʱ�����ʱ��������ʱ��Ϊ�˷�ֹ���������⣬�������ͣ��	
	line1_choose(0);//����ִ�н�ˮ�Ͳ����ĺ�����0�����һ��λ��
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	line1_choose(1);
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	line1_choose(2);
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	line1_choose(3);
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	line1_choose(4);
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	line1_choose(5);
	move_forward(70,s1,s2,s3,s4);//�ߵ���ɫʮ�ִ���һ����˵�߳�ȥ�ľ���Ӧ������һ���߽����ľ���+20cm
	special_delay();	
	modifyflag=0;//һ��Ҫǰ���Ӧ

	small_modify(1);//ת��ǰ����
//��ʼת��
	z1=abs(zyaw);
	STEP_Turn_1();//��һ��ת90�ȣ�ʵ������΢�������ֵ
	while(stopflag1==0){
		delay_ms(5);
	}
	STEP_Stop();
	move_forward(70,0,0,0,0);//����60cm
	special_delay();
	modify_lines();//Ȼ��ʼ�۲�Ͱ�ɫʮ�ּ�ľ���
	if(distance<=10){//������
		distance=40;
	}
	move_forward(distance,0,0,0,0);//�ߵ���ɫʮ�ִ�
	special_delay();
	STEP_Turn_2();//�ߵ��ڶ�����ʼλ�ô����ڶ���ת90��
	while(stopflag2==0){
		delay_ms(5);
	}
	STEP_Stop();
}
void line2_act(void){
		//��ʼ�ڶ���
	modifyflag1=1;//�����Ƿ����õ�������,modifyflag1������b����d��
	move_forward(50,s1,s2,s3,s4);
	special_delay();
	line2_choose(0);//�ڶ��еĽ�ˮ��Ⱥ���
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	line2_choose(1);
  on_poflag1=1;
  down_poflag1=1;
	poi=0;
	y1=abs(yroll);
	move_forward(60,s1,s2,s3,s4);//��������Ϊ�˱�������;�н�ˮ,���ǵڶ������µ�һ��
	special_delay();
	line2_choose(2);
	move_forward(60,s1,s2,s3,s4);//���ǵڶ������������ߵ�һ��
	special_delay();
	line2_choose(3);
	move_forward(60,s1,s2,s3,s4);
	special_delay();//������ʱ����������ʱ��Ϊ�˷�ֹС����������
	line2_choose(4);
	move_forward(60,s1,s2,s3,s4);
	special_delay();//������60cm��������ʱ����
	line2_choose(5);
	move_forward(70,s1,s2,s3,s4);//�߳�70cm
	special_delay();
	modifyflag1=0;	
	z3=zyaw;	//��¼��ǰƫ����
	//��ʼת��
	STEP_Turn_3();	
	while(stopflag3==0){
		delay_ms(5);
	}
	STEP_Stop();
  move_forward(70,0,0,0,0);//���߳�60cm�����ɫʮ�ֺͳ�֮��ľ���
	special_delay();
	modify_lines();
	if(distance<=15){//������
		distance=50;
	}
	move_forward(distance,0,0,0,0);
	special_delay();
	z4=zyaw;//��¼��ǰ��ƫ����
	STEP_Turn_4();
	while(stopflag4==0){
		delay_ms(5);
	}
	STEP_Stop();
	//������ɺ�

}
void line3_act(void){
	modifyflag=1;//modifyflag������a,c������
	poj=0;
	on_poflag1=0;
	down_poflag1=0;
	on_poflag2=1;
	down_poflag2=1;
	move_forward(50,s1,s2,s3,s4);
	special_delay();	
	y1=abs(yroll);//��¼��ǰyroll��ֵ
	line3_choose(0);//�������Զ�ʶ��ĺ������Ѿ�д�÷�װ�ã��Ķ��͸�k210.py����ļ���
	move_forward(60,s1,s2,s3,s4);//Ϊ�˼���������¶�׼���ĺ���
	special_delay();
	line3_choose(1);
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	line3_choose(2);
  move_forward(60,s1,s2,s3,s4);
	special_delay();
	line3_choose(3);
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	line3_choose(4);
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	line3_choose(5);
	move_forward(70,s1,s2,s3,s4);//�߳�65cm
	special_delay();
	modifyflag=0;//��ʱ��modifyflag���͵�0
	//��ʼת��
	z5=zyaw;
	STEP_Turn_5();
	while(stopflag5==0){
		delay_ms(10);
	}
	STEP_Stop();
	//small_modify(3);//�ú������ʺ���̫�࣬���򳵺ܿ�����а·
	move_forward(70,0,0,0,0);//���߳�60cmʶ���ɫʮ��
	special_delay();
	modify_lines();
	if(distance<=10){
		distance=50;
	}
	move_forward(distance,0,0,0,0);
	special_delay();
	z6=zyaw;
	STEP_Turn_6();
	while(stopflag6==0){
		delay_ms(10);
	}
	STEP_Stop();	
}
void line4_act(void){
	//modifyflag1=1;//modifyflag1��Ϊb��,d�����Ƶ�
	move_forward(50,s1,s2,s3,s4);
	special_delay();
	move_forward(20,s1,s2,s3,s4);
	special_delay();	
	line1_middle();
	line4_right();
	line4_config(3);
	line1_middle();
	line_fold();
	//line4_choose();//�����е�����ʶ���Ⱥ�������С�����ڸ������м��ʱ��Ч�����
	move_forward(35,s1,s2,s3,s4);
	special_delay();	
	line1_middle();
	line4_left();
	line4_config(2);
	line1_middle();
	line_fold();
	move_forward(40,s1,s2,s3,s4);
	special_delay();
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	//line4_choose();
	line1_middle();
	line4_right();
	line4_config(1);
	line1_middle();
	line_fold();
	move_forward(20,s1,s2,s3,s4);
	special_delay();
	line1_middle();
	line4_left();
	line4_config(3);
	line1_middle();
	line_fold();
	move_forward(40,s1,s2,s3,s4);
	special_delay();
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	//line4_choose();

	line1_middle();
	line4_right();
	line4_config(2);
	line1_middle();
	line_fold();
	move_forward(20,s1,s2,s3,s4);
	special_delay();
	line1_middle();
	line4_left();
	line4_config(1);
	line1_middle();
	line_fold();
	move_forward(85,s1,s2,s3,s4);
	special_delay();
	modifyflag1=0;

}
float change_angle(float angle){//�ڽ���ڶ���ǰת������
	return (angle<=0)?-angle:180+(180-angle);
}
float change_angle_1(float angle){//�ڽ��������ǰת������
	return (angle<=0)?-angle:-angle;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){//��ʱ���жϵĴ�����
	if(htim==(&htim4)){//���������������
		BUJIN_1+=1;
		if(BUJIN_1>=3200*BUJIN_CIRCLE){	
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
			BUJIN_1=0;
		}

		
	}
  if(htim==(&htim6)){//�������õ�JY901S�ĽǶ���Ϣ��htim6->����õ���������Ϣ
		gyro_data=(float)stcGyro.w[2]/32768*200*180;
		gyroz_last=gyroz;
		gyroz=(gyro_data+0)*10/180;
		gyroz = gyroz * 0.9 + gyroz_last * 0.1;
		anglez+=(float)(gyroz)*0.01;
		if(anglez>180)
		{
			anglez=-180;
		}
			if(anglez<-180)
		{
			anglez=180;
		}
		zyaw=(int)anglez;
		gyro_datay=(float)stcGyro.w[1]/32768*200*180;
		gyroy_last=gyroy;
		gyroy=(gyro_datay+0)*10/180;
		gyroy = gyroy * 0.9 + gyroy_last * 0.1;
		angley+=(float)(gyroy)*0.01;
		if(angley>180)
		{
			angley=-180;
		}
			if(angley<-180)
		{
			angley=180;
		}
		yroll=(int)angley;
		gyro_datax=(float)stcGyro.w[0]/32768*200*180;
		gyrox_last=gyrox;
		gyrox=(gyro_datax+0)*10/180;
		gyrox = gyrox * 0.9 + gyrox_last * 0.1;
		anglex+=(float)(gyrox)*0.01;
		if(anglex>180)
		{
			anglex=-180;
		}
			if(anglex<-180)
		{
			anglex=180;
		}
		xpitch=(int)anglex;
	}
	if(htim==(&htim2)){//�����ж����¹���,50ms�����ж�һ��
		if(turnflag1==1){//���ǳ���һ�к��ת��
			if(float_abs(anglez)>=84){
				turnflag1=0;
				stopflag1=1;
		}
		}
		if(turnflag2==1){//���ǽ���ڶ���ǰ��ת��
			if(change_angle(anglez)>=174){
				turnflag2=0;
				stopflag2=1;
			}
		}
		if(on_poflag1==1){//��һ�����µĺ���
			if(float_abs(angley)>=y1+6){
				bujin_fan(1.5);
				poi=0;
				on_poflag1=0;
				y2=yroll;//��¼��ʱ��yֵ
			}
		}

		if(down_poflag1==1&&on_poflag1==0){//����˫�ر���
			poi++;
			if(poi>=3300){
				bujin_zheng(1.5);
				down_poflag1=0;
				poi=0;
			}
		}
		if(moveflag==1){//���ڸ�С��ǰ���ṩ��ʱ����
			movei++;
			if(movei>=400){
				moveflag=0;
				movei=0;
			}
		}
		if(turnflag3==1){//�ڳ��ڶ��к��ת��
			if(float_abs(anglez)<=95){//��Ҫ������ǰ��
				turnflag3=0;
				stopflag3=1;
			}
		}
		if(turnflag4==1){//�ڽ��������ǰ��ת�䣬ת90��
			if(change_angle_1(anglez)<=3){
				turnflag4=0;
				stopflag4=1;
			}
		}
		if(turnflag5==1){//�����ڳ������к��ת��
			if(float_abs(anglez)>=84){
				turnflag5=0;
				stopflag5=1;
			}
		}
		if(turnflag6==1){//�����ڽ�������ǰ��ת��
			if(change_angle(anglez)>=176){
				turnflag6=0;
				stopflag6=1;
			}
		}
		if(modifyflag==1){//a����c���ĵ�������
				if(anglez>=0.5){
					s1=0;
					s3=0;
					s2=0.5*abs(zyaw);//һ��Ҫ��ȡ����ֵ�ĺ��������ӵĻ��п��ܳ����������
					s4=0.5*abs(zyaw);
				}
				if(anglez<=-0.5){
					s1=0.5*abs(zyaw);
					s3=0.5*abs(zyaw);
					s2=0;
					s4=0;
				}
		}
		if(modifyflag1==1){//b����d���ĵ�������,ע��a����b����С���ĵ���������ͬ��
				if(anglez>=-179.5&&anglez<0){
					s1=0;
					s3=0;
					s2=0.5*abs(180-abs(zyaw));//һ��Ҫ��ȡ����ֵ�ĺ��������ӵĻ��п��ܳ����������
					s4=0.5*abs(180-abs(zyaw));
				}
				if(anglez<=179.5&&anglez>0){
					s1=0.5*abs(180-abs(zyaw));
					s3=0.5*abs(180-abs(zyaw));
					s2=0;
					s4=0;
				}
		}
		if(on_poflag2==1){//���angley���ж�������
			if(float_abs(angley)>=y1+7){
				bujin_fan(1.5);
				on_poflag2=0;
				y2=yroll;//
			}
		}

		if(down_poflag2==1&&on_poflag2==0){//�趨˫�ر���
			poj++;
			if(poj>=4700){
				bujin_zheng(1.5);
				down_poflag2=0;			
			}
		}
	}
}
void jiaoshui(void){//��ˮ��Ⱥ���
	HAL_GPIO_WritePin(pump_1_GPIO_Port,pump_1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(pump_2_GPIO_Port,pump_2_Pin,GPIO_PIN_RESET);
	delay_ms(400);
	HAL_GPIO_WritePin(pump_1_GPIO_Port,pump_1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pump_2_GPIO_Port,pump_2_Pin,GPIO_PIN_RESET);
}
void watch(uint8_t address,uint8_t direction,uint8_t acc){//���ڼ��ǰ��λ�ú�����ֹ��������ĺ���
	uint8_t errorcount=0;
	uint8_t errorflag=0;
	Emm_V5_Pos_Control(address,direction,100,acc,collect[address-1],false,true);
	delay_ms(30);
	while (gsize>=4){
		if(g(0)!=0x01||g(3)!=0x6B)
		  {
			  gdelete(1);
		  }
		else
		  {
			  break;
		  }
	}
  if(g(0)==0x01&&g(3)==0x6B){
		if(errorflag==0){
			if(g(1)!=0xFD||g(2)!=0x02){//��ʾû����ȷ����
			watch(address,direction,acc);
			gdelete(4);
			errorcount+=1;
			if(errorcount>=5){
				errorflag=1;//��ʾ��⵽������������·��
				
			}
		}
	}
}
	gdelete(4);
}
void watch_together(void){//���ڼ�ܿ���һ�����еĺ���  
	Emm_V5_Synchronous_motion(0);//����ͬʱ�˶�
	delay_ms(30);
	uint8_t erroflagt=0;
	uint8_t errorcount=0;
	while (gsize>=4){
		if(g(0)!=0x01||g(3)!=0x6B)
		  {
			  gdelete(1);
		  }

		else
		  {
			  break;
		  }
	}
  if(g(0)==0x01&&g(3)==0x6B){
		if(erroflagt==0){	
			if(g(1)!=0xFF||g(2)!=0x02){//��ʾû����ȷ����
			watch_together();
			gdelete(4);
			errorcount+=1;
		  if(errorcount>=5){
				erroflagt=1;
			}
		}
		}
	}
	gdelete(4);
}
void move_forward(uint32_t distance,uint32_t ks1,uint32_t ks2,uint32_t ks3,uint32_t ks4){//С����ǰ�˶�����cm�ĺ�����	
	roll_1=(distance*3200)/(26);
	roll_2=(distance*3200)/26;
	roll_3=(distance*3200)/25.9;
	roll_4=(distance*3200)/26;
	rolls1=(s1*3200)/(26);
	rolls2=(s2*3200)/(26);
	rolls3=(s3*3200)/(25.9);
	rolls4=(s4*3200)/(26);
	s1=0;
	s2=0;
	s3=0;
	s4=0;//�ù�֮�������������
	collect[0]=roll_1+rolls1;
	collect[1]=roll_2+rolls2;
	collect[2]=roll_3+rolls3;
	collect[3]=roll_4+rolls4;
	uint32_t collect[4]={roll_1+rolls1,roll_2+rolls2,roll_3+rolls3,roll_4+rolls4};//��ע���ܿӵ����⣬uint32_t��ֵ������Ϊ����������ͻ��ɺܴ������
	delay_ms(30);
	watch(1,1,50);
	watch(2,0,61);
	watch(3,1,50);
	watch(4,0,61);
	watch_together();
}

uint8_t turnangle_set(uint8_t angle){
	return angle;
}
void line1_left(void){//��һ��������Ի�е���趨λ��
	servo_edit(936,429,565);
}
void line1_right(void){//��һ���Ҳ����Ի�е���趨λ�ã����Ұ���С��ǰ���������,��ʱ�Ҳ�����ֱ
	servo_edit(174,442,565);
}
void line1_middle(void){//��һ�л�е�۴����м�λ�ô�
	servo_edit(544,441,564);
}
void line_fold(void){//��е�������۵��ĺ���
	servo_edit(556,777,286);
}
void line2_left(void){//�ڵڶ�����С��ǰ������Ϊ����������������
	servo_edit(900,422,624);
}
void line2_right(void){
	servo_edit(180,422,625);

}
void line2_config(uint8_t li){//line1->B
	if(line1[li]==1){
		play1();//��΢�ɺ�
		jiaoshui();
	}
	else if(line1[li]==2){
		play2();//һ��ɺ�
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
	else if(line1[li]==3){
		play3();//���ظɺ�
		jiaoshui();
		delay_ms(500);
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
}
void line1_config(uint8_t li){//line2-A
	if(line2[li]==1){
		play1();//��΢�ɺ�
		jiaoshui();
	}
	else if(line2[li]==2){
		play2();//һ��ɺ�
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
	else if(line2[li]==3){
		play3();//���ظɺ�
		jiaoshui();
		delay_ms(500);
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
}
void line3_config(uint8_t li){//line3->C


	if(line3[li]==1){
		play1();//��΢�ɺ�
		jiaoshui();
	}
	else if(line3[li]==2){
		play2();//һ��ɺ�
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
	else if(line3[li]==3){
		play3();//���ظɺ�
		jiaoshui();
		delay_ms(500);
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
}
void line4_config(uint8_t i){//���ǵ����еĲ����͹�Ⱥ���
	if(i==1){
		play1();//��΢�ɺ�
		jiaoshui();
	}
	else if(i==2){
		play2();//һ��ɺ�
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
	else if(i==3){
		play3();
	  jiaoshui();
		delay_ms(500);
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
}
void line3_left(void){
	servo_edit(913,422,534);//������ڳ�ʼλ��
	sendnumber(1);//����mode1
	special_delay_2();
	special_delay_2();
	judge();
	caluclate3l(xl,yl);
	udelete(6);
}
/**
@Brief ���λ�õ���
@Param x1:���һ�Ĳ�����x2:������Ĳ���,x3:������Ĳ���
@Retval NULL
*/
void servo_edit(uint16_t x1,uint16_t x2,uint16_t x3){
	moveServo(1,x1,1000);
	moveServo(2,x2,1000);
	moveServo(3,x3,1000);
	delay_ms(1000);
}
void line3_right(void){
	servo_edit(171,434,521);
	sendnumber(2);
	special_delay_2();
	special_delay_2();
	judge();
	caluclate3r(xr,yr);
	udelete(6);
}
void line4_left(void){//�ڵ��������Ļ�е�۶���
	servo_edit(913,432,508);
	delay_ms(1000);
	sendnumber(3);//���������ʱ����������������С��ǰ���ķ���
	special_delay_2();
	special_delay_2();
	judge();
	caluclate4l(xl,yl);
	//line4_config(color);
	udelete(6);
}
void line4_right(void){//�ڵ������Ҳ�Ļ�е�۶���
  servo_edit(151,450,513);
	delay_ms(1000);
	sendnumber(4);//�������Ҳ�ʱ������
	special_delay_2();
	special_delay_2();
  judge();
	caluclate4r(xr,yr);
	//line4_config(color);
	udelete(6);
}
/**
@Brief ���ڵ������Ҳ����������ֵ
@Param �õ�����໨���x��y������
@Retval һ��������Ԫ����ɵ�����Ҳ������������˶�����
*/
void caluclate3r(uint16_t x,uint16_t y){
	float angle1=atan2((y-128),(x+50))*(180/3.14);
	int angle1i=(int)angle1;
	float a=171-(angle1i/0.20);
	int ai=(int)a;//���Ƕ��һ��Ҫ�Ĳ���
	if(ai<=0){
		ai=0;
	}

	float bangle1=(acos(25/(sqrt(pow((x+50),2)/25+pow((y-128),2)/25+900)))-atan2(30,(sqrt(pow(x+50,2)/25+pow(y-128,2)/25))))*(180/3.14);
	int bangle1i=(int)bangle1;//���Ƕ������Ҫ�ĽǶ�
	float b=521+(bangle1i/0.3);
	int bi=(int)b;
	if(ai>=1000){
		ai=1000;
	}
	if(bi<=0){
		bi=0;
	}
	if(bi>=1000){
		bi=1000;
	
	}
	servo_edit(ai,434,bi);
}
/**
@Brief ���ڵ�����������������ֵ
@Param �õ����Ҳ໨���x��y������
@Retval һ��������Ԫ����ɵ�����Ҳ������������˶�����
*/
void caluclate3l(uint16_t x,uint16_t y){
	float angle1=atan2((y-107),(x+108))*(180/3.14);
	int angle1i=(int)angle1;
	float a=913-(angle1i/0.2);
	int ai=(int)a;//���Ƕ��һ��Ҫ�Ĳ���
	if(ai<=0){
		ai=0;
	}
	float bangle1=(acos(25/(sqrt(pow((x+108),2)/25+pow((y-107),2)/25+900)))-atan2(30,(sqrt(pow(x+108,2)/25+pow(y-107,2)/25))))*(180/3.14);
	int bangle1i=(int)bangle1;//���Ƕ������Ҫ�ĽǶ�
	float b=413+(bangle1i/0.3);
	int bi=(int)b;
	if(ai>=1000){
		ai=1000;
	}
	if(bi<=0){
		bi=0;
	}
	if(bi>=1000){
		bi=1000;
	
	}
	servo_edit(ai,448,bi);
}
/**
@Brief ���ڵ�����������������ֵ
@Param �õ�����໨���x��y������
@Retval һ��������Ԫ����ɵ�����Ҳ������������˶�����
*/
void caluclate4l(uint16_t x,uint16_t y){
	float angle1=atan2((y-136),(x+70))*(180/3.14);
	int angle1i=(int)angle1;
	float a=913-(angle1i/0.2);
	int ai=(int)a;//���Ƕ��һ��Ҫ�Ĳ���
	if(ai<=0){
		ai=0;
	}
	float bangle1=(acos(25/(sqrt(pow((x+70),2)/25+pow((y-136),2)/25+900)))-atan2(30,(sqrt(pow(x+70,2)/25+pow(y-136,2)/25))))*(180/3.14);
	int bangle1i=(int)bangle1;//���Ƕ������Ҫ�ĽǶ�
	float b=508+(bangle1i/0.25);//����ʵ�ʲ���,�ڵ����е�ʱ��ˮ������Ӧ�õ�СһЩ��ԭ����ֵ��0.3
	int bi=(int)b;
	if(ai>=1000){
		ai=1000;
	}
	if(bi<=0){
		bi=0;
	}
	if(bi>=1000){
		bi=1000;
	}

	servo_edit(ai,432,bi);
}
/**
@Brief ���ڵ������Ҳ����������ֵ
@Param �õ����Ҳ໨���x��y������
@Retval һ��������Ԫ����ɵ�����Ҳ������������˶�����
*/
void caluclate4r(uint16_t x,uint16_t y){
	float angle1=atan2((y-110),(x+50))*(180/3.14);
	int angle1i=(int)angle1;
	float a=151-(angle1i/0.20);
	
	int ai=(int)a;//���Ƕ��һ��Ҫ�Ĳ���
	if(ai<=0){
		ai=0;
	}
	float bangle1=(acos(25/(sqrt(pow((x+50),2)/25+pow((y-110),2)/25+900)))-atan2(30,(sqrt(pow(x+50,2)/25+pow(y-110,2)/25))))*(180/3.14);
	int bangle1i=(int)bangle1;//���Ƕ������Ҫ�ĽǶ�
	float b=532+(bangle1i/0.25);
	int bi=(int)b;
	if(bi<=0){
		bi=0;
	}
	if(bi>=1000){
		bi=1000;
	}
	if(ai>=1000){
		ai=1000;
	}

	servo_edit(ai,450,bi);
}//ע����ɫ��ֵ���׺ͳ��صĻ�ɫ ������Ӧ��������ֵ�趨��СЩ
/**
@Brief ���ڸ��ݰ�ɫʮ�־�������ĺ���
@Param NULL
@Brief NULL
*/
void modify_lines(void){
	line_watch();
	delay_ms(1000);
	distance=0;
	dydistance=0;//�ݷ���ľ�������
	sendnumber(5);
	special_delay_2();
	special_delay_2();
	special_delay_2();
  judge();
	distance=35+((xc-148)/5);//��������߼�ľ��뼴��
	dydistance=(yc-125)/5;//dydistance���ܻ��Ǹ��������Ҫ����Ϊint32_t
	line_fold();//�۵��ú�����	
	udelete(6);	

}
void line_watch(void){//����ɫʮ��ʱ��Ļ�е��λ��
	servo_edit(544,441,460);
}
float float_abs(float k){//����ȡ�ø������ľ���ֵ
	return k>0?k:-k;
}
void small_modify(uint8_t mode){//΢С�����ĺ���
	uint8_t modifycount=0;//��ʾ��¼΢�����������������ֱ���˳���˵�����ֹ���
	if(mode==1){//��ʾ������a����c���ĵ���������������Ŀ������0�ȵ���
		while(zyaw!=0){
			if(zyaw>0){//����zyaw������Ļ���Ӧ������΢��
				right_modify();
				modifycount+=1;
				if(modifycount>=4){
					break;
				}
			}
			else {
				left_modify();
				modifycount+=1;
				if(modifycount>=4){
					break;
				}
			}
		}
	}
	if(mode==2){//��ʾ����b����d���ĵ�������
		while(change_angle_modify(zyaw)!=179){
			if(change_angle_modify(zyaw)<179){
				right_modify();
				modifycount+=1;
				if(modifycount>=4){
					break;
				}
			}
			else{
				left_modify();
				modifycount+=1;
				if(modifycount>=4){
					break;	
				}
			}
		}
	}
	if(mode==3){
		while(abs(zyaw)!=90){
			if(abs(zyaw)>90){
				left_modify();
				modifycount+=1;
				if(modifycount>=4){
					break;
				}
			}
			else{
				right_modify();
				if(modifycount>=4){
					break;
				}
			}
		}
	}
}
void left_modify(void){//����΢���ĺ���
	delay_ms(30);
	watch_vel(1,1);
	watch_vel(2,1);
	watch_vel(3,1);
	watch_vel(4,1);
	watch_together();//��ʾ�ලһ���˶�
	delay_ms(70);
	STEP_Stop();
}
void right_modify(void){//��������΢���ĺ��������Ұ���С��ǰ���ķ�������
	delay_ms(30);
	//��16ϸ�ֵ�״̬�£�3200ת��һȦ��32000��ʮȦ
  watch_vel(1,0);
	watch_vel(2,0);
	watch_vel(3,0);
	watch_vel(4,0);
	watch_together();//��ʾ�ලһ���˶�
	delay_ms(70);
	STEP_Stop();
}
int change_angle_modify(int z){
	return z<0?-z:360-z;

}