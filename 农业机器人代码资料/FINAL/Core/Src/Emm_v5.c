#include "Emm_V5.h"
#include "gpio.h"
#include "tim.h"
#include "LobotServoController.h"
#include "music.h"
#include "JY901S.h"
#include "tjc_usart_hmi.h"
#include "stdlib.h"
#include "NRF24L01.h"
#include "math.h"//用于计算三角函数
#include "usart.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/
extern float BUJIN_CIRCLE;//步进电机圈数
extern uint8_t line1[6];//每行的旱情信息显示
extern uint8_t line2[6];
extern uint8_t line3[6];
extern uint16_t BUJIN_1;//步进计数
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
int yroll;//y轴横滚角
int zyaw;//z轴偏航角
int xpitch;//x轴偏角
uint8_t turnflag1=0;//这是a 区的转弯标志
uint8_t turnflag2=0;//这是第二个转弯处的标志
uint8_t turnflag3=0;//这是在二区结束时候的拐弯标志
uint8_t turnflag4=0;//这是在三区开始时候的拐弯标志
uint8_t turnflag5=0;//这是在第三区结束时候的拐弯标志
uint8_t turnflag6=0;//这是在第四区开始时候的拐弯标志
uint8_t stopflag1=0;//这是在第一行末尾拐弯时候的结束标志
uint8_t stopflag2=0;//这是在第二行开头拐弯时候的结束标志
uint8_t stopflag3=0;//这是在第二行末尾拐弯时候的结束标志
uint8_t stopflag4=0;//这是在第三行开头拐弯时候的结束标志
uint8_t stopflag5=0;//这是在第三行末尾拐弯时候的结束标志
uint8_t stopflag6=0;//这是在第四行开头拐弯时候的结束标志
extern int k;//这是第二次转弯前的zyaw记忆值
int x1;//刚刚走到第二行的时候xpitch的记忆值
int x2;//在坡上的时候xpitch的记忆值
int y1;//在第一个坡坡上的时候的yroll的记忆值
int y2;//在第二个坡上的时候的yroll的记忆值
int z1;//在第一行末端转弯前zyaw的记忆值
int z3;//这是第二行末尾时候zyaw的记忆值
int z4;//这是第三行开头zyaw的记忆值
int z5;//这是第三行末尾时候zyaw的记忆值
int z6;//这是第四行开头时候转弯前zyaw的记忆值
uint8_t on_poflag1=0;//第一次上坡时候的标志位
uint8_t down_poflag1=0;//第一次下坡时候的标志位
uint8_t on_poflag2=0;//第二次上坡时候的标志位
uint8_t down_poflag2=0;//第二次下坡时候的标志位
uint8_t xgewei=0;
uint8_t xshibai=0;
uint8_t ygewei=0;
uint8_t yshibai=0;
uint8_t color=0;//用于在第四行的识别中接收颜色信息
uint8_t modifyflag=0;//这是a区和c区的矫正函数
uint8_t modifyflag1=0;//这是b区和d区的矫正函数,实际没有用到
extern uint8_t Rxdata;
extern uint32_t s1;//矫正参数
extern uint32_t s2;
extern uint32_t s3;
extern uint32_t s4;
uint16_t xl,yl,xr,yr=0;
uint32_t poi=0;//用于在上坡时候的调平计数
uint32_t poj=0;//用于在第二次上坡的时候的调平计数
uint8_t moveflag=0;//用于小车行进时候的标志位
uint32_t movei=0;//用于小车行进时候的计数
uint16_t xc=0;//xc是识别白色十字的时候返回的横坐标
uint8_t yc=0;//yc是识别白色十字的时候返回的纵坐标,实际上不用
uint32_t distance=0;//识别白字和机械臂轴心的距离
int32_t dydistance=0;//表示识别白色十字后对于纵向移动距离的纠正
uint8_t spy_po_flag=0;//第三行检测到坡后的标志位，检测后拉到1
uint32_t roll_1=0;//下面这些是用于小车前进时候声明的全局变量,这是右前轮的
uint32_t roll_2=0;//这是左前轮的
uint32_t roll_3=0;//这是右后轮的
uint32_t roll_4=0;//这是左后轮的
uint32_t rolls1=0;//服务于矫正参数的
uint32_t rolls2=0;
uint32_t rolls3=0;
uint32_t rolls4=0;
uint8_t spy_down_po_flag=0;//表示看见坡之后启用下坡前进方式的函数
uint8_t judge_flag=0;//表示判断回读的步进电机命令
uint32_t collect[4]={0,0,0,0};

void ALL_INIT(void){
	HAL_UART_Receive_IT(&huart1,RxBuff,1);//开启k210的中断，baud=115200
	HAL_UART_Receive_IT(&huart4,RxBuff_1,1);//开启ZDT步进电机的中断，baud=115200
	HAL_UART_Receive_IT(&huart3,&Rxdata,1);//开启JY901S串口接收的中断
	JY901S_init();//初始化陀螺仪
	HAL_TIM_Base_Start_IT(&htim4);//开启步进电机中断
	HAL_TIM_Base_Start_IT(&htim6);//开启JY901S的定时器的中断
	HAL_TIM_Base_Start_IT(&htim2);//开启总的处理部分的中断
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//TIM_C1->步进 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,7);//设置浇水的水流速，TIM_C4->浇水	

}
/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 功能码
  cmd[2] =  0x52;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;//读取实时位置
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, i);
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 6);
}

/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0是直接启动
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 8);
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 13);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 功能码
  cmd[2] =  0x88;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32]={0};
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 功能码
  cmd[2] =  0xAE;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
  cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
  cmd[19] =  0x6B;                      // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 功能码
  cmd[2] =  0x48;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 4);
}
/**
  * @brief    step motor init
  * @param    NULL
  * @retval   NULL
  */


uint8_t L1,L2,M,R1,R2;
void step_clear(){//清零闭环步进电机示数的函数，实际没有用到
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
void HUIDU_read(void){//读取灰度传感器信息
	L2=HAL_GPIO_ReadPin(HUIDU_1_GPIO_Port,HUIDU_1_Pin);
	L1=HAL_GPIO_ReadPin(HUIDU_2_GPIO_Port,HUIDU_2_Pin);
	M=HAL_GPIO_ReadPin(HUIDU_3_GPIO_Port,HUIDU_3_Pin);
	R1=HAL_GPIO_ReadPin(HUIDU_4_GPIO_Port,HUIDU_4_Pin);
	R2=HAL_GPIO_ReadPin(HUIDU_5_GPIO_Port,HUIDU_5_Pin);
}
void watch_vel(uint8_t ID,uint8_t direction){//默认按照速度40，加速度50来运动,监督速度模式的函数
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
			if(g(1)!=0xF6||g(2)!=0x02){//表示没有正确发送
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
void STEP_Turn_1(void){	//在第一个转弯处的动作函数
	delay_ms(30);
	//在16细分的状态下，3200转是一圈，32000是十圈
  watch_vel(1,0);
	watch_vel(2,0);
	watch_vel(3,0);
	watch_vel(4,0);
	watch_together();//表示监督一齐运动
	turnflag1=1;
}
void STEP_Turn_2(void){//在第二行开始处的拐弯
	delay_ms(30);
	//在16细分的状态下，3200转是一圈，32000是十圈
  watch_vel(1,0);
	watch_vel(2,0);
	watch_vel(3,0);
	watch_vel(4,0);
	watch_together();//表示监督一齐运动
	turnflag2=1;
}
void STEP_Turn_3(void){//在第二行结束的时候开始拐弯
	delay_ms(30);
	watch_vel(1,1);
	watch_vel(2,1);
	watch_vel(3,1);
	watch_vel(4,1);
	watch_together();//表示监督一齐运动
	turnflag3=1;
}
void STEP_Turn_4(void){//在第三行开始的时候开始拐弯
	delay_ms(30);
	watch_vel(1,1);
	watch_vel(2,1);
	watch_vel(3,1);
	watch_vel(4,1);
	watch_together();//表示监督一齐运动
	turnflag4=1;
}
void STEP_Turn_5(void){//在第三行结束的时候开始拐弯
	delay_ms(30);
	watch_vel(1,0);
	watch_vel(2,0);
	watch_vel(3,0);
	watch_vel(4,0);
	watch_together();//表示监督一齐运动
	turnflag5=1;
}
void STEP_Turn_6(void){//在第四行开始的时候开始拐弯
	delay_ms(30);
	watch_vel(1,0);
	watch_vel(2,0);
	watch_vel(3,0);
	watch_vel(4,0);
	watch_together();//表示监督一齐运动
	turnflag6=1;
}
void watch_stop(uint8_t ID){//监管小车停止的函数
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
			if(g(1)!=0xFE||g(2)!=0x02){//表示没有正确发送
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
@Brief 用速度环控制的小车运动，实际上不好用
@Param distance
@Retval NULL
*/
void vel_move(uint16_t distance,uint8_t s1,uint8_t s2,uint8_t s3,uint8_t s4){
	uint32_t a=0;//代指角度
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
  Emm_V5_Synchronous_motion(0);//两者同时运动
	delay_ms(50);
	initRingBuff1();
	while(step_flag==1){//26.37cm是车轮的周长
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
			step_clear();//角度清零
			step_flag=0;
			break;
		}
	}
}
void bujin_zheng(float quanshu){//控制调平步进正转,往上运动
	HAL_GPIO_WritePin(bujin_CON_GPIO_Port,bujin_CON_Pin,GPIO_PIN_SET);
	BUJIN_1=0;
	BUJIN_CIRCLE=quanshu;
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,5);
}
void bujin_fan(float quanshu){//控制调平步进反转，往下运动
	HAL_GPIO_WritePin(bujin_CON_GPIO_Port,bujin_CON_Pin,GPIO_PIN_RESET);
	BUJIN_1=0;
	BUJIN_CIRCLE=quanshu;
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,5);
}
/**
@Brief 第一行灌溉报时的函数
@Param 第一行的花盆数
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
@Brief 第二行灌溉报时的函数
@Param 第二行的花盆数
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
@Brief 第三行灌溉报时的函数
@Param 第三行的花盆数
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
@Brief 第四行灌溉报时的函数
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
//下面若干函数过于冗余
void special_delay(void){//用于小车行进时候的普通延时函数
	moveflag=1;
	movei=0;
	while(moveflag==1){
		delay_ms(10);
	}
}
void special_delay_2(void){//用于k210等的特殊延时函数
	uint8_t i;
	for(i=0;i<20;i++){
		delay_ms(200);
	
	}
}
//本小车有两个中心，一个是机械臂中心，另外一个是车中心，但凡涉及浇水动作的则启用机械臂中心，涉及到转弯则启用车中心
void line1_act(void){//小车在第一行的相应的行动
	modifyflag=1;//开启自纠正功能
	move_forward(55,s1,s2,s3,s4);
	special_delay();//这是卡时间的延时函数，卡时间为了防止车出现意外，因此立刻停下	
	line1_choose(0);//这是执行浇水和播报的函数，0代表第一个位置
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
	move_forward(70,s1,s2,s3,s4);//走到白色十字处，一般来说走出去的距离应该是下一行走进来的距离+20cm
	special_delay();	
	modifyflag=0;//一定要前后呼应

	small_modify(1);//转弯前对齐
//开始转弯
	z1=abs(zyaw);
	STEP_Turn_1();//第一次转90度，实际上略微大于这个值
	while(stopflag1==0){
		delay_ms(5);
	}
	STEP_Stop();
	move_forward(70,0,0,0,0);//先走60cm
	special_delay();
	modify_lines();//然后开始观测和白色十字间的距离
	if(distance<=10){//保险行
		distance=40;
	}
	move_forward(distance,0,0,0,0);//走到白色十字处
	special_delay();
	STEP_Turn_2();//走到第二行起始位置处，第二次转90度
	while(stopflag2==0){
		delay_ms(5);
	}
	STEP_Stop();
}
void line2_act(void){
		//开始第二区
	modifyflag1=1;//决定是否启用调整功能,modifyflag1适用于b区和d区
	move_forward(50,s1,s2,s3,s4);
	special_delay();
	line2_choose(0);//第二行的浇水灌溉函数
	move_forward(60,s1,s2,s3,s4);
	special_delay();
	line2_choose(1);
  on_poflag1=1;
  down_poflag1=1;
	poi=0;
	y1=abs(yroll);
	move_forward(60,s1,s2,s3,s4);//这样做是为了避免上坡途中浇水,这是第二行上坡的一段
	special_delay();
	line2_choose(2);
	move_forward(60,s1,s2,s3,s4);//这是第二行在坡上行走的一段
	special_delay();
	line2_choose(3);
	move_forward(60,s1,s2,s3,s4);
	special_delay();//特殊延时函数，卡延时是为了防止小车出现意外
	line2_choose(4);
	move_forward(60,s1,s2,s3,s4);
	special_delay();//适用于60cm的特殊延时函数
	line2_choose(5);
	move_forward(70,s1,s2,s3,s4);//走出70cm
	special_delay();
	modifyflag1=0;	
	z3=zyaw;	//记录当前偏航角
	//开始转弯
	STEP_Turn_3();	
	while(stopflag3==0){
		delay_ms(5);
	}
	STEP_Stop();
  move_forward(70,0,0,0,0);//先走出60cm侯检测白色十字和车之间的距离
	special_delay();
	modify_lines();
	if(distance<=15){//保险行
		distance=50;
	}
	move_forward(distance,0,0,0,0);
	special_delay();
	z4=zyaw;//记录当前的偏航角
	STEP_Turn_4();
	while(stopflag4==0){
		delay_ms(5);
	}
	STEP_Stop();
	//拐弯完成后

}
void line3_act(void){
	modifyflag=1;//modifyflag适用于a,c两区域
	poj=0;
	on_poflag1=0;
	down_poflag1=0;
	on_poflag2=1;
	down_poflag2=1;
	move_forward(50,s1,s2,s3,s4);
	special_delay();	
	y1=abs(yroll);//记录当前yroll的值
	line3_choose(0);//第三行自动识别的函数，已经写好封装好，改动就改k210.py里面的即可
	move_forward(60,s1,s2,s3,s4);//为了检测上坡下坡而准备的函数
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
	move_forward(70,s1,s2,s3,s4);//走出65cm
	special_delay();
	modifyflag=0;//此时将modifyflag拉低到0
	//开始转弯
	z5=zyaw;
	STEP_Turn_5();
	while(stopflag5==0){
		delay_ms(10);
	}
	STEP_Stop();
	//small_modify(3);//该函数不适合用太多，否则车很可能走邪路
	move_forward(70,0,0,0,0);//先走出60cm识别白色十字
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
	//modifyflag1=1;//modifyflag1是为b区,d区定制的
	move_forward(50,s1,s2,s3,s4);
	special_delay();
	move_forward(20,s1,s2,s3,s4);
	special_delay();	
	line1_middle();
	line4_right();
	line4_config(3);
	line1_middle();
	line_fold();
	//line4_choose();//第四行的特殊识别灌溉函数，当小车处在该行正中间的时候效果最佳
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
float change_angle(float angle){//在进入第二行前转弯所用
	return (angle<=0)?-angle:180+(180-angle);
}
float change_angle_1(float angle){//在进入第三行前转弯所用
	return (angle<=0)?-angle:-angle;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){//定时器中断的处理函数
	if(htim==(&htim4)){//负责驱动步进电机
		BUJIN_1+=1;
		if(BUJIN_1>=3200*BUJIN_CIRCLE){	
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
			BUJIN_1=0;
		}

		
	}
  if(htim==(&htim6)){//负责解算得到JY901S的角度信息，htim6->解算得到陀螺仪信息
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
	if(htim==(&htim2)){//用于判断上坡过程,50ms进入中断一次
		if(turnflag1==1){//这是出第一行后的转弯
			if(float_abs(anglez)>=84){
				turnflag1=0;
				stopflag1=1;
		}
		}
		if(turnflag2==1){//这是进入第二行前的转弯
			if(change_angle(anglez)>=174){
				turnflag2=0;
				stopflag2=1;
			}
		}
		if(on_poflag1==1){//第一次上坡的函数
			if(float_abs(angley)>=y1+6){
				bujin_fan(1.5);
				poi=0;
				on_poflag1=0;
				y2=yroll;//记录此时的y值
			}
		}

		if(down_poflag1==1&&on_poflag1==0){//设置双重保护
			poi++;
			if(poi>=3300){
				bujin_zheng(1.5);
				down_poflag1=0;
				poi=0;
			}
		}
		if(moveflag==1){//用于给小车前进提供延时服务
			movei++;
			if(movei>=400){
				moveflag=0;
				movei=0;
			}
		}
		if(turnflag3==1){//在出第二行后的转弯
			if(float_abs(anglez)<=95){//都要留出提前量
				turnflag3=0;
				stopflag3=1;
			}
		}
		if(turnflag4==1){//在进入第三行前的转弯，转90度
			if(change_angle_1(anglez)<=3){
				turnflag4=0;
				stopflag4=1;
			}
		}
		if(turnflag5==1){//这是在出第三行后的转弯
			if(float_abs(anglez)>=84){
				turnflag5=0;
				stopflag5=1;
			}
		}
		if(turnflag6==1){//这是在进第四行前的转弯
			if(change_angle(anglez)>=176){
				turnflag6=0;
				stopflag6=1;
			}
		}
		if(modifyflag==1){//a区和c区的调整方案
				if(anglez>=0.5){
					s1=0;
					s3=0;
					s2=0.5*abs(zyaw);//一定要加取绝对值的函数，不加的话有可能出现数据溢出
					s4=0.5*abs(zyaw);
				}
				if(anglez<=-0.5){
					s1=0.5*abs(zyaw);
					s3=0.5*abs(zyaw);
					s2=0;
					s4=0;
				}
		}
		if(modifyflag1==1){//b区和d区的调整方案,注意a区和b区的小车的调整方案不同的
				if(anglez>=-179.5&&anglez<0){
					s1=0;
					s3=0;
					s2=0.5*abs(180-abs(zyaw));//一定要加取绝对值的函数，不加的话有可能出现数据溢出
					s4=0.5*abs(180-abs(zyaw));
				}
				if(anglez<=179.5&&anglez>0){
					s1=0.5*abs(180-abs(zyaw));
					s3=0.5*abs(180-abs(zyaw));
					s2=0;
					s4=0;
				}
		}
		if(on_poflag2==1){//检测angley来判断上下坡
			if(float_abs(angley)>=y1+7){
				bujin_fan(1.5);
				on_poflag2=0;
				y2=yroll;//
			}
		}

		if(down_poflag2==1&&on_poflag2==0){//设定双重保护
			poj++;
			if(poj>=4700){
				bujin_zheng(1.5);
				down_poflag2=0;			
			}
		}
	}
}
void jiaoshui(void){//浇水灌溉函数
	HAL_GPIO_WritePin(pump_1_GPIO_Port,pump_1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(pump_2_GPIO_Port,pump_2_Pin,GPIO_PIN_RESET);
	delay_ms(400);
	HAL_GPIO_WritePin(pump_1_GPIO_Port,pump_1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pump_2_GPIO_Port,pump_2_Pin,GPIO_PIN_RESET);
}
void watch(uint8_t address,uint8_t direction,uint8_t acc){//用于监管前进位置函数防止电机犯病的函数
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
			if(g(1)!=0xFD||g(2)!=0x02){//表示没有正确发送
			watch(address,direction,acc);
			gdelete(4);
			errorcount+=1;
			if(errorcount>=5){
				errorflag=1;//表示检测到错误发生，先跑路了
				
			}
		}
	}
}
	gdelete(4);
}
void watch_together(void){//用于监管看护一齐运行的函数  
	Emm_V5_Synchronous_motion(0);//两者同时运动
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
			if(g(1)!=0xFF||g(2)!=0x02){//表示没有正确发送
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
void move_forward(uint32_t distance,uint32_t ks1,uint32_t ks2,uint32_t ks3,uint32_t ks4){//小车向前运动多少cm的函数。	
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
	s4=0;//用过之后即清零纠正参数
	collect[0]=roll_1+rolls1;
	collect[1]=roll_2+rolls2;
	collect[2]=roll_3+rolls3;
	collect[3]=roll_4+rolls4;
	uint32_t collect[4]={roll_1+rolls1,roll_2+rolls2,roll_3+rolls3,roll_4+rolls4};//标注，很坑的问题，uint32_t的值不可以为负数，否则就会变成很大的正数
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
void line1_left(void){//第一行左侧盆栽机械臂设定位置
	servo_edit(936,429,565);
}
void line1_right(void){//第一行右侧盆栽机械臂设定位置，左，右按照小车前进方向而定,此时右侧正垂直
	servo_edit(174,442,565);
}
void line1_middle(void){//第一行机械臂处于中间位置处
	servo_edit(544,441,564);
}
void line_fold(void){//机械臂弯曲折叠的函数
	servo_edit(556,777,286);
}
void line2_left(void){//在第二行以小车前进方向为正方向来区分左右
	servo_edit(900,422,624);
}
void line2_right(void){
	servo_edit(180,422,625);

}
void line2_config(uint8_t li){//line1->B
	if(line1[li]==1){
		play1();//轻微干旱
		jiaoshui();
	}
	else if(line1[li]==2){
		play2();//一般干旱
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
	else if(line1[li]==3){
		play3();//严重干旱
		jiaoshui();
		delay_ms(500);
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
}
void line1_config(uint8_t li){//line2-A
	if(line2[li]==1){
		play1();//轻微干旱
		jiaoshui();
	}
	else if(line2[li]==2){
		play2();//一般干旱
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
	else if(line2[li]==3){
		play3();//严重干旱
		jiaoshui();
		delay_ms(500);
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
}
void line3_config(uint8_t li){//line3->C


	if(line3[li]==1){
		play1();//轻微干旱
		jiaoshui();
	}
	else if(line3[li]==2){
		play2();//一般干旱
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
	else if(line3[li]==3){
		play3();//严重干旱
		jiaoshui();
		delay_ms(500);
		jiaoshui();
		delay_ms(500);
		jiaoshui();
	}
}
void line4_config(uint8_t i){//这是第四行的播报和灌溉函数
	if(i==1){
		play1();//轻微干旱
		jiaoshui();
	}
	else if(i==2){
		play2();//一般干旱
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
	servo_edit(913,422,534);//舵机臂在初始位置
	sendnumber(1);//发送mode1
	special_delay_2();
	special_delay_2();
	judge();
	caluclate3l(xl,yl);
	udelete(6);
}
/**
@Brief 舵机位置调控
@Param x1:舵机一的参数，x2:舵机二的参数,x3:舵机三的参数
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
void line4_left(void){//在第四行左侧的机械臂动作
	servo_edit(913,432,508);
	delay_ms(1000);
	sendnumber(3);//第四行左侧时候发送三，正方向是小车前进的方向
	special_delay_2();
	special_delay_2();
	judge();
	caluclate4l(xl,yl);
	//line4_config(color);
	udelete(6);
}
void line4_right(void){//在第四行右侧的机械臂动作
  servo_edit(151,450,513);
	delay_ms(1000);
	sendnumber(4);//第四行右侧时候发送四
	special_delay_2();
	special_delay_2();
  judge();
	caluclate4r(xr,yr);
	//line4_config(color);
	udelete(6);
}
/**
@Brief 用于第三行右侧解算舵机参数值
@Param 得到的左侧花盆的x和y的坐标
@Retval 一个由三个元素组成的数组也即三个舵机的运动数组
*/
void caluclate3r(uint16_t x,uint16_t y){
	float angle1=atan2((y-128),(x+50))*(180/3.14);
	int angle1i=(int)angle1;
	float a=171-(angle1i/0.20);
	int ai=(int)a;//这是舵机一需要的参数
	if(ai<=0){
		ai=0;
	}

	float bangle1=(acos(25/(sqrt(pow((x+50),2)/25+pow((y-128),2)/25+900)))-atan2(30,(sqrt(pow(x+50,2)/25+pow(y-128,2)/25))))*(180/3.14);
	int bangle1i=(int)bangle1;//这是舵机三需要的角度
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
@Brief 用于第三行左侧解算舵机参数值
@Param 得到的右侧花盆的x和y的坐标
@Retval 一个由三个元素组成的数组也即三个舵机的运动数组
*/
void caluclate3l(uint16_t x,uint16_t y){
	float angle1=atan2((y-107),(x+108))*(180/3.14);
	int angle1i=(int)angle1;
	float a=913-(angle1i/0.2);
	int ai=(int)a;//这是舵机一需要的参数
	if(ai<=0){
		ai=0;
	}
	float bangle1=(acos(25/(sqrt(pow((x+108),2)/25+pow((y-107),2)/25+900)))-atan2(30,(sqrt(pow(x+108,2)/25+pow(y-107,2)/25))))*(180/3.14);
	int bangle1i=(int)bangle1;//这是舵机三需要的角度
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
@Brief 用于第四行左侧解算舵机参数值
@Param 得到的左侧花盆的x和y的坐标
@Retval 一个由三个元素组成的数组也即三个舵机的运动数组
*/
void caluclate4l(uint16_t x,uint16_t y){
	float angle1=atan2((y-136),(x+70))*(180/3.14);
	int angle1i=(int)angle1;
	float a=913-(angle1i/0.2);
	int ai=(int)a;//这是舵机一需要的参数
	if(ai<=0){
		ai=0;
	}
	float bangle1=(acos(25/(sqrt(pow((x+70),2)/25+pow((y-136),2)/25+900)))-atan2(30,(sqrt(pow(x+70,2)/25+pow(y-136,2)/25))))*(180/3.14);
	int bangle1i=(int)bangle1;//这是舵机三需要的角度
	float b=508+(bangle1i/0.25);//经过实际测试,在第四行的时候浇水俯仰角应该调小一些，原来数值是0.3
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
@Brief 用于第四行右侧解算舵机参数值
@Param 得到的右侧花盆的x和y的坐标
@Retval 一个由三个元素组成的数组也即三个舵机的运动数组
*/
void caluclate4r(uint16_t x,uint16_t y){
	float angle1=atan2((y-110),(x+50))*(180/3.14);
	int angle1i=(int)angle1;
	float a=151-(angle1i/0.20);
	
	int ai=(int)a;//这是舵机一需要的参数
	if(ai<=0){
		ai=0;
	}
	float bangle1=(acos(25/(sqrt(pow((x+50),2)/25+pow((y-110),2)/25+900)))-atan2(30,(sqrt(pow(x+50,2)/25+pow(y-110,2)/25))))*(180/3.14);
	int bangle1i=(int)bangle1;//这是舵机三需要的角度
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
}//注意绿色阈值容易和场地的黄色 混淆，应当将其阈值设定的小些
/**
@Brief 用于根据白色十字纠正航向的函数
@Param NULL
@Brief NULL
*/
void modify_lines(void){
	line_watch();
	delay_ms(1000);
	distance=0;
	dydistance=0;//纵方向的距离修正
	sendnumber(5);
	special_delay_2();
	special_delay_2();
	special_delay_2();
  judge();
	distance=35+((xc-148)/5);//计算出两者间的距离即可
	dydistance=(yc-125)/5;//dydistance可能会是负数，因此要声明为int32_t
	line_fold();//折叠好后再走	
	udelete(6);	

}
void line_watch(void){//看白色十字时候的机械臂位置
	servo_edit(544,441,460);
}
float float_abs(float k){//用于取得浮点数的绝对值
	return k>0?k:-k;
}
void small_modify(uint8_t mode){//微小调整的函数
	uint8_t modifycount=0;//表示记录微调次数，假如过大则直接退出，说明出现故障
	if(mode==1){//表示这是在a区和c区的调整函数，调整的目标是向0度调整
		while(zyaw!=0){
			if(zyaw>0){//假如zyaw大于零的话则应当向右微调
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
	if(mode==2){//表示这是b区和d区的调整函数
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
void left_modify(void){//向左微调的函数
	delay_ms(30);
	watch_vel(1,1);
	watch_vel(2,1);
	watch_vel(3,1);
	watch_vel(4,1);
	watch_together();//表示监督一齐运动
	delay_ms(70);
	STEP_Stop();
}
void right_modify(void){//这是向右微调的函数，左右按照小车前进的方向来定
	delay_ms(30);
	//在16细分的状态下，3200转是一圈，32000是十圈
  watch_vel(1,0);
	watch_vel(2,0);
	watch_vel(3,0);
	watch_vel(4,0);
	watch_together();//表示监督一齐运动
	delay_ms(70);
	STEP_Stop();
}
int change_angle_modify(int z){
	return z<0?-z:360-z;

}