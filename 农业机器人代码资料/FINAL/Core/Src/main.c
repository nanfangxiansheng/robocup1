/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24L01.h"
#include "tjc_usart_hmi.h"
#include "LobotServoController.h"
#include "Emm_v5.h"
#include "JY901S.h"
#include "music.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t line1[6];//第一行的旱情信息
uint8_t line2[6];//第二行的旱情信息
uint8_t line3[6];//第三行的旱情信息
uint8_t rj=0;//旱情传输	
uint8_t Buf[32] ;//旱情缓冲区储存
uint8_t NRflag=1;
uint16_t BUJIN_1=0;
float BUJIN_CIRCLE=0;
extern uint8_t Rxdata;
extern uint8_t turnflag1;
extern uint8_t turnflag2;
extern  uint8_t turnflag3;
extern uint8_t turnflag4;
extern uint8_t turnflag5;
extern uint8_t turnflag6;

extern int zyaw;
extern int xpitch;
int k=0;//用于暂存zyaw
extern int x2;
extern int x1;
extern int z1;
extern int z3;
extern int z4;
extern int z5;
extern int z6;
extern uint8_t on_poflag1;
extern uint8_t down_poflag1;
extern uint8_t on_poflag2;
extern uint8_t down_poflag2;
extern uint8_t stopflag1;
extern uint8_t stopflag2;
extern uint8_t stopflag3;
extern uint8_t stopflag4;
extern uint8_t stopflag5;
extern uint8_t  stopflag6;
extern uint32_t poi;
uint32_t s1=0;//步进一的速度加成
uint32_t s2=0;//步进二的速度加成
uint32_t s3=0;//步进三的速度加成
uint32_t s4=0;//步进四的速度加成
extern uint8_t modifyflag;
extern uint8_t modifyflag1;
extern uint16_t distance;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	initRingBuff();//为k210初始化
	initRingBuff1();//为接收ZDT数据初始化
	Buf_clear1();//为接收旱情信息的缓冲区进行初始化
	hanqing_init();//为旱情储存区初始化
	NRF24L01_Init();//初始化NRF设备
	while(NRflag==1){//这个循环用于接收旱情信息
		if (NRF24L01_Get_Value_Flag() == 0)
		{
			NRF24L01_GetRxBuf(Buf);
			judge_receive();
			rj++;	
			if(rj>=1){
				NRflag=0;
				break;
		  }	
	}
  }		
	show_hanqing(); //在串口屏上显示旱情数据
	ALL_INIT();//定时器，陀螺仪串口中断的初始化等等
	line_fold();
	line1_act();//第一行的动作函数
	line2_act();//b区动作函数
	line3_act();//c区动作函数
	line4_act();//d区动作函数

	 
	/*
	读者须知：在你们调试的时候，一定要注意电气安全，不能让电池电压低于11.2V,调前务必使得小车上面的平面处于水平状态
	我大部分的处理转弯和丝杠电机调平的函数都写在了定时器中断里面HAL_TIM_PeriodElapsedCallback(里面可以在各个turn_flag对应下，更改角度值)
	我使用了三个定时器，其中一个专门调平，一个解算JY901S数据，一个处理转弯等，我用到了大量的标志位，可以在Emm_v5.c的头部看到
	我用了多个延时函数，这是为了缩短时间，防止电机犯病，至于电机犯病的原因还没找到，怀疑可能是概率问题。
	上坡下坡是十分之困难的，轮胎似乎要直径更大一些才方便，此外，还有一个拙劣的办法，就是小车在坡上不作停留，直接通过，也可以试下。
	但是直径大些转弯可能不是很容易。
	调视觉识别的话可以在k210.py里面调，那个旱情识别模块已经没有任何问题了，你们要努力干，咱们时间不多了，想办法解决上下坡偏航的问题
	想办法找出电机犯病的问题。我这里面的参数轻易不要改动，都是多次测试后得出的，延时函数不要任意改，Final.ioc里面不要改
	我最近一周多十分忙，可能抽不出时间来
	*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		delay_ms(500);
		


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
