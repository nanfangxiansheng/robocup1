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
uint8_t line1[6];//��һ�еĺ�����Ϣ
uint8_t line2[6];//�ڶ��еĺ�����Ϣ
uint8_t line3[6];//�����еĺ�����Ϣ
uint8_t rj=0;//���鴫��	
uint8_t Buf[32] ;//���黺��������
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
int k=0;//�����ݴ�zyaw
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
uint32_t s1=0;//����һ���ٶȼӳ�
uint32_t s2=0;//���������ٶȼӳ�
uint32_t s3=0;//���������ٶȼӳ�
uint32_t s4=0;//�����ĵ��ٶȼӳ�
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
	initRingBuff();//Ϊk210��ʼ��
	initRingBuff1();//Ϊ����ZDT���ݳ�ʼ��
	Buf_clear1();//Ϊ���պ�����Ϣ�Ļ��������г�ʼ��
	hanqing_init();//Ϊ���鴢������ʼ��
	NRF24L01_Init();//��ʼ��NRF�豸
	while(NRflag==1){//���ѭ�����ڽ��պ�����Ϣ
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
	show_hanqing(); //�ڴ���������ʾ��������
	ALL_INIT();//��ʱ���������Ǵ����жϵĳ�ʼ���ȵ�
	line_fold();
	line1_act();//��һ�еĶ�������
	line2_act();//b����������
	line3_act();//c����������
	line4_act();//d����������

	 
	/*
	������֪�������ǵ��Ե�ʱ��һ��Ҫע�������ȫ�������õ�ص�ѹ����11.2V,��ǰ���ʹ��С�������ƽ�洦��ˮƽ״̬
	�Ҵ󲿷ֵĴ���ת���˿�ܵ����ƽ�ĺ�����д���˶�ʱ���ж�����HAL_TIM_PeriodElapsedCallback(��������ڸ���turn_flag��Ӧ�£����ĽǶ�ֵ)
	��ʹ����������ʱ��������һ��ר�ŵ�ƽ��һ������JY901S���ݣ�һ������ת��ȣ����õ��˴����ı�־λ��������Emm_v5.c��ͷ������
	�����˶����ʱ����������Ϊ������ʱ�䣬��ֹ������������ڵ��������ԭ��û�ҵ������ɿ����Ǹ������⡣
	����������ʮ��֮���ѵģ���̥�ƺ�Ҫֱ������һЩ�ŷ��㣬���⣬����һ��׾�ӵİ취������С�������ϲ���ͣ����ֱ��ͨ����Ҳ�������¡�
	����ֱ����Щת����ܲ��Ǻ����ס�
	���Ӿ�ʶ��Ļ�������k210.py��������Ǹ�����ʶ��ģ���Ѿ�û���κ������ˣ�����ҪŬ���ɣ�����ʱ�䲻���ˣ���취���������ƫ��������
	��취�ҳ�������������⡣��������Ĳ������ײ�Ҫ�Ķ������Ƕ�β��Ժ�ó��ģ���ʱ������Ҫ����ģ�Final.ioc���治Ҫ��
	�����һ�ܶ�ʮ��æ�����ܳ鲻��ʱ����
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
