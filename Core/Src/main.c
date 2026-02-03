/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define  NO_SOUND       0x00
#define  FREQ_UP        0x01
#define  FREQ_DOWN      0x02

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCEN_MAX        3
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t direct_dev = 0; // 0 - ����������� ARR , 1 -  ��������� ARR
const uint8_t step_freq = 10;
uint8_t MAX_MIN_ARR[2] = {10, 100};
static uint8_t curr_freq = 10;

scenario_t Scenic[SCEN_MAX]={
  {.period_sec          = 3*2,
  .dir                  = 0,
   .fmin                = 5000,
   .f0                  = 5000,
   .fmax                = 50000,
   .step_freq_Hz        = 5000,
   .step_ms             = 3*1000}, 
     
   {.period_sec         = 60*15,
   .dir                 = 0,
   .fmin                = 5000,
   .f0                  = 5000,
   .fmax                = 50000,
   .step_freq_Hz        = 5000,
   .step_ms             = 60*5*1000},
     
   {.period_sec         = 60,
   .dir                 = 0,
   .fmin                = 5000,
   .f0                  = 5000,
   .fmax                = 50000,
   .step_freq_Hz        = 5000,
   .step_ms             = 300},

};

scenic_ctrl_t controller ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t get_Freq(uint8_t freq_type)
{
  if(freq_type > 1) freq_type = 0;
  return MAX_MIN_ARR[freq_type];
}

void changeScenario()
{

}
void stepFreq(scenario_t *_scen)
{

    switch(_scen->mode)
    {
      case FREQ_SWING:
      {
        if(_scen->f0 >= _scen->fmax)
        {
          _scen->f0 -= _scen->step_freq_Hz;
        }
        else
        {
          _scen->f0 += _scen->step_freq_Hz;
        }
      }
      break;
      case FREQ_INCREASE:
      {
        if(_scen->f0 >= _scen->fmax)
        {
          _scen->f0 = _scen->fmin;
        }
        else
        {
          _scen->f0 += _scen->step_freq_Hz;
        }
      }
      break;
      case FREQ_PULSE:
      {
        if(0 == _scen->dir){
          _scen->dir = 1;
          _scen->f0 = 0;
        }
        else{
          _scen->dir = 0;
          if(_scen->f0 >= _scen->fmax){
            _scen->f0 = _scen->fmin;
          }
          else{
            _scen->f0 += _scen->step_freq_Hz;
          }
        }
      }
      break;
    }
}

uint8_t _stepFreq(void)
{
  if( direct_dev)
  {
    if(curr_freq >= MAX_MIN_ARR[ARR_MIN_INDX] && curr_freq <= MAX_MIN_ARR[ARR_MAX_INDX])
    {
      curr_freq -= step_freq;
    }
    else
    {
      direct_dev = !direct_dev;
      curr_freq += step_freq;
    }
  }
  else
  {
    if(curr_freq >= MAX_MIN_ARR[ARR_MIN_INDX] && curr_freq <= MAX_MIN_ARR[ARR_MAX_INDX] )
    {
      curr_freq += step_freq;
    }
    else
    {
      direct_dev = !direct_dev;
      curr_freq -= step_freq;
    }
  }
  return curr_freq;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t st=0;
uint32_t tick =0;
scenario_t *sc;
 
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
  MX_TIM3_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  sc = &Scenic[controller.indx_scen];
  stepFreq(sc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  controller.time_start = HAL_GetTick();
  controller.time_stop  = 
tick = HAL_GetTick() + 1000;
  while (1)
  {
    if(HAL_GetTick()> tick)
    {
      tick += 200;
      uint8_t arr = _stepFreq();
      TIM3_Set_Arr( arr);
      
    }
    
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
