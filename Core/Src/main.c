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

#define DEBUG 0

//#if ((DEBUG == 1))
//#define F_MIN           2000
//#define F_MAX           6000
//#define F_STEP          200
//#else
#define F_MIN           20000
#define F_MAX           60000
#define F_STEP          2000
//#endif
#define F_DIV_ULTRA     1
#define F_DIV_NORMAL    1
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

uint8_t direct_dev = 0; // 0 -  ARR , 1 -  ��������� ARR
const uint8_t step_freq = 10;
uint8_t MAX_MIN_ARR[2] = {10, 100};
static uint8_t curr_freq = 10;
uint32_t stop_step_ms =0;
static uint32_t cur_freq = F_MIN;

scenario_t Scenic[SCEN_MAX]={
  {.period_sec          = 60,
  .dir                  = 0,
   .fmin                = F_MIN,
   .f0                  = F_MIN,
   .fmax                = F_MAX,
   .step_freq_Hz        = F_STEP,
   .step_ms             = 5000,//3*1000
   .mode                = FREQ_PULSE 
  }, 
     
   {.period_sec         = 60,//60*15,
   .dir                 = 0,
   .fmin                = F_MIN,
   .f0                  = F_MIN,
   .fmax                = F_MAX,
   .step_freq_Hz        = F_STEP,
   .step_ms             = 5000,//60*5*1000
   .mode                = FREQ_INCREASE,
   },
     
   {.period_sec         = 60,
   .dir                 = 0,
   .fmin                = F_MIN,
   .f0                  = F_MIN,
   .fmax                = F_MAX,
   .step_freq_Hz        = F_STEP,
   .step_ms             = 5000,
   .mode                = FREQ_SWING
   },
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
  if(++controller.indx_scen>=SCEN_MAX)
  {
    controller.indx_scen = 0;
  }
}
void stepFreq(scenario_t *_scen)
{
uint8_t div = controller.freq_div;
    switch(_scen->mode)
    {
      case FREQ_SWING:
      {
        if(_scen->dir)
        {
          _scen->f0 += _scen->step_freq_Hz;
          if(_scen->f0 > _scen->fmax)
          {
            _scen->f0 -= _scen->step_freq_Hz;
            _scen->dir = ~_scen->dir;
          }
        }
        else
        {
          if((_scen->f0 >=_scen->fmin))
          {
            _scen->f0 -= _scen->step_freq_Hz;
           
          }
           if(_scen->f0 <_scen->fmin)
            {
              _scen->dir = ~_scen->dir;
              _scen->f0 += _scen->step_freq_Hz;
            }
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
        
        if(_scen->dir)
        {
          _scen->dir = 0;
          cur_freq += _scen->step_freq_Hz;
          
          if(cur_freq > _scen->fmax){
            cur_freq = _scen->fmin;
          }
          _scen->f0 += cur_freq;
        }
        else
        {
          _scen->dir = ~_scen->dir;
          _scen->f0 = 0;
        }
      }
      break;
    }
}

void SetFreq(uint32_t freq_Hz)
{
  uint32_t arr=0;
  // 10000Hz => arr = 100 тиков по 1 мкс
  // 50000Hz => arr = 20 тиков по 1 мкс
  if(freq_Hz !=0)
  {
    arr = (1000000/freq_Hz)/2;// т.к. TIM_OCMODE_TOGGLE то период делим на 2
  }
    TIM3_Set_Arr( arr);
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

  sc = &Scenic[controller.indx_scen];
//  stepFreq(sc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  controller.time_start = HAL_GetTick();
  controller.time_stop  = controller.time_start + sc->period_sec*1000;
  stop_step_ms =  HAL_GetTick() + sc->step_ms ;
  while (1)
  {
    if(HAL_GetTick()> controller.time_stop )
    {
      changeScenario();
      sc = &Scenic[controller.indx_scen];
      stepFreq(sc);
      SetFreq(sc->f0);
      controller.time_start = HAL_GetTick();
      controller.time_stop  = controller.time_start + sc->period_sec*1000;
      
    }
    if(HAL_GetTick()> stop_step_ms )
    {
      stepFreq(sc);
      SetFreq(sc->f0);
      stop_step_ms =  HAL_GetTick() + sc->step_ms ;
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
