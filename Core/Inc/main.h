/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
  FREQ_SWING,
  FREQ_INCREASE,
  FREQ_PULSE,
}scen_mode_e;

typedef struct
{
  uint32_t period_sec;          // длительность сценария
  uint8_t  dir;                 // 1 -увеличивать  0- уменьшать частоту
  uint32_t f0;                  // текущая частота
  uint32_t fmin;                
  uint32_t fmax;
  uint32_t step_freq_Hz;        // на сколько меняется частота
  uint32_t step_ms;             // через какое время менять частоту
  scen_mode_e  mode;
}scenario_t;

typedef struct 
{
  uint8_t       indx_scen;     
  uint32_t      time_start;
  uint32_t      time_stop;
  uint8_t       freq_div;
}scenic_ctrl_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ARR_MIN_INDX 0
#define ARR_MAX_INDX 1
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
uint8_t _stepFreq(void);
void stepFreq(scenario_t *_scen);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
