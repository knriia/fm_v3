/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "startup_task.h"
#include "diagnostic_task.h"
#include "task_context.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

static osThreadId_t startup_task_handle;
static const osThreadAttr_t startup_task_attr = {
  .name = "StartupTask",
  .stack_size = STARTUP_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t) osPriorityNormal,
};

static osThreadId_t diagnostic_task_handle;
static const osThreadAttr_t diagnostic_task_attr = {
  .name = "DiagnosticTask",
  .stack_size = DIAGNOSTIC_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t) osPriorityNormal,
};

static osEventFlagsId_t lwip_ready_flags;
static NetworkTaskContext diagnostic_task_context;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void MX_FREERTOS_Init(void);

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void MX_FREERTOS_Init(void)
{
  lwip_ready_flags = osEventFlagsNew(NULL);
  if (lwip_ready_flags == NULL)
  {
    Error_Handler();
  }

  startup_task_handle = osThreadNew(StartStartupTask, &lwip_ready_flags, &startup_task_attr);
  if (startup_task_handle == NULL)
  {
    Error_Handler();
  }

  diagnostic_task_context.lwip_flags = lwip_ready_flags;
  diagnostic_task_handle = osThreadNew(DiagnosticTask, &diagnostic_task_context, &diagnostic_task_attr);
  if (diagnostic_task_handle == NULL)
  {
    Error_Handler();
  }
}

/* USER CODE END Application */

