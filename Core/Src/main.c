/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Trace.h"
#include "Motor.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRACE_CENTER_HOLD_SPEED 240
#define TRACE_CURVE_SPEED 200
#define TRACE_CURVE_ERROR 1.5f
#define TRACE_SHARP_CURVE_ERROR 4.0f   // 急弯误差门限，只有线偏到最外侧才降速
#define TRACE_SHARP_CURVE_SPEED 160    // 急弯（180度）降速值
#define TRACE_SEARCH_SPIN_SPEED 170
#define TRACE_SEARCH_COAST_CYCLES 4U
#define TRACE_SEARCH_COAST_FRAMES 20U  // 丢线后先差速滑行100ms，普通弯道可重新找线
#define STARTUP_SETTLE_MS    200U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
static uint8_t startup_done = 0U;
static uint16_t lost_frames = 0U;  // 丢线后持续帧数

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float absf_local(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

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
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // 启动稳定期：让传感器滤波器建立稳态
  {
    uint32_t settle_start = HAL_GetTick();
    while ((HAL_GetTick() - settle_start) < STARTUP_SETTLE_MS) {
      trace_get_error();
      HAL_Delay(10);
    }
    startup_done = 1U;
    pid_reset();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    float error = trace_get_error();
    const TraceState *trace = trace_get_state();
    uint8_t line_lost = trace->line_lost;

    if (line_lost)
    {
      lost_frames++;
      float search_dir = trace_get_search_direction();
      int16_t spin = (search_dir >= 0.0f) ? 1 : -1;

      if (lost_frames <= TRACE_SEARCH_COAST_FRAMES)
      {
        // 阶段0：差速滑行，用丢线前的误差继续转向，普通弯道可重新找线
        float coast_error = trace_get_last_error();
        float coast_abs = coast_error < 0.0f ? -coast_error : coast_error;
        int16_t coast_base = TRACE_CURVE_SPEED;
        float coast_pid = get_pid_output();
        int16_t coast_left  = (int16_t)(coast_base + (int16_t)coast_pid);
        int16_t coast_right = (int16_t)(coast_base - (int16_t)coast_pid);
        int16_t coast_outer = coast_base * 2 / 3;
        if (coast_pid >= 0.0f) {
          coast_left  = clamp_i16(coast_left,  coast_outer, PWM_MAX);
          coast_right = clamp_i16(coast_right, 80, PWM_MAX);
        } else {
          coast_left  = clamp_i16(coast_left,  80, PWM_MAX);
          coast_right = clamp_i16(coast_right, coast_outer, PWM_MAX);
        }
        (void)coast_abs;
        motor_set(coast_left, coast_right);
      }
      else if (lost_frames <= TRACE_SEARCH_COAST_FRAMES + TRACE_SEARCH_COAST_CYCLES * 8U)
      {
        // 阶段1：原地自转找线，内轮反转
        motor_set(spin * TRACE_SEARCH_SPIN_SPEED, -spin * TRACE_SEARCH_SPIN_SPEED);
      }
      else
      {
        // 阶段2：超时加速自转，扩大搜索范围
        int16_t fast_spin = (int16_t)(TRACE_SEARCH_SPIN_SPEED * 3 / 2);
        if (fast_spin > PWM_MAX) fast_spin = PWM_MAX;
        motor_set(spin * fast_spin, -spin * fast_spin);
      }
      HAL_Delay(5);
      continue;
    }

    lost_frames = 0U;

    calc_pid(error, line_lost);

    float abs_error = absf_local(error);

    // 速度曲线：完全由误差驱动，状态标志只在必要时限速
    int16_t base_speed = (int16_t)(MAX_RUN_SPEED - (int16_t)(abs_error * TURN_SLOWDOWN));
    if (base_speed < TRACE_CURVE_SPEED) base_speed = TRACE_CURVE_SPEED;
    if (base_speed > MAX_RUN_SPEED)     base_speed = MAX_RUN_SPEED;

    // 急弯(误差>=3，线偏到外侧)额外降速，防止180度弯冲过头丢线
    if (abs_error >= TRACE_SHARP_CURVE_ERROR)
    {
      if (base_speed > TRACE_SHARP_CURVE_SPEED) base_speed = TRACE_SHARP_CURVE_SPEED;
    }
    // uncertain/wide 才限速（十字路口/宽线）
    else if (trace->uncertain || trace->wide)
    {
      if (base_speed > TRACE_CENTER_HOLD_SPEED) base_speed = TRACE_CENTER_HOLD_SPEED;
    }

    float pid_out = get_pid_output();
    int16_t left  = (int16_t)(base_speed + (int16_t)pid_out);
    int16_t right = (int16_t)(base_speed - (int16_t)pid_out);

    // 外轮下限 = base_speed*2/3，内轮最低 80 克服静摩擦保持缓慢转动
    int16_t outer_min = base_speed * 2 / 3;
    int16_t inner_min = 80;
    if (pid_out >= 0.0f) {
      left  = clamp_i16(left,  outer_min, PWM_MAX);
      right = clamp_i16(right, inner_min, PWM_MAX);
    } else {
      left  = clamp_i16(left,  inner_min, PWM_MAX);
      right = clamp_i16(right, outer_min, PWM_MAX);
    }

    motor_set(left, right);

    HAL_Delay(5);
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

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  // --- 循迹传感器输入 (PA8, PA9) ---
  GPIO_InitStruct.Pin = S5_PIN | S6_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // --- 循迹传感器输入 (PB12-PB15) ---
  GPIO_InitStruct.Pin = S1_PIN | S2_PIN | S3_PIN | S4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // --- 电机方向控制输出 (PB0, PB1, PB10, PB11) ---
  GPIO_InitStruct.Pin = MOTOR_L_IN1_PIN | MOTOR_L_IN2_PIN | 
                        MOTOR_R_IN1_PIN | MOTOR_R_IN2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
