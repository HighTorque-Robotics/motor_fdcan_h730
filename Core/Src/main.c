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
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "my_fdcan.h"
#include "motor_control.h"
#include "motor_config.h"
#include "motor.h"
#include "debug_print.h"

#include "test_motor.h"
#include "test_motor_many.h"

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
    uint32_t tick_ctrl  = 0;   /* 1kHz MIT 控制 */
    uint32_t tick_vofa  = 0;   /* 5ms (200Hz) VOFA 发送 */
    uint32_t tick_print = 0;   /* 500ms 终端打印 */

    /* 正弦波 MIT 参数 */
    const float amplitude = 0.5f;      /* 幅值 0.5 圈 */
    const float freq_hz   = 0.25f;     /* 频率 0.25 Hz (4s 一周期) */
    const float dt        = 0.001f;    /* 控制周期 1ms */
    float time = 0.0f;                 /* 时间累加器 (秒) */
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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  /* USER CODE BEGIN 2 */
    fdcan_filter_init(&hfdcan1);
    fdcan_filter_init(&hfdcan2);
    fdcan_filter_init(&hfdcan3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    printf("此工程引脚配置适用于高擎主控板 v1.6 及以上版本\r\n");
    printf("例程版本号："MOTOR_SDK_VERSION"\r\n");

    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        /* ---- 1kHz: MIT 正弦波位置控制 ---- */
        if (HAL_GetTick() - tick_ctrl >= 2)
        {
            tick_ctrl = HAL_GetTick();

            /* 正弦波目标位置 (圈, MOTOR_DATA_TYPE_FLAG=TURNS) */
            float pos_target = amplitude * sinf(MY_2PI * freq_hz * time);
            time += dt;
            test_motor_many();
            //test_motor_many_cycle();
            /* MIT 模式: pos=正弦波, vel=0, tqe=0, KP=100, KD=3 */
            // motor_set_pos_vel_tqe_kp_kd_2(PORT2, TINT16, 1,
            //                               pos_target, 0, 0, 100, 30);
        }

        // /* ---- 5ms (200Hz): VOFA+ JustFloat 波形 (DMA 发送) ---- */
        // if (HAL_GetTick() - tick_vofa >= 5)
        // {
        //     tick_vofa = HAL_GetTick();

        //     motor_state_s *p_state = motor_get_state(PORT2, 1);

        //     /* CH1=目标位置(圈), CH2=实际位置(圈), CH3=速度(圈/s), CH4=力矩(Nm)
        //      * CH5=模式,          CH6=温度(°C),    CH7=故障码,     CH8=时间(s) */
        //     debug_print(8,
        //         amplitude * sinf(MY_2PI * freq_hz * time),  /* CH1 */
        //         p_state->position,                           /* CH2 */
        //         p_state->velocity,                           /* CH3 */
        //         p_state->torque,                             /* CH4 */
        //         (double)p_state->mode,                       /* CH5 */
        //         (double)p_state->temp,                       /* CH6 */
        //         (double)p_state->fault,                      /* CH7 */
        //         (double)time);                               /* CH8 */
        // }

        /* ---- 持续解析电机返回的 FDCAN 数据 ---- */
        motor_process_state_all();

        /* ---- 500ms: 终端打印 + LED ---- */
        if (HAL_GetTick() - tick_print >= 500)
        {
            tick_print = HAL_GetTick();
            led_toggle();
            motor_print_state();
        }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
       ex: printf("Wrong parameters velue: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
