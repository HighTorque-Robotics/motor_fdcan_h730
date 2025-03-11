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
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "my_fdcan.h"
#include "livelybot_fdcan.h"
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
  * @retvel int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint32_t tick_100ms = 0;
	uint32_t tick_1000ms = 0;
    uint32_t num = 0;
#if  POS_FLAG == 1 // float
    uint32_t tick_1ms = 0;
    float pos = 0;
    float dir = 0.0001;
#define  POS_MAX  0.5f
#define  SET_POS  set_pos_float
#elif POS_FLAG == 2
    uint32_t tick_1ms = 0;
    int32_t pos = 0;
    int32_t dir = 10;
#define  POS_MAX  50000
#define  SET_POS  set_pos_int32
#elif POS_FLAG == 3
    uint32_t tick_1ms = 0;
    int32_t pos = 0;
    int32_t dir = 1;
#define  POS_MAX  5000
#define  SET_POS  set_pos_int16
#endif


#ifdef POS_REZERO
    uint32_t motor_pos_rezero_num = 0;
#endif


#ifdef MOTOR_STOP
    uint32_t motor_stop_num = 0;
#endif


#ifdef MOTOR_BRAKE
    uint32_t motor_brake_num = 0;
#endif
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
	printf("此工程引脚配置适用于高擎主控板 v1.4 及以上版本\r\n");
    while (1)
    {
#if POS_FLAG == 1 || POS_FLAG == 2 || POS_FLAG == 3
        if (HAL_GetTick() - tick_1ms >= 1)
        {
            tick_1ms = HAL_GetTick();

            pos += dir;
            SET_POS(&hfdcan1, 1, pos);

            if (pos <= -POS_MAX || pos >= POS_MAX)
            {
                dir = -dir;
            }
        }
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if (HAL_GetTick() - tick_100ms >= 100)
        {
			// printf("AAA\r\n");
            tick_100ms = HAL_GetTick();

            /* 读取电机状态 */
#if READ_MOTOR_FLAG == 1
            read_motor_state_float(&hfdcan1, 1);
#elif READ_MOTOR_FLAG == 2
            read_motor_state_int32(&hfdcan1, 1);
#elif READ_MOTOR_FLAG == 3
            read_motor_state_int16(&hfdcan1, 1);
#endif

            HAL_Delay(1);

            /* dq 电压模式，0.3v */
            // set_dq_volt_float(&hfdcan1, 1, 0.3);
            // set_dq_volt_int32(&hfdcan1, 1, 300);
            // set_dq_volt_int16(&hfdcan1, 1, 3);


            /* dq 电流模式 */
            // set_dq_current_float(&hfdcan1, 1, 0.4);
            // set_dq_current_int32(&hfdcan1, 1, 400);
            // set_dq_current_int16(&hfdcan1, 1, 4);


            /* 力矩控制 */
            // set_torque_float(&hfdcan1, 1, 0.7);
            // set_torque_int32(&hfdcan1, 1, 700);
            // set_torque_int16(&hfdcan1, 1, 70);


            /* 位置、速度和力矩控制 */
            // set_pos_vel_tqe_float(&hfdcan1, 1, NAN_FLOAT, 0.1, 1);
            // set_pos_vel_tqe_int32(&hfdcan1, 1, NAN_INT32, 10000, 100000);
            // set_pos_vel_tqe_int16(&hfdcan1, 1, NAN_INT16, 400, 10000);


            /* 速度 */
            // set_vel_float(&hfdcan1, 1, 0.1);
            // set_vel_int32(&hfdcan1, 1, 10000);
            set_vel_int16(&hfdcan1, 1, 400);


            /* 位置、速度、力矩、PD控制 */
            // set_pos_vel_tqe_pd_float(&hfdcan1, 1, 0.3, 0.1, 1, 0.1, 0.1);
            // set_pos_vel_tqe_pd_int32(&hfdcan1, 1, 30000, 10000, 10000, 100, 100);
            // set_pos_vel_tqe_pd_int16(&hfdcan1, 1, 3000, 1000, 1000, 1, 1);


            /* 位置、速度、力矩、PD控制 */
            // set_stoppos_vel_tqe_kp_kd_int16(&hfdcan1, 1, 3000, 1000, 0, 10, 10);


            /* 速度、速度限制 */
            // set_vel_velmax_int16(&hfdcan1, 1, 4000, 2000);


            /* 位置、速度、加速度限制（梯形控制） */
            // set_pos_velmax_acc_float(&hfdcan1, 1, 30, 1, 0.1);
            // set_pos_velmax_acc_int32(&hfdcan1, 1, -3000000, 100000, 100000);
            // set_pos_velmax_acc_int16(&hfdcan1, 1, 30000, 1000, 100);


            /* 速度、加速度控制 */
            // set_vel_acc_float(&hfdcan1, 1, 10, 0.1);
            // set_vel_acc_int32(&hfdcan1, 1, 1000000, 10000);
            // set_vel_acc_int16(&hfdcan1, 1, 10000, 100);

            /* 一拖多 位置控制 */
            // set_many_pos_int16(&hfdcan1, 3000, -3000, 5000, 5000);
			
			/* 一拖多 速度控制 */
            // set_many_vel_int16(&hfdcan1, -400, 400, 5000, 5000);

            /* 一拖多 力矩控制 */
            // set_many_tqe_int16(&hfdcan1, 400, 100, 100, 300);

            /* 一拖多 电压控制 */
            // set_many_volt_int16(&hfdcan1, 6, 10, 1, 2);

            /* 一拖多 电流控制 */
            // set_many_current_int16(&hfdcan1, 30, 40, 1, 2);
			
			/* 一拖多 位置、速度、力矩控制 */
            // set_many_pos_vel_tqe_int16(&hfdcan1, -30000, 400, 1000, 10000, 400, 1000);

            /* 一拖多 位置、速度、力矩、PD控制 */
            // set_many_pos_vel_tqe_pd_int16(&hfdcan1, -30000, 400, 1000, 32767, 32767, -10000, 400, 1000, 32767, 32767);

            /* 一拖多 位置、速度、PD控制 */
            // set_many_pos_vel_pd_int16(&hfdcan1, -10000, 400, 32767, 32767, 30000, 400, 32767, 32767);

			/* 一拖多 位置、速度、加速度控制（一拖多 梯形控制） */
            // set_many_pos_vel_acc_int16(&hfdcan1, -30000, 1000, 100, -30000, 4000, 400);

#ifdef POS_REZERO
            if (motor_pos_rezero_num++ > 30)
            {
				set_motor_brake(&hfdcan1, 1);  // 这里是防止电机控制函数没有完全注释掉，实际使用时无需加上这句，保存电机不动即可
				HAL_Delay(100);
                set_pos_rezero(&hfdcan1, 1);
                while(1);
            }
#endif


#ifdef MOTOR_STOP
            if (motor_stop_num++ > 30)
            {
                set_motor_stop(&hfdcan1, 1);
                while(1);
            }
#endif


#ifdef MOTOR_BRAKE
            if (motor_brake_num++ > 30)
            {
                set_motor_brake(&hfdcan1, 1);
                while(1);
            }
#endif
        }


        if (motor_read_flag == 1)
        {
            motor_read_flag = 0;
            if (num++ > 10)
            {
                num = 0;
#if READ_MOTOR_FLAG == 1
				printf("mode:%lf, pos:%lf, vel:%lf, tqe:%lf\r\n", motor_state.motor.mode, motor_state.motor.position, motor_state.motor.velocity, motor_state.motor.torque);
#elif READ_MOTOR_FLAG == 2 || READ_MOTOR_FLAG == 3
				printf("mode:%d, pos:%d, vel:%d, tqe:%d\r\n", motor_state.motor.mode, motor_state.motor.position, motor_state.motor.velocity, motor_state.motor.torque);
#endif
            }
        }
		
		if (HAL_GetTick() - tick_1000ms >= 1000)
		{
			tick_1000ms = HAL_GetTick();
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			printf("AAA\r\n");
		}
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retvel None
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
  * @retvel None
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
  * @retvel None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters velue: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
