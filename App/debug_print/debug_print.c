#include <stdio.h>
#include <stdarg.h>
#include <debug_print.h>
#include <string.h>

#include "usart.h"


static float data[10] = {0};
const uint8_t tail[] = {0x00, 0x00, 0x80, 0x7f};



void debug_print(int num, ...)
{
    if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    {
        return;
    }

    va_list args;
    va_start(args, num);

    for (int i = 0; i < num; i++)
    {
        data[i] = (float)va_arg(args, double);
    }

    data[num] = *(float *)tail;

    HAL_UART_Transmit(&huart1, (uint8_t *)data,
                      (uint16_t)((num + 1) * sizeof(float)), 10);

    va_end(args);
}



// 启用 DWT 计数器（微秒级时间戳）
void DWT_Init(void) 
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) 
	{
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 使能 DWT
    }
    DWT->CYCCNT = 0;                  // 清零计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 使能 CYCCNT
}

// 获取微秒值
uint32_t DWT_GetMicroseconds(void) 
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}


