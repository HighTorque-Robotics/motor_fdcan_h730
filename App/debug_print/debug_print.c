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

    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data,
                      (uint16_t)((num + 1) * sizeof(float)));

    va_end(args);
}

// 示例代码

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