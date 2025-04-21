#include "motor_config.h"


uint8_t motor_config_closed_loop(void (*action)(FDCAN_HandleTypeDef*, uint8_t), FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);
    const uint8_t id_index = id - 1;
    uint16_t t = 0;
    const uint16_t t_max = 10;

    p_motor_state[id_index].ack = 0;
    for (int i = 0; i < 10; i++)
    {
        set_pos_rezero(fdcanHandle, id);
        HAL_Delay(100);
        if (p_motor_state[id_index].ack != 0)
        {
            break;
        }

        if (++t > t_max)
        {
            return 1;
        }
    }

    return 0;
}



uint8_t motor_pos_reset(FDCAN_HandleTypeDef *fdcanHandle, const uint8_t id)
{
    set_motor_reset(fdcanHandle, id);
    set_motor_reset(fdcanHandle, id);
    set_motor_reset(fdcanHandle, id);
    HAL_Delay(100);

    if (motor_config_closed_loop(set_pos_rezero, fdcanHandle, id) != 0)
    {
        return 1;
    }

    if (motor_config_closed_loop(set_conf_write, fdcanHandle, id) != 0)
    {
        return 2;
    }

    set_motor_reset(fdcanHandle, id);
    set_motor_reset(fdcanHandle, id);
    set_motor_reset(fdcanHandle, id);
    HAL_Delay(100);

    printf("重置零位成功！！！\r\n");
    return 0;
}

