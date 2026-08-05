#include "test_motor_many.h"
#include <stdio.h>

/* test_motor_many_cycle 中各 mode 对应的展示名称 */
static const char *const many_mode_label[9] = {
    "0-一拖多DQ电压",            "1-一拖多DQ电流",            "2-一拖多位置",
    "3-一拖多速度",              "4-一拖多力矩",              "5-一拖多位置+速度+力矩限制",
    "6-一拖多位置+速度+加速度",    "7-一拖多MIT运控",          "8-一拖多速度+加速度",
};

void test_motor_many_cycle(void)
{
    static uint8_t  mode      = 0;
    static uint32_t tick_send = 0;   /* 指令发送: 100ms (触发电机回传) */
    static uint32_t tick_mode = 0;   /* 模式维持: 5000ms */

    /* ① 每 100ms 重复下发当前模式指令 → 实时回传状态 */
    if (HAL_GetTick() - tick_send >= 100)
    {
        tick_send = HAL_GetTick();
        for (uint8_t id = 1; id <= MANY_MOTOR_SIZE; id++) {
            switch (mode) {
            case 0: motor_many_dq_volt(PORT1, id, 1);                  break;
            case 1: motor_many_dq_current(PORT1, id, 1);               break;
            case 2: motor_many_pos(PORT1, id, 0);                      break;
            case 3: motor_many_vel(PORT1, id, 0.2);                    break;
            case 4: motor_many_tqe(PORT1, id, 0.5f);                   break;
            case 5: motor_many_pos_vel_MAXtqe(PORT1, id, 3, 0.1, 0.5);  break;
            case 6: motor_many_pos_vel_acc(PORT1, id, 0, 1, 0.2);       break;
            case 7: motor_many_pos_vel_tqe_kp_kd_2(PORT1, id, 3, 0, 0, 5, 1); break;
            case 8: motor_many_vel_acc(PORT1, id, 0.5, 0.1);           break;
            default: return;
            }
        }
        motor_many_send(PORT1, MANY_GET_MODE_FLAUT_POS_VEL_TQE);
    }

    /* ② 每 5s 切换到下一个模式并打印预告 */
    if (tick_mode != 0 && HAL_GetTick() - tick_mode < 5000)
        return;
    tick_mode = HAL_GetTick();

    mode = (mode + 1) % 9;              /* 先切到新模式 */
    tick_send = 0;                      /* 强制下个调用立刻下发首帧指令 */

    p_motor_state_s st = motor_get_state(PORT1, 1);
    printf("[一拖多循环] → %s  (id1 回传 mode=%2d/0x%02X)\r\n",
           many_mode_label[mode], st->mode, st->mode);
}

void test_time_out(int16_t t_ms)
{
    for (uint8_t id = 1; id <= MANY_MOTOR_SIZE; id++)
    {
        motor_many_time_out(PORT1, id, t_ms);
    }

    motor_many_send(PORT1, MANY_GET_MODE_FLAUT_POS_VEL_TQE);
    motor_many_send(PORT1, MANY_GET_MODE_FLAUT_POS_VEL_TQE);
    motor_many_send(PORT1, MANY_GET_MODE_FLAUT_POS_VEL_TQE);
}


void test_motor_many()
{
    const uint8_t mode = 3;

    for (uint8_t id = 1; id <= MANY_MOTOR_SIZE; id++)
    {
        switch (mode)
        {
        case 0:
            motor_many_dq_volt(PORT1, id, 0);
            break;
        case 1:
            motor_many_dq_current(PORT1, id, 0);
            break;
        case 2:
            motor_many_pos(PORT1, id, 0);
            break;
        case 3:
            motor_many_vel(PORT1, id, -0.1);
            break;
        case 4:
            motor_many_tqe(PORT1, id, 0.0f);
            break;
        case 5:
            motor_many_pos_vel_MAXtqe(PORT1, id, 0, 0.0, 0.0);
            break;
        case 6:
            motor_many_pos_vel_acc(PORT1, id, 0, 0, 0.0);
            break;
        case 7:
            motor_many_pos_vel_tqe_kp_kd_2(PORT1, id, 0, 0, 0, 0, 0);
            break;
        case 8:
            motor_many_vel_acc(PORT1, id, 0.0, 0.0);
            break;
        default:
            break;
        }
    }

    motor_many_send(PORT1, MANY_GET_MODE_FLAUT_POS_VEL_TQE);
}
