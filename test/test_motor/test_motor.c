#include "test_motor.h"
#include "motor_many.h"
#include <stdio.h>

/* test_motor_cycle 中各 mode 对应的展示名称 */
static const char *const mode_label[10] = {
    "0-DQ电压(VOLT)",       "1-DQ电流(CUR)",        "2-位置(POS)",
    "3-速度(VEL)",          "4-力矩(TQE)",          "5-位置+速度",
    "6-位置+速度+力矩限制",   "7-位置+速度+加速度",    "8-MIT运控",
    "9-速度+加速度(VEL_ACC)",
};

/* 各模式多参数组 (3组, 由 param_idx 索引) */
static const float pos_vals[3]       = {0.0f, 1.0f, 2.0f};
static const float vel_vals[3]       = {0.1f, 0.5f, 1.0f};
static const float posvel_pos[3]     = {0.0f, 0.5f, 1.0f};
static const float posvel_vel[3]     = {0.1f, 0.5f, 1.0f};
static const float acc_pos[3]        = {0.0f, 0.5f, 1.0f};
static const float acc_vel[3]        = {0.1f, 0.5f, 1.0f};
static const float acc_acc[3]        = {0.05f, 0.1f, 0.5f};
static const float mit_pos[3]        = {0.0f, 1.0f, 3.0f};
static const float mit_kp[3]         = {10.0f, 20.0f, 30.0f};
static const float mit_kd[3]         = {1.0f, 2.0f, 3.0f};

void test_motor_cycle(port_t portx, data_type_t type, uint8_t id)
{
    static uint8_t  mode      = 0;
    static uint8_t  param_idx = 0;  /* 参数组索引, 每轮mode循环完递增 */
    static uint32_t tick_send  = 0;   /* 指令发送: 100ms (触发电机回传) */
    static uint32_t tick_mode  = 0;   /* 模式维持: 5000ms */

    uint8_t pi = param_idx; /* 当前参数组索引 */

    /* ① 每 100ms 重复下发当前模式指令 → 实时回传状态 */
    if (HAL_GetTick() - tick_send >= 100)
    {
        tick_send = HAL_GetTick();
        switch (mode) {
        case 0: motor_set_dq_vlot(portx, type, id, 1.0);  break;
        case 1: motor_set_dq_current(portx, type, id, 1.0); break;
        case 2: motor_set_pos(portx, type, id, pos_vals[pi]);  break;
        case 3: motor_set_vel(portx, type, id, vel_vals[pi]); break;
        case 4: motor_set_tqe(portx, type, id, 0.5f); break;
        case 5: motor_set_pos_vel(portx, type, id, posvel_pos[pi], posvel_vel[pi]); break;
        case 6: motor_set_pos_vel_MAXtqe(portx, type, id, posvel_pos[pi], posvel_vel[pi], 0.5f); break;
        case 7: motor_set_pos_velmax_acc(portx, type, id, acc_pos[pi], acc_vel[pi], acc_acc[pi]); break;
        case 8: motor_set_pos_vel_tqe_kp_kd_2(portx, type, id, mit_pos[pi], 0, 0, mit_kp[pi], mit_kd[pi]); break;
        case 9: motor_set_vel_acc(portx, type, id, 1, 0.1); break;
        default: return;
        }
    }

    /* ② 每 5s 切换到下一个模式并打印预告 */
    if (tick_mode != 0 && HAL_GetTick() - tick_mode < 5000)
    {
        return;
    } 
    tick_mode = HAL_GetTick();

    mode = (mode + 1) % 10;              /* 先切到新模式 */
    tick_send = 0;                       /* 强制下个调用立刻下发首帧指令 */

    /* mode 循环归零时, 切换到下一组参数 */
    if (mode == 0) {
        param_idx = (param_idx + 1) % 3;
    }

    p_motor_state_s st = motor_get_state(portx, id);
    printf("[单电机循环] → %s (参数组%d) \r\n",
           mode_label[mode], param_idx);
}


void test_motor_control(const uint8_t id)
{
    const uint8_t mode = 3;
    const data_type_t type = TINT16;
    const port_t portx = PORT2;

    switch (mode)
    {
    case 0:
        motor_set_dq_vlot(portx, type, id, 0.0);
        break;
    case 1:
        motor_set_dq_current(portx, type, id, 0.0);
        break;
    case 2:
        motor_set_pos(portx, type, id, 0.0);
        break;
    case 3:
        motor_set_vel(portx, type, id, 0.1f);
        break;
    case 4:
        motor_set_tqe(portx, type, id, 0.0f);
        break;
    case 5:
        motor_set_pos_vel(portx, type, id, 0.0, 0.0);
        break;
    case 6:
        motor_set_pos_vel_MAXtqe(portx, type, id, 0.0, 0.0, 0.0);
        break;
    case 7:
        motor_set_pos_velmax_acc(portx, type, id, 0, 0.0, 0.0);
        break;
    case 8:
        motor_set_pos_vel_tqe_kp_kd_2(portx, type, id, 0, 0, 0, 0, 0);
        break;
    case 9:
        motor_set_vel_acc(portx, type, id, 0.0, 0.0);
        break;
    case 10:
        motor_set_stop(portx, type, id);
        break;
    case 11:
        motor_get_state_send(portx, type, id);
        break;   
    case 12:
        motor_get_version(portx, id);
        break;
    case 13:
        motor_set_brake(portx, type, id);
        break;
    case 14:
        motor_get_model(portx, id);
        break;
    default:
        break;
    }
}

/**
 * @brief  test_motor_connect_test  FDCAN 带宽压测
 * @note   每 1ms 调用一次，轮询 ID 1~MOTOR_MAX_NUM 发送指令帧
 */
void test_motor_connect_test(void)
{
    const uint8_t     mode  = 3;      /* 切换测试不同指令类型 */
    const data_type_t type  = TINT16;
    const port_t      portx = PORT1;
    FDCAN_HandleTypeDef *hfdcan = motor_get_fdcan_pointer(portx);

    /* 轮询所有电机，while 忙等 总线 释放后继续发 */
    for (uint8_t id = 1; id <= MOTOR_MAX_NUM; id++)
    {
        while (!HAL_FDCAN_GetTxFifoFreeLevel(hfdcan))
            ;   /* 忙等直到总线释放 */

        switch (mode)
        {
        case 0:  motor_set_dq_vlot(portx, type, id, 0.0f);           break;
        case 1:  motor_set_dq_current(portx, type, id, 0.0f);        break;
        case 2:  motor_set_vel(portx, type, id, 0.0f);               break;
        case 3:  motor_set_tqe(portx, type, id, 0.0f);               break;
        case 5:  motor_set_pos_vel(portx, type, id, 0.0f, 0.0f);     break;
        case 6:  motor_set_pos_vel_MAXtqe(portx, type, id, 0.0f, 0.0f, 0.0f); break;
        case 7:  motor_set_pos_velmax_acc(portx, type, id, 0.0f, 0.0f, 0.0f);   break;
        case 8:  motor_set_pos_vel_tqe_kp_kd_2(portx, type, id, 0.0f, 0, 0, 0.0f, 0.0f); break;
        case 9:  motor_set_vel_acc(portx, type, id, 0.0f, 0.0f);    break;
        case 10: motor_set_stop(portx, type, id);                   break;
        default: return;
        }
    }
}

