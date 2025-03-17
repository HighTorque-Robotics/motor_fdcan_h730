#include "test_motor.h"



void test_motor_control(const uint8_t id)
{
    const uint8_t mode = 3;
    const data_type_t type = TFLOAT;

    switch (mode)
    {
    case 0:
        motor_set_dq_vlot(&hfdcan1, type, id, 1);
        break;
    case 1:
        motor_set_dq_current(&hfdcan1, type, id, 0.2);
        break;
    case 2:
        motor_set_pos(&hfdcan1, type, id, 0.1);
        break;
    case 3:
        motor_set_vel(&hfdcan1, type, id, 0.1f);
        break;
    case 4:
        motor_set_tqe(&hfdcan1, type, id, 0.1, M5047_36);
        break;
    case 5:
        motor_set_pos_vel(&hfdcan1, type, id, 0.1, 0.1);
        break;
    case 6:
        motor_set_pos_vel_MAXtqe(&hfdcan1, type, id, 0.1, 0.1, 0.1, M5047_36);
        break;
    default:
        break;
    }

    motor_set_state(&hfdcan1, TINT16, id);
}

