#include "test_motor_many.h"


many_data_s many_data_port1;


void test_time_out(int16_t t_ms)
{
    for (uint8_t id = 1; id <= MANY_MOTOR_SIZE; id++)
    {
        motor_many_time_out(&many_data_port1, id, t_ms);
    }

    motor_many_send(&hfdcan1, &many_data_port1);
    motor_many_send(&hfdcan1, &many_data_port1);
    motor_many_send(&hfdcan1, &many_data_port1);
}


void test_motor_many()
{
    const uint8_t mode = 8;

    for (uint8_t id = 1; id <= MANY_MOTOR_SIZE; id++)
    {
        switch (mode)
        {
        case 0:
            motor_many_dq_volt(&many_data_port1, id, 1);
            break;
        case 1:
            motor_many_dq_current(&many_data_port1, id, 1);
            break;
        case 2:
            motor_many_pos(&many_data_port1, id, 1);
            break;
        case 3:
            motor_many_vel(&many_data_port1, id, 0.1);
            break;
        case 4:
            motor_many_tqe(&many_data_port1, id, 1);
            break;
        case 5:
            motor_many_pos_vel(&many_data_port1, id, 1, 0.1);
            break;
        case 6:
			motor_many_pos_vel_MAXtqe(&many_data_port1, id, 1, 0.1, NAN_FLOAT);
            break;
        case 7:
            motor_many_pos_vel_acc(&many_data_port1, id, 1, 1, 0.2);
            break;
        case 8:
            motor_many_pos_vel_tqe_kp_kd(&many_data_port1, id, 1, 0.1, 0, 1, 1);
            break;
        case 9:
            motor_many_pos_vel_tqe_kp_ki_kd(&many_data_port1, id, 1, 0.1, 0, 1, 0, 1);
            break;
        default:
            break;
        }
    }

    motor_many_send(&hfdcan1, &many_data_port1);
}
