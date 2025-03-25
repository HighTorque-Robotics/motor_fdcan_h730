#include "motor.h"


motor_state_s motor_state[MOTOR_MAX_NUM] =
{
    {
        .id = 1,
        .model = M5047_36,
    },
};


void motor_print_state()
{
    for (uint8_t i = 0; i < MOTOR_MAX_NUM; i++)
    {
        //        printf("id:%2d, mode:%2d, fault:%2d, pos:%.3lf, vel:%.3lf, tqe:%.3lf\r\n", motor_state[i].id, motor_state[i].mode, motor_state[i].fault,
        //               motor_state[i].position, motor_state[i].velocity, motor_state[i].torque);

        printf("%d, %d, %d, %.3lf, %.3lf, %.3lf\r\n", motor_state[i].id, motor_state[i].mode, motor_state[i].fault,
               motor_state[i].position, motor_state[i].velocity, motor_state[i].torque);
    }
}



static void motor_process_state(const uint8_t id, const uint8_t *p_data, const uint8_t len)
{
    const uint8_t id_index = id - 1;
    if (p_data[0] == 0x24 && p_data[1] == 0x04 && p_data[2] == 0x00  // TINT16 ½âÎö
            && p_data[11] == 0x21 && p_data[12] == 0x0F)
    {
        int16_t pos = 0;
        int16_t vel = 0;
        int16_t tqe = 0;

        my_memcpy((uint8_t *)&pos, p_data + 5, sizeof(int16_t));
        my_memcpy((uint8_t *)&vel, p_data + 7, sizeof(int16_t));
        my_memcpy((uint8_t *)&tqe, p_data + 9, sizeof(int16_t));

        motor_state[id_index].mode = p_data[3];
        motor_state[id_index].position = pos_int2float(pos, TINT16);
        motor_state[id_index].velocity = vel_int2float(vel, TINT16);
        const float tqe_temp = tqe_int2float(tqe, TINT16);
        motor_state[id_index].torque = tqe_restore(tqe_temp, motor_state[id_index].model);
        motor_state[id_index].fault = (uint8_t)p_data[13];
    }
    else if (p_data[0] == 0x28 && p_data[1] == 0x04 && p_data[2] == 0x00  // TINT32 ½âÎö
             && p_data[19] == 0x21 && p_data[20] == 0x0F)
    {
        int32_t pos = 0;
        int32_t vel = 0;
        int32_t tqe = 0;

        my_memcpy((uint8_t *)&pos, p_data + 7, sizeof(int32_t));
        my_memcpy((uint8_t *)&vel, p_data + 11, sizeof(int32_t));
        my_memcpy((uint8_t *)&tqe, p_data + 15, sizeof(int32_t));

        motor_state[id_index].mode = p_data[3];
        motor_state[id_index].position = pos_int2float(pos, TINT32);
        motor_state[id_index].velocity = vel_int2float(vel, TINT32);
        const float tqe_temp = tqe_int2float(tqe, TINT32);
        motor_state[id_index].torque = tqe_restore(tqe_temp, motor_state[id_index].model);
        motor_state[id_index].fault = (uint8_t)p_data[21];
    }
    else if (p_data[0] == 0x2C && p_data[1] == 0x04 && p_data[2] == 0x00  // TFLOAT ½âÎö
             && p_data[19] == 0x21 && p_data[20] == 0x0F)
    {
        float pos = 0;
        float vel = 0;
        float tqe = 0;

        my_memcpy((uint8_t *)&pos, p_data + 7, sizeof(float));
        my_memcpy((uint8_t *)&vel, p_data + 11, sizeof(float));
        my_memcpy((uint8_t *)&tqe, p_data + 15, sizeof(float));

        motor_state[id_index].mode = p_data[3];
        motor_state[id_index].position = pos_int2float(pos, TFLOAT);
        motor_state[id_index].velocity = vel_int2float(vel, TFLOAT);
        const float tqe_temp = tqe_int2float(tqe, TFLOAT);
        motor_state[id_index].torque = tqe_restore(tqe_temp, motor_state[id_index].model);
        motor_state[id_index].fault = (uint8_t)p_data[21];
    }
}


static FDCAN_RxHeaderTypeDef fdcan_rx_header;
static uint8_t fdcan_rdata[64] = {0};
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan->Instance == FDCAN1 || hfdcan->Instance == FDCAN2 || hfdcan->Instance == FDCAN3)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_rx_header, fdcan_rdata);
        if (fdcan_rx_header.DataLength != 0)
        {
            const uint16_t len = get_fdcan_data_size(fdcan_rx_header.DataLength);

            motor_process_state(fdcan_rx_header.Identifier >> 8, fdcan_rdata, len);
        }
    }
}
