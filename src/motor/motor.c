#include "motor.h"



/************************************下面为需要修改的部分*******************************************/

static motor_state_s motor_state_port[MOTOR_PORT_NUM][MOTOR_MAX_NUM];


const port_mapping_s port_maping[MOTOR_PORT_NUM] =  // 通道映射表
{
    {
        .port = PORT1,
        .fdcan = &hfdcan1,
        .state = motor_state_port[0],
    },

    {
        .port = PORT2,
        .fdcan = &hfdcan2,
        .state = motor_state_port[1],
    },

    // {
    //     .port = PORT3,
    //     .fdcan = &hfdcan3,
    //     .state = motor_state_port[2],
    // },
};

/*******************************************END***************************************************/


p_motor_state_s motor_get_state_pointer1(FDCAN_HandleTypeDef *fdcanHandle)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (fdcanHandle->Instance == port_maping[i].fdcan->Instance)
        {
            return port_maping[i].state;
        }
    }

    MOTOR_ERR();
    return NULL;
}


p_motor_state_s motor_get_state_pointer2(port_t portx)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (portx == port_maping[i].port)
        {
            return port_maping[i].state;
        }
    }

    MOTOR_ERR();
    return NULL;
}


FDCAN_HandleTypeDef *motor_get_fdcan_pointer(port_t portx)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (portx == port_maping[i].port)
        {
            return port_maping[i].fdcan;
        }
    }

    MOTOR_ERR();
    return NULL;
}



void motor_print_state()
{
    for (uint8_t portx = PORT1; portx < PORT1 + MOTOR_PORT_NUM; portx++)
    {
        for (uint8_t id = 1; id <= MOTOR_MAX_NUM; id++)
        {
            motor_state_s *p_motor_state = motor_get_state(portx, id);
            printf("PORT: %d, ID: %2d, mode: %2d, temp: %2d, fault: %2d, pos: %.3lf, vel: %.3lf, tqe: %.3lf\r\n", portx, id, p_motor_state->mode, p_motor_state->temp,
                   p_motor_state->fault, p_motor_state->position, p_motor_state->velocity, p_motor_state->torque);
        }
        printf("\r\n");
    }
}


void motor_print_version()
{
    for (uint8_t portx = PORT1; portx < PORT1 + MOTOR_PORT_NUM; portx++)
    {
        for (uint8_t id = 1; id <= MOTOR_MAX_NUM; id++)
        {
            const p_version_s p_version = &(motor_get_state(portx, id)->version);

            printf("PORT: %d, ID: %2d, version = %d.%d.%d\r\n", portx, id, p_version->major, p_version->minor, p_version->patch);
        }
        printf("\r\n");
    }
}



/**
 * @brief 获取指定端口和ID的电机状态指针
 * @param portx 指定电机所在的端口，可能的值为 PORT1 或 PORT2
 * @param id 电机 ID
 * @return 返回类型为 `p_motor_state_s` 的指针
 */
p_motor_state_s motor_get_state(port_t portx, uint8_t id)
{
    const uint8_t index = id - 1;

    return &(motor_get_state_pointer2(portx)[index]);
}


/**
 * @brief 解析电机返回信息
 * @param fdcanHandle
 * @param id 电机 ID
 * @param p_data fdcan 帧数据指针
 * @param len fdcan 数据长度
 */
static void motor_process_state(FDCAN_HandleTypeDef *fdcanHandle, const uint8_t id, const uint32_t id_title, const uint8_t *p_data, const uint8_t len)
{
    p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);
    const uint8_t id_index = id - 1;

    switch (p_data[0])
    {
    // ===================== QUERY_MODE_FAULT_POS_VEL_TQE (0x0B) 响应 =====================
    case QUERY_MODE_FLAUT_POS_VEL_TQE:
    {
        // --------- 按数据类型分发 (TINT16/TINT32/TFLOAT) ---------
        switch (id_title)
        {
        case ID_PREFIX_TINT16:
        {
            int16_t pos = 0, vel = 0, tqe = 0;

            my_memcpy((uint8_t *)&pos, p_data + 3, sizeof(int16_t));
            my_memcpy((uint8_t *)&vel, p_data + 5, sizeof(int16_t));
            my_memcpy((uint8_t *)&tqe, p_data + 7, sizeof(int16_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].position  = conv_from_turns(pos_int2float(pos, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel_int2float(vel, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe_int2float(tqe, TINT16);
            break;
        }
        case ID_PREFIX_TINT32:
        {
            int32_t pos = 0, vel = 0, tqe = 0;

            my_memcpy((uint8_t *)&pos, p_data + 3, sizeof(int32_t));
            my_memcpy((uint8_t *)&vel, p_data + 7, sizeof(int32_t));
            my_memcpy((uint8_t *)&tqe, p_data + 11, sizeof(int32_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].position  = conv_from_turns(pos_int2float(pos, TINT32), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel_int2float(vel, TINT32), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe_int2float(tqe, TINT32);
            break;
        }
        case ID_PREFIX_TFLOAT:
        {
            float pos = 0, vel = 0, tqe = 0;

            my_memcpy((uint8_t *)&pos, p_data + 3, sizeof(float));
            my_memcpy((uint8_t *)&vel, p_data + 7, sizeof(float));
            my_memcpy((uint8_t *)&tqe, p_data + 11, sizeof(float));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = (uint8_t)p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].position  = conv_from_turns(pos, MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel, MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe;
            break;
        }
        default:
            break;
        }
        break;
    }

    // ===================== QUERY_MODE_FAULT_TEMP_POS_VEL_TQE (0x0C) 响应 =====================
    case QUERY_MODE_FLAUT_TEMP_POS_VEL_TQE:
    {

        // --------- 按数据类型分发 (TINT16/TINT32/TFLOAT) ---------
        switch (id_title)
        {
        case ID_PREFIX_TINT16:
        {
            int16_t pos = 0, vel = 0, tqe = 0, temp_raw = 0;

            my_memcpy((uint8_t *)&temp_raw, p_data + 3, sizeof(int16_t));
            my_memcpy((uint8_t *)&pos,     p_data + 5, sizeof(int16_t));
            my_memcpy((uint8_t *)&vel,     p_data + 7, sizeof(int16_t));
            my_memcpy((uint8_t *)&tqe,     p_data + 9, sizeof(int16_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].temp      = (int8_t)temp_int2float(temp_raw, TINT16);  // 0.1°C/LSB
            p_motor_state[id_index].position  = conv_from_turns(pos_int2float(pos, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel_int2float(vel, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe_int2float(tqe, TINT16);
            break;
        }
        case ID_PREFIX_TINT32:
        {
            int32_t pos = 0, vel = 0, tqe = 0, temp_raw = 0;

            my_memcpy((uint8_t *)&temp_raw, p_data + 3, sizeof(int32_t));
            my_memcpy((uint8_t *)&pos,     p_data + 7, sizeof(int32_t));
            my_memcpy((uint8_t *)&vel,     p_data + 11, sizeof(int32_t));
            my_memcpy((uint8_t *)&tqe,     p_data + 15, sizeof(int32_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].temp      = (int8_t)temp_int2float(temp_raw, TINT32);  // 0.001°C/LSB
            p_motor_state[id_index].position  = conv_from_turns(pos_int2float(pos, TINT32), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel_int2float(vel, TINT32), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe_int2float(tqe, TINT32);
            break;
        }
        case ID_PREFIX_TFLOAT:
        {
            float pos = 0, vel = 0, tqe = 0, temp_raw = 0;

            my_memcpy((uint8_t *)&temp_raw, p_data + 3, sizeof(float));
            my_memcpy((uint8_t *)&pos,     p_data + 7, sizeof(float));
            my_memcpy((uint8_t *)&vel,     p_data + 11, sizeof(float));
            my_memcpy((uint8_t *)&tqe,     p_data + 15, sizeof(float));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = (uint8_t)p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].temp      = (int8_t)temp_int2float(temp_raw, TFLOAT);  // 1°C/LSB
            p_motor_state[id_index].position  = conv_from_turns(pos, MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel, MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe;
            break;
        }
        default:
            break;
        }
        break;
    }
    // ===================== QUERY_MODE_FLAUT_CD_CQ (0x0D) 响应 =====================
    case QUERY_MODE_FLAUT_CD_CQ:
    {

        // --------- 按数据类型分发 (TINT16/TINT32/TFLOAT) ---------
        switch (id_title)
        {
        case ID_PREFIX_TINT16:
        {
            int16_t i_d = 0, i_q = 0;

            my_memcpy((uint8_t *)&i_d, p_data + 3, sizeof(int16_t));
            my_memcpy((uint8_t *)&i_q, p_data + 5, sizeof(int16_t));

            p_motor_state[id_index].query   = p_data[0];
            p_motor_state[id_index].mode    = (uint8_t)p_data[1];
            p_motor_state[id_index].fault   = p_data[2];
            p_motor_state[id_index].i_d     = cur_int2float(i_d, TINT16);
            p_motor_state[id_index].i_q     = cur_int2float(i_q, TINT16);
            break;
        }
        case ID_PREFIX_TINT32:
        {
            int32_t i_d = 0, i_q = 0;

            my_memcpy((uint8_t *)&i_d, p_data + 3, sizeof(int32_t));
            my_memcpy((uint8_t *)&i_q, p_data + 7, sizeof(int32_t));

            p_motor_state[id_index].query   = p_data[0];
            p_motor_state[id_index].mode    = (uint8_t)p_data[1];
            p_motor_state[id_index].fault   = p_data[2];
            p_motor_state[id_index].i_d     = cur_int2float(i_d, TINT32);
            p_motor_state[id_index].i_q     = cur_int2float(i_q, TINT32);
            break;
        }
        case ID_PREFIX_TFLOAT:
        {
            float i_d = 0, i_q = 0;

            my_memcpy((uint8_t *)&i_d, p_data + 3, sizeof(float));
            my_memcpy((uint8_t *)&i_q, p_data + 7, sizeof(float));

            p_motor_state[id_index].query   = p_data[0];
            p_motor_state[id_index].mode    = (uint8_t)p_data[1];
            p_motor_state[id_index].fault   = p_data[2];
            p_motor_state[id_index].i_d     = i_d;
            p_motor_state[id_index].i_q     = i_q;
            break;
        }
        default:
            break;
        }
        break;
    }
    // ===================== QUERY_FLAUT_POS_VEL_TQE (0x0E) 响应 =====================
    // 返回帧: 查询码(0x0E) | 错误码 | 位置 | 速度 | 力矩, 无模式字段
    // 字段宽度由 CAN ID 类型位决定: TINT16=2B, TINT32/TFLOAT=4B
    case QUERY_FLAUT_POS_VEL_TQE:
    {
        switch (id_title)
        {
        case ID_PREFIX_TINT16:
        {
            int16_t pos = 0, vel = 0, tqe = 0;

            my_memcpy((uint8_t *)&pos, p_data + 2, sizeof(int16_t));
            my_memcpy((uint8_t *)&vel, p_data + 4, sizeof(int16_t));
            my_memcpy((uint8_t *)&tqe, p_data + 6, sizeof(int16_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].fault     = p_data[1];
            p_motor_state[id_index].position  = conv_from_turns(pos_int2float(pos, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel_int2float(vel, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe_int2float(tqe, TINT16);
            break;
        }
        case ID_PREFIX_TINT32:
        {
            int32_t pos = 0, vel = 0, tqe = 0;

            my_memcpy((uint8_t *)&pos, p_data + 2, sizeof(int32_t));
            my_memcpy((uint8_t *)&vel, p_data + 6, sizeof(int32_t));
            my_memcpy((uint8_t *)&tqe, p_data + 10, sizeof(int32_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].fault     = p_data[1];
            p_motor_state[id_index].position  = conv_from_turns(pos_int2float(pos, TINT32), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel_int2float(vel, TINT32), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe_int2float(tqe, TINT32);
            break;
        }
        case ID_PREFIX_TFLOAT:
        {
            float pos = 0, vel = 0, tqe = 0;

            my_memcpy((uint8_t *)&pos, p_data + 2, sizeof(float));
            my_memcpy((uint8_t *)&vel, p_data + 6, sizeof(float));
            my_memcpy((uint8_t *)&tqe, p_data + 10, sizeof(float));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].fault     = p_data[1];
            p_motor_state[id_index].position  = conv_from_turns(pos, MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel, MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe;
            break;
        }
        default:
            break;
        }
        break;
    }
    // ===================== 电机固件版本 (0x04) =====================
    // 返回帧: 04 | patch | minor | major (各1字节)
    case 0x04:
    {
        p_motor_state[id_index].version.major = p_data[3];
        p_motor_state[id_index].version.minor = p_data[2];
        p_motor_state[id_index].version.patch = p_data[1];
        break;
    }
    // ===================== 电机型号查询响应 =====================
    // 注意: 必须在"一拖多"分支之前, 否则会被 id_index < MOTOR_MAX_NUM 拦截
    case 0x07:
    {
        const uint8_t model_len = p_data[1];

        if (model_len > 0 && model_len <= 15 && len >= model_len + 2)
        {
            char model_str[25] = {0};

            // 型号数据为 ASCII 字符直读 (如 0x35='5', 0x5F='_'), 直接复制即可
            for (uint8_t i = 0; i < model_len; i++)
            {
                model_str[i] = (char)p_data[2 + i];
            }

            // 保存电机型号到状态结构体
            my_memcpy((uint8_t *)p_motor_state[id_index].model, (uint8_t *)model_str, sizeof(model_str));
        }
        break;
    }
    // ===================== 配置命令应答帧 (0x03) =====================
    // 应答帧(2字节): 03 | result
    //   result = 0x00 : 配置成功
    //   result != 0x00: 配置失败, result 即失败原因码
    // 注意: 这里是"返回的"2 字节应答帧, 与"发送的"3~8 字节命令帧严格区分
    // 对应发送的系统命令:
    //   周期返回 03 00 05 <查询码> + 4字节微秒
    //   软重启 03 03 01 / 保存设置 03 03 02 / 重设零位 03 03 03 / 改ID 03 03 04 <新ID>
    case 0x03:
    {
        if (len >= 2)
        {
            const uint8_t result = p_data[1];

            //  result == 0 表示成功 (03 00), 非 0 表示失败 (03 XX)
            // motor_config_closed_loop 以"非 0"视为确认成功
            // 成功 → ack = 1(非零); 失败 → ack = 0
            p_motor_state[id_index].ack = (result == 0) ? 1 : 0;

            // 失败时把原因码存入 fault 便于排查; 成功时清零
            p_motor_state[id_index].fault = (result == 0) ? 0 : result;
        }
        break;
    }
    default:
        break;
    }
}




static FDCAN_RxHeaderTypeDef fdcan_rx_header;
static uint8_t fdcan_rdata[64] = {0};

/**
 * @brief 解析所有 CAN 通道 FIFO 中的电机状态数据
 *
 */
void motor_process_state_all()
{
    for (int i = 0; i < MOTOR_PORT_NUM; i++)
    {
        while (HAL_FDCAN_GetRxMessage(port_maping[i].fdcan, FDCAN_RX_FIFO0, &fdcan_rx_header, fdcan_rdata) == HAL_OK)
        {
            if (fdcan_rx_header.DataLength != 0)
            {
                const uint16_t len = get_fdcan_data_size(fdcan_rx_header.DataLength);

                const uint32_t id_title = fdcan_rx_header.Identifier & 0x00030000UL;  // 提取 bits[17:16] 数据类型
                const uint8_t  motor_id = (fdcan_rx_header.Identifier >> 8) & 0x7F;  // 提取 bits[14:8] 主机ID (电机返回ID, 1~127)


                if (motor_id > 0 && motor_id <= MOTOR_MAX_NUM)
                {
                    motor_process_state(port_maping[i].fdcan, motor_id, id_title, fdcan_rdata, len);
                }
            }
        }
    }
}


