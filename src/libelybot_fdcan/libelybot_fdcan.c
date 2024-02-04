#include "libelybot_fdcan.h"
#include "my_fdcan.h"
#include "string.h"


FDCAN_RxHeaderTypeDef fdcan_rx_header1;
uint8_t fdcan1_rdata[64] = {0};

motor_state_t motor_state;
uint8_t motor_read_flag = 0;



//static void print_data(uint8_t *data, uint16_t len)
//{
//    for (int i = 0; i < len; i++)
//    {
//        printf("%d\r\n", &data[i]);
//    }
//    printf("\r\n\r\n");
//}


/**
 * @brief 电压控制 float
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param volt 电压，例：0.3 -> 0.3v
 */
void set_dq_volt_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float volt)
{
    //                          dq电压模式   2个16位      d                        q                       占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x0E, 0x1a, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(float *)&cmd[9] = volt;

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电压控制 int32
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param volt 电压，单位：0.001V
 */
void set_dq_volt_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t volt)
{
    //                   		dq电压模式   2个32位      d                        q                       占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x0A, 0x1a, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(int32_t *)&cmd[9] = volt;

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电压控制 int16
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param volt 电压，单位：0.1V
 */
void set_dq_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t volt)
{
    //                   		dq电压模式   2个16位      d           q           占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x06, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(int16_t *)&cmd[7] = volt;

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电流控制 float
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param volt 电流，例：0.3 -> 0.3A
 */
void set_dq_current_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float current)
{
    //                   		dq电流模式   2个32位      q电流       			  d电流       			  占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(float *)&cmd[5] = current;

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电流控制 int32
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param volt 电流，单位：0.001A
 */
void set_dq_current_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t current)
{
    //                   		dq电流模式   2个32位      q电流       			  d电流       			   占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x0A, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(int32_t *)&cmd[5] = current;

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电流控制 int16
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param volt 电流，单位：0.1A
 */
void set_dq_current_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t current)
{
    //                   		dq电流模式   2个16位      q电流       d电流       占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x06, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(int16_t *)&cmd[5] = current;

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 力矩控制 float
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param torque 力矩，单位：xxx NM
 */
void set_torque_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float torque)
{
    //                     		  位置模式    float  6个        位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0c, 0x06, 0x20, 0x00, 0x00,
                            // 			速度                 	力矩
                            0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc,
                            //			kp                	    kd
                            0xcc, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            //			最大力矩                占位（fdcan）
                            0x00, 0x00, 0x00, 0x00, 0xc0, 0x7f, 0x50, 0x50
                           };

    *(int32_t *)&cmd[14] = *(int32_t *)&torque;
    // memcpy(&cmd[14], &torque, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 力矩控制 int32
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 * @param torque 力矩，单位：xxx NM
 */
void set_torque_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t torque)
{
    //                     位置模式    		 int32  6个    	    位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x08, 0x06, 0x20, 0x00, 0x00,
                            //          速度                    力矩
                            0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            //          kp  					kd
                            0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            //          最大力矩   		 	    占位（fdcan）
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x50, 0x50
                           };

    *(int32_t *)&cmd[14] = *(int32_t *)&torque;
    // memcpy(&cmd[14], &torque, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 力矩控制 int16
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 * @param torque 力矩，单位：xxx NM
 */
void set_torque_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t torque)
{
    //                     		位置模式     int16   6个        位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x04, 0x06, 0x20, 0x00, 0x80,
                            //速度      力矩        kp          kd          最大力矩    占位（fdcan）
                            0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x50, 0x50
                           };

    *(int16_t *)&cmd[10] = *(int16_t *)&torque;
    // memcpy(&cmd[10], &torque, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 电机位置-速度-前馈力矩(最大力矩)控制，float型
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 * @param val 速度：单位 1 转/秒，如 val = 0.5 表示 0.5 转/秒
 * @param torque 最大力矩：单位：1NM，如 torque = 1.1 表示最大力矩为 1.1NM
 */
void set_pos_vel_tqe_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float val, float torque)
{
    //                           位置模式     int32       位置                    速度                    			  力矩                    停止位置                占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0e, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0xcd, 0xcc, 0xcc, 0x3d, 0x0e, 0x25, 0x00, 0x00, 0x80, 0x3f, 0x9a, 0x99, 0x00, 0x00, 0x50};

    // *(int32_t *)&cmd[9] = *(int32_t *)&val;
    // *(int32_t *)&cmd[15] = *(int32_t *)&torque;
    // *(int32_t *)&cmd[19] = *(int32_t *)&pos;

    memcpy(&cmd[9], &val, sizeof(float));
    memcpy(&cmd[15], &torque, sizeof(float));
    memcpy(&cmd[19], &pos, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置-速度-前馈力矩(最大力矩)控制，int32型
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置。
 * @param val 速度：单位 0.00001 转/秒，如 val = 50000 表示 0.5 转/秒
 * @param torque 最大力矩：单位：0.00001 NM，如 torque = 110000 表示最大力矩为 1.1NM
 */
void set_pos_vel_tqe_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t val, int32_t torque)
{
    //                           位置模式     int32       位置                    速度                    			  力矩                    停止位置                占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0a, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50};

    // *(int32_t *)&cmd[9] =  val;
    // *(int32_t *)&cmd[13] = torque;
    // *(int32_t *)&cmd[17] = pos;

    memcpy(&cmd[9], &val, sizeof(int32_t));
    memcpy(&cmd[15], &torque, sizeof(int32_t));
    memcpy(&cmd[19], &pos, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置-速度-前馈力矩(最大力矩)控制，int16型
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param val 速度：单位 0.00025 转/秒，如 val = 400 表示 0.1 转/秒
 * @param torque 最大力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM
 */
void set_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t val, int16_t torque)
{
    //                           位置模式   2个int16      位置        速度		 2个int16	  力矩		  占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x06, 0x25, 0x00, 0x00, 0x50, 0x50};

    // *(int16_t *)&cmd[7] =  val;
    // *(int16_t *)&cmd[11] = torque;
    // *(int16_t *)&cmd[13] = pos;

    memcpy(&cmd[7], &val, sizeof(int16_t));
    memcpy(&cmd[11], &torque, sizeof(int16_t));
    memcpy(&cmd[13], &pos, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置控制 float
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 */
void set_pos_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos)
{
    //                           位置模式   1个float      位置                    占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[5], &pos, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置控制 int32
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置。
 */
void set_pos_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos)
{
    //                           位置模式   1个int32      位置                    占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x09, 0x20, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[5], &pos, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置控制 int16
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 */
void set_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos)
{
    //                          位置模式     1个int16     位置
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x05, 0x20, 0x00, 0x00};

    memcpy(&cmd[5], &pos, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度控制 float
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param val 速度：单位 1 转/秒，如 val = 0.1 -> 0.1 转/秒
 */
void set_val_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float val)
{
    //							  位置模式     2个float 	位置					速度					占位（fdcan）
    static uint8_t cmd[16] = {0x01, 0x00, 0x0A, 0x0E, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[9], &val, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度控制 int32
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param val 速度：单位 0.00001 转/秒，如 val = 50000 表示 0.5 转/秒
 */
void set_val_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t val)
{
    //							  位置模式     2个int32 	位置					速度					占位（fdcan）
    static uint8_t cmd[16] = {0x01, 0x00, 0x0A, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[9], &val, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度控制 int16
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param val 速度：单位 0.00025 转/秒，如 val = 400 表示 0.1 转/秒
 */
void set_val_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val)
{
    //							位置模式     2个int16 	  位置		  速度		  占位（fdcan）
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[7], &val, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置、速度、最大力矩、Kp、Kd控制 float 类型
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 * @param val 速度：单位 1 转/秒，如 val = 0.5 表示 0.5 转/秒
 * @param tqe 最大力矩：单位：1NM，如 torque = 1.1 表示最大力矩为 1.1NM
 * @param rkp 电机 Kp 比例，电机真实 Kp = 电机内部Kp * rkp
 * @param rkd 电机 Kd 比例，电机真实 Kd = 电机内部Kd * rkd
 */
void set_pos_val_tqe_pd_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float val, float tqe, float rkp, float rkd)
{
    //                           位置模式    2个float     位置（NAN）
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0e, 0x20, 0x00, 0x00, 0xc0, 0x7f,
                            //速度                  float  4个
                            0xcd, 0xcc, 0xcc, 0x3d, 0x0c, 0x04, 0x23,
                            //rKp                   rKd                     最大力矩                停止位置
                            0xcd, 0xcc, 0x4c, 0x3d, 0xcd, 0xcc, 0x4c, 0x3d, 0x00, 0x00, 0x80, 0x3f, 0x9a, 0x99, 0x99, 0x3e
                           };

    memcpy(&cmd[9], &val, sizeof(float));
    memcpy(&cmd[16], &rkp, sizeof(float));
    memcpy(&cmd[20], &rkd, sizeof(float));
    memcpy(&cmd[24], &tqe, sizeof(float));
    memcpy(&cmd[28], &pos, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置、速度、最大力矩、Kp、Kd控制 int32
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置
 * @param val 速度：单位 0.00001 转/秒，如 val = 50000 表示 0.5 转/秒
 * @param tqe 最大力矩：单位：0.00001 NM，如 torque = 110000 表示最大力矩为 1.1NM
 * @param rkp 电机 Kp 比例，电机真实 Kp = 电机内部Kp * rkp
 * @param rkd 电机 Kd 比例，电机真实 Kd = 电机内部Kd * rkd
 */
void set_pos_val_tqe_pd_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t val, int32_t tqe, float rkp, float rkd)
{
    //                            位置模式
    static uint8_t cmd[48] = {0x01, 0x00, 0x0a, 0x0A, 0x20,
                              //位置（NAN）           速度
                              0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x0E, 0x23,
                              //rKp                   rKd
                              0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0xc0, 0x7f, 0x0A, 0x25,
                              //最大力矩              停止位置
                              0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80,
                              //占位（fdcan）
                              0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50
                             };

    memcpy(&cmd[9], &val, sizeof(int32_t));
    memcpy(&cmd[15], &rkp, sizeof(float));
    memcpy(&cmd[19], &rkd, sizeof(float));
    memcpy(&cmd[25], &tqe, sizeof(int32_t));
    memcpy(&cmd[29], &pos, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, 48);
}


/**
 * @brief 电机位置、速度、最大力矩、Kp、Kd控制 int16
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param val 速度：单位 0.00025 转/秒，如 val = 400 表示 0.1 转/秒
 * @param tqe 最大力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM
 * @param rkp 电机 Kp 比例，电机真实 Kp = 电机内部Kp * rkp
 * @param rkd 电机 Kd 比例，电机真实 Kd = 电机内部Kd * rkd
 */
void set_pos_val_tqe_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t val, int16_t tqe, float rkp, float rkd)
{
    //                           位置模式
    static uint8_t cmd[32] = {0x01, 0x00, 0x0a, 0x06, 0x20,
                              //位置（NAN）速度
                              0x00, 0x80, 0x00, 0x80, 0x0e, 0x23,
                              //rKp       			  rKd
                              0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0xc0, 0x7f, 0x06, 0x25,
                              //最大力矩  停止位置
                              0x00, 0x80, 0x00, 0x80
                             };

    memcpy(&cmd[7], &val, sizeof(int16_t));
    memcpy(&cmd[11], &rkp, sizeof(float));
    memcpy(&cmd[15], &rkd, sizeof(float));
    memcpy(&cmd[21], &tqe, sizeof(int16_t));
    memcpy(&cmd[23], &pos, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、速度限幅控制（如果 val > val_max，则用 val_max） int16
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param val 速度：单位 0.00025 转/秒，如 val = 400 表示 0.1 转/秒
 * @param val_max 速度限幅：单位 0.00025 转/秒，如 val = 400 表示 0.1 转/秒
 */
void set_val_valmax_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val, int16_t val_max)
{
    //							位置模式				  位置        速度		  			  速度限制
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x05, 0x28, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[11], &val_max, sizeof(int16_t));
    memcpy(&cmd[7], &val, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置、速度限幅控制 float
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 * @param vel_max 速度限制，单位 1 转/秒，如 val = 0.5 表示 0.5 转/秒
 */
void set_pos_valmax_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float vel_max)
{
    // 							位置模式				  位置		  						  速度限制
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x80, 0x0D, 0x29, 0x00, 0x00, 0x00, 0x00};

    memcpy(&cmd[5], &pos, sizeof(float));
    memcpy(&cmd[11], &vel_max, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置、速度限幅控制 int32
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置
 * @param val_max 速度限制：单位 0.00001 转/秒，如 val = 50000 表示 0.5 转/秒
 */
void set_pos_valmax_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t vel_max)
{
    // 							位置模式				  位置		  						  速度限制
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x09, 0x20, 0x00, 0x00, 0x00, 0x80, 0x09, 0x29, 0x00, 0x00, 0x00, 0x00};

    memcpy(&cmd[5], &pos, sizeof(int32_t));
    memcpy(&cmd[11], &vel_max, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置、速度限幅控制 int16
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置
 * @param vel_max 速度：单位 0.00025 转/秒，如 val = 400 表示 0.1 转/秒
 */
void set_pos_valmax_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t vel_max)
{
    // 							位置模式				  位置		  速度限制
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x05, 0x20, 0x00, 0x80, 0x05, 0x29, 0x50, 0x50};

    memcpy(&cmd[5], &pos, sizeof(int16_t));
    memcpy(&cmd[9], &vel_max, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、加速度控制 float
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param val 速度，单位： 1 转/秒，如 val = 0.5 表示 0.5 转/秒
 * @param acc 加速度，单位：1 转/秒^2
 */
void set_val_acc_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float val, float acc)
{
    //							位置模式				  位置					  速度								  加速度
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0E, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x0D, 0x29, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[9], &val, sizeof(float));
    memcpy(&cmd[15], &acc, sizeof(float));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、加速度控制 int32
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param val 速度：单位 0.00001 转/秒，如 val = 50000 表示 0.5 转/秒
 * @param acc 加速度：单位 0.00001 转/秒^2，如 val = 50000 表示 0.5 转/秒^2
 */
void set_val_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t val, int32_t acc)
{
    //							位置模式				  位置					  速度								  加速度
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x09, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50, 0x50};

    memcpy(&cmd[9], &val, sizeof(int32_t));
    memcpy(&cmd[15], &acc, sizeof(int32_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、加速度控制 int16
 * @param fdcanHandle &hfdcanx
 * @param motor 电机ID
 * @param val 速度：单位 0.00025 转/秒，如 val = 400 表示 0.1 转/秒
 * @param acc 加速度：单位 0.00025 转/秒^2，如 val = 400 表示 0.1 转/秒^2
 */
void set_val_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val, int16_t acc)
{
    //							位置模式				  位置		  速度					  加速度
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x05, 0x29, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[7], &val, sizeof(int16_t));
    memcpy(&cmd[11], &acc, sizeof(int16_t));

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 重设电机零位
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 */
void set_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    static uint8_t cmd[] = {0x40, 0x01, 0x15, 0x64, 0x20, 0x63, 0x66, 0x67, 0x2d, 0x73, 0x65, 0x74, 0x2d, 0x6f, 0x75, 0x74, 0x70, 0x75, 0x74, 0x20, 0x30, 0x2e, 0x30, 0x0a};

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));

    HAL_Delay(1000);

    set_conf_write(fdcanHandle, motor);
}


/**
 * @brief 保存电机设置
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 */
void set_conf_write(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    static uint8_t cmd[] = {0x40, 0x01, 0x0B, 0x63, 0x6F, 0x6E, 0x66, 0x20, 0x77, 0x72, 0x69, 0x74, 0x65, 0x0A, 0x50, 0x50};

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机停止，注意：需让电机停止后再重置零位，否则无效
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 */
void set_motor_stop(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x00};

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 */
void set_motor_brake(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x0f};

    fdcan_send(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 float，状态、位置、速度、转矩
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 */
void read_motor_state_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    const uint8_t cmd[] = {0x1C, 0x04, 0x00};

    fdcan_send(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 int32，状态、位置、速度、转矩
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 */
void read_motor_state_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    const uint8_t cmd[] = {0x18, 0x04, 0x00};

    fdcan_send(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 int16，状态、位置、速度、转矩
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 */
void read_motor_state_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    const uint8_t cmd[] = {0x14, 0x04, 0x00};

    fdcan_send(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint8_t len = 0;
    if(hfdcan->Instance == FDCAN1)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_rx_header1, fdcan1_rdata);
        if (fdcan_rx_header1.DataLength != 0)
        {

            len = get_fdcan_data_size(fdcan_rx_header1.DataLength);
            motor_state.motor.id = fdcan_rx_header1.Identifier;  // 获取电机 id
#if READ_MOTOR_FLAG == 1 || READ_MOTOR_FLAG == 2 || READ_MOTOR_FLAG == 3
            memcpy(&motor_state.data[0], &fdcan1_rdata[3], len - 3);  // 获取电机状态数据
#endif
            motor_read_flag = 1;
        }
    }
}



