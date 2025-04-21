#ifndef _MOTOR_H
#define _MOTOR_H


#include "fdcan.h"
#include "led.h"
#include "my_fdcan.h"
#include "convert.h"
#include "livelybot_fdcan.h"
#include "motor_many.h"


#define  MOTOR_MAX_NUM  2



typedef struct
{
    float position;
    float velocity;
    float torque;
    uint8_t mode;
    uint8_t fault;
    uint8_t ack;
    const uint8_t id;
    const motor_type_t model;  // µç»úÐÍºÅ
} motor_state_s, *p_motor_state_s;


extern many_data_s many_data_port1;
extern many_data_s many_data_port2;


void motor_print_state(void);
p_many_data_s motor_get_many_pointer(port_t portx);
FDCAN_HandleTypeDef *motor_get_fdcan_pointer(port_t portx);
p_motor_state_s motor_get_state_pointer1(FDCAN_HandleTypeDef *fdcanHandle);
p_motor_state_s motor_get_state_pointer2(port_t portx);
uint8_t motor_get_model1(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
uint8_t motor_get_model2(port_t portx, uint8_t id);


#endif
