#ifndef _MOTOR_H
#define _MOTOR_H


#include "my_fdcan.h"
#include "convert.h"
#include "livelybot_fdcan.h"


#define  MOTOR_MAX_NUM  1



typedef struct
{
    float position;
    float velocity;
    float torque;
    uint8_t mode;
    uint8_t fault;
    const uint8_t id;
    const motor_type_t model;  // µç»úÐÍºÅ
} motor_state_s;



extern motor_state_s motor_state[MOTOR_MAX_NUM];


void motor_print_state(void);


#endif
