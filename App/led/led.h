#ifndef _LED_H
#define _LED_H


#include "main.h"


#define  LED_ERR_FLAG   // 启用 MOTOR_ERR (convert.h)，错误时调用 led_toggle_err


void led_toggle(void);
void led_toggle_err(void);


#endif
