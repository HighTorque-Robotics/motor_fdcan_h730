#ifndef _DEBUG_PEINT_H
#define _DEBUG_PEINT_H
#include <stdint.h>


void debug_print(int num, ...);

void DWT_Init(void);
uint32_t DWT_GetMicroseconds(void);


#endif
