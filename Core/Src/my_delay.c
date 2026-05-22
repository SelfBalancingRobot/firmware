/*
 * my_delay.c
 *
 *  Created on: May 14, 2026
 *      Author: ozker
 */
#include "my_delay.h"

#ifdef RTOS
#include "cmsis_os.h"
#endif

void myDelay(uint32_t ms){
#ifdef RTOS
	osDelay(ms);
#else
	HAL_Delay(ms);
#endif
}
