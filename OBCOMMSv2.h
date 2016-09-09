/*
 * OBCOMMSv2.h
 *
 *  Created on: 7 Eyl 2016
 *      Author: Mehmet Deniz Aksulu
 */

#ifndef OBCOMMSV2_H_
#define OBCOMMSV2_H_

#include "mbed.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define DEBUG 1 // Debug mode enabled
#if DEBUG
	#define DBG(...) \
			if(xSemaphoreTake(MUTEX_DEBUG, portMAX_DELAY) == pdTRUE) \
			{ \
				time_t seconds = time(NULL);\
				char buffer[32]; \
    			strftime(buffer, 32, "%Y %b %d    %H:%M:%S", localtime(&seconds)); \
				DebugPort.printf("\n %s | %30s:%-4d | ", buffer, __FILE__, __LINE__); \
				DebugPort.printf(__VA_ARGS__); \
				xSemaphoreGive(MUTEX_DEBUG); \
			}
#else
#define DBG(...)
#endif

namespace OBCOMMSv2
{
	extern DigitalOut LED_BLUE;
    extern TaskHandle_t WDT_TaskHandle;
    extern SemaphoreHandle_t MUTEX_DEBUG;
    extern Serial DebugPort;

}



#endif /* OBCOMMSV2_H_ */
