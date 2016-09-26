/*
 * OBCOMMSv2.cpp
 *
 *  Created on: 7 Eyl 2016
 *      Author: Mehmet Deniz Aksulu
 */
#include "OBCOMMSv2.h"


namespace OBCOMMSv2
{
	DigitalOut LED_BLUE(PB_7);
	DigitalOut LED_RED(PB_14);
    TaskHandle_t WDT_TaskHandle = NULL;
#if DEBUG
		Serial DebugPort(PD_8, PD_9, "DebugPort");
#endif
	SemaphoreHandle_t MUTEX_DEBUG = xQueueCreateMutex(queueQUEUE_TYPE_MUTEX);
}
