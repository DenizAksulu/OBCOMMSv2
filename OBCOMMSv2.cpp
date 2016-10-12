/*
 * OBCOMMSv2.cpp
 *
 *  Created on: 7 Eyl 2016
 *      Author: Mehmet Deniz Aksulu
 */
#include "OBCOMMSv2.h"


namespace OBCOMMSv2
{
	DigitalOut LED_GREEN(PI_1);
    TaskHandle_t WDT_TaskHandle = NULL;
#if DEBUG
		Serial DebugPort(PA_9, PB_7, "DebugPort");
#endif
	SemaphoreHandle_t MUTEX_DEBUG;
}
