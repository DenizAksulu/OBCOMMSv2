#include "OBCOMMSv2.h"
#include "nanocam.h"

using namespace OBCOMMSv2;

void WDT_TaskCode(void* PARAMETERS);

int main()
{
	//*********************************************************
	// SDRAM initialization************************************
	//*********************************************************
	bool sdram_result = BSP_SDRAM_Init();
	//*********************************************************
	//*********************************************************
	//*********************************************************

	//*********************************************************
	// Create heap regions for FreeRTOS************************
	//*********************************************************
	const HeapRegion_t xHeapRegions[] =
	{
	    { ( uint8_t * ) 0xC0000000, configTOTAL_HEAP_SIZE },
	    { NULL, 0 } /* Terminates the array. */
	};

	vPortDefineHeapRegions( xHeapRegions );
	//*********************************************************
	//*********************************************************
	//*********************************************************

#if DEBUG
	MUTEX_DEBUG = xQueueCreateMutex(queueQUEUE_TYPE_MUTEX); // Initialize MUTEX for debug port
	DebugPort.baud(115200);	// Initialize debug port
#endif

	DBG("OBCOMMSv2 code started with CPU frequency %d Hz.", SystemCoreClock);
	if(sdram_result == SDRAM_OK)
	{
		DBG("SDRAM initialized successfully with refresh rate %2f MHz.", (((uint)REFRESH_COUNT + 20) / 15.6));
	}
	else
	{
		DBG("SDRAM initialization error.")
	}

	DBG("Executing kernel with %d kB heap size and heap address 0x%X.", configTOTAL_HEAP_SIZE, xHeapRegions[0].pucStartAddress);

    xTaskCreate(WDT_TaskCode, "WDTTask", 500, NULL, 5, &WDT_TaskHandle);
    vTaskStartScheduler();
    while(1)
    {
    }
}

void WDT_TaskCode(void* PARAMETERS)
{
	while(1)
	{
		LED_GREEN = 1;
		vTaskDelay(1000);
		LED_GREEN = 0;
		vTaskDelay(1000);
	}
}

void HAL_SDRAM_RefreshErrorCallback(SDRAM_HandleTypeDef *hsdram)
{
	// SDRAM error occured!!!
}
