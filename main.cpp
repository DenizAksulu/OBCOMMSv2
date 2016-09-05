#include "mbed.h"
#include "FreeRTOS.h"
#include "task.h"

void WDT_TaskCode(void* PARAMETERS);


DigitalOut led(PB_7);
int main()
{
	SystemCoreClock = 216000000;
	SystemCoreClockUpdate();

    /*
     *
     * INITIALIZE SDRAM HERE! BEFORE ANY FreeRTOS FUNCTION!!
     *
     */

    TaskHandle_t WDT_TaskHandle = NULL;
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
		led = 1;
		vTaskDelay(10000);
		led = 0;
		vTaskDelay(10000);
	}
}

void HAL_SDRAM_RefreshErrorCallback(SDRAM_HandleTypeDef *hsdram)
{
	// SDRAM error occured!!!
}
