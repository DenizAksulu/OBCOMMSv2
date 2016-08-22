#include "mbed.h"
#include "FreeRTOS.h"
#include "task.h"

int main()
{
    DigitalOut led(PA_6);

    /*
     *
     * INITIALIZE SDRAM HERE! BEFORE ANY FreeRTOS FUNCTION!!
     *
     */

    vTaskStartScheduler();
    while(1)
    {
    	led = 1;
    	wait(1000);
    	led = 0;
    	wait(1000);
    }
}

void HAL_SDRAM_RefreshErrorCallback(SDRAM_HandleTypeDef *hsdram)
{
	// SDRAM error occured!!!
}
