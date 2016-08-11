#include "mbed.h"
#include "FreeRTOS.h"
#include "task.h"

int main()
{
    DigitalOut led(PA_6);
    
    vTaskStartScheduler();
    while(1)
    {
    	led = 1;
    	wait(1000);
    	led = 0;
    	wait(1000);
    }
}
