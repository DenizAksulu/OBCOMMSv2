#include "mbed.h"


int main()
{
    DigitalOut led(PA_5);
    
    while(1)
    {
    	led = 1;
    	wait(1000);
    	led = 0;
    	wait(1000);
    }
}
