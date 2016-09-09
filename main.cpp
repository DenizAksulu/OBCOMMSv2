#include "OBCOMMSv2.h"

using namespace OBCOMMSv2;

void WDT_TaskCode(void* PARAMETERS);
static void SystemClock_Config (void);

int main()
{
#if DEBUG
	DebugPort.baud(115200);	// initialize debug port
#endif
	DBG("OBCOMMSv2 code started with CPU frequency %d Hz", SystemCoreClock);
    /*
     *
     * INITIALIZE SDRAM HERE! BEFORE ANY FreeRTOS FUNCTION!!
     *
     */

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
		LED_BLUE = 1;
		vTaskDelay(1000);
		LED_BLUE = 0;
		vTaskDelay(1000);
	}
}

void HAL_SDRAM_RefreshErrorCallback(SDRAM_HandleTypeDef *hsdram)
{
	// SDRAM error occured!!!
}


/*
   System Clock Configuration
     System Clock source            = PLL (HSE)
     SYSCLK(Hz)                     = 216000000
     HCLK(Hz)                       = 216000000
     AHB Prescaler                  = 1
     APB1 Prescaler                 = 4
     APB2 Prescaler                 = 2
     HSE Frequency(Hz)              = 25000000
     PLL_M                          = 25
     PLL_N                          = 432
     PLL_P                          = 2
     PLL_Q                          = 9
     VDD(V)                         = 3.3
     Main regulator output voltage  = Scale1 mode
     Flash Latency(WS)              = 7
 */
static void SystemClock_Config (void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	}

	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
	}

	SystemCoreClockUpdate();
	///SystemCoreClock = 216000000;
}

