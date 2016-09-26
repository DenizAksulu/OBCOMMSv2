#include "OBCOMMSv2.h"

using namespace OBCOMMSv2;

void WDT_TaskCode(void* PARAMETERS);
static void SystemClock_Config (void);

int main()
{
	//SystemClock_Config();
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


void SystemClock_Config(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct;
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;

	  __HAL_RCC_PWR_CLK_ENABLE();

	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = 25;
	  RCC_OscInitStruct.PLL.PLLN = 432;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = 2;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
		  LED_RED = 1;
	  }

	  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	  {
		  LED_RED = 1;
	  }

	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	  {
		  LED_RED = 1;
	  }

	  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	  /* SysTick_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  SystemCoreClockUpdate();
}
