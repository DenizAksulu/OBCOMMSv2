GCC_BIN = 
PROJECT = OBCOMMSv2
OBJECTS = ./FreeRTOS/Source/portable/MemMang/heap_4.o ./FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.o ./FreeRTOS/Source/timers.o ./FreeRTOS/Source/tasks.o ./FreeRTOS/Source/queue.o ./FreeRTOS/Source/list.o ./FreeRTOS/Source/list.o ./FreeRTOS/Source/event_groups.o  ./FreeRTOS/Source/croutine.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/TARGET_DISCO_F746NG/TOOLCHAIN_GCC_ARM/startup_stm32f746xx.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_adc.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_adc_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_can.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_cec.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_cortex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_crc.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_crc_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_cryp.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_cryp_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_dac.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_dac_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_dcmi.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_dcmi_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_dma.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_dma2d.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_dma_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_eth.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_flash.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_flash_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_gpio.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_hash.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_hash_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_hcd.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_i2c.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_i2c_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_i2s.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_irda.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_iwdg.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_lptim.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_ltdc.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_nand.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_nor.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_pcd.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_pcd_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_pwr.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_pwr_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_qspi.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_rcc.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_rcc_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_rng.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_rtc.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_rtc_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_sai.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_sai_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_sd.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_sdram.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_smartcard.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_smartcard_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_spdifrx.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_spi.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_sram.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_tim.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_tim_ex.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_uart.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_usart.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_hal_wwdg.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_ll_fmc.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_ll_sdmmc.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/stm32f7xx_ll_usb.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/TARGET_DISCO_F746NG/cmsis_nvic.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/TARGET_DISCO_F746NG/hal_tick.o ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/TARGET_DISCO_F746NG/system_stm32f7xx.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/analogin_api.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/analogout_api.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/gpio_api.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/gpio_irq_api.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/i2c_api.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/mbed_overrides.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/pinmap.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/port_api.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/pwmout_api.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/rtc_api.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/serial_api.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/sleep.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/spi_api.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/us_ticker.o ./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/TARGET_DISCO_F746NG/PeripheralPins.o ./mbed-src/common/assert.o ./mbed-src/common/board.o ./mbed-src/common/error.o ./mbed-src/common/gpio.o ./mbed-src/common/lp_ticker_api.o ./mbed-src/common/mbed_interface.o ./mbed-src/common/pinmap_common.o ./mbed-src/common/rtc_time.o ./mbed-src/common/semihost_api.o ./mbed-src/common/ticker_api.o ./mbed-src/common/us_ticker_api.o ./mbed-src/common/wait_api.o ./main.o ./mbed-src/common/BusIn.o ./mbed-src/common/BusInOut.o ./mbed-src/common/BusOut.o ./mbed-src/common/CAN.o ./mbed-src/common/CallChain.o ./mbed-src/common/Ethernet.o ./mbed-src/common/FileBase.o ./mbed-src/common/FileLike.o ./mbed-src/common/FilePath.o ./mbed-src/common/FileSystemLike.o ./mbed-src/common/I2C.o ./mbed-src/common/I2CSlave.o ./mbed-src/common/InterruptIn.o ./mbed-src/common/InterruptManager.o ./mbed-src/common/LocalFileSystem.o ./mbed-src/common/RawSerial.o ./mbed-src/common/SPI.o ./mbed-src/common/SPISlave.o ./mbed-src/common/Serial.o ./mbed-src/common/SerialBase.o ./mbed-src/common/Stream.o ./mbed-src/common/Ticker.o ./mbed-src/common/Timeout.o ./mbed-src/common/Timer.o ./mbed-src/common/TimerEvent.o ./mbed-src/common/retarget.o 
SYS_OBJECTS = 
INCLUDE_PATHS = -I./FreeRTOS -I./FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I./FreeRTOS/Source/include -I. -I./mbed-src -I./mbed-src/api -I./mbed-src/hal -I./mbed-src/targets -I./mbed-src/targets/cmsis -I./mbed-src/targets/cmsis/TARGET_STM -I./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7 -I./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/TARGET_DISCO_F746NG -I./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/TARGET_DISCO_F746NG/TOOLCHAIN_GCC_ARM -I./mbed-src/targets/hal -I./mbed-src/targets/hal/TARGET_STM -I./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7 -I./mbed-src/targets/hal/TARGET_STM/TARGET_STM32F7/TARGET_DISCO_F746NG -I./mbed-src/common 
LIBRARY_PATHS = 
LIBRARIES = 
LINKER_SCRIPT = ./mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F7/TARGET_DISCO_F746NG/TOOLCHAIN_GCC_ARM/STM32F746NG.ld

############################################################################### 
AS      = $(GCC_BIN)arm-none-eabi-as
CC      = $(GCC_BIN)arm-none-eabi-gcc
CPP     = $(GCC_BIN)arm-none-eabi-g++
LD      = $(GCC_BIN)arm-none-eabi-gcc
OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy
OBJDUMP = $(GCC_BIN)arm-none-eabi-objdump
SIZE    = $(GCC_BIN)arm-none-eabi-size 

ifeq ($(HARDFP),1)
	FLOAT_ABI = hard
else
	FLOAT_ABI = softfp
endif


CPU = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=$(FLOAT_ABI) 
CC_FLAGS = -c -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=softfp -Os -std=gnu99 -include mbed_config.h -MMD -MP
CPPC_FLAGS = -c -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=softfp -Os -std=gnu++98 -fno-rtti -Wvla -include mbed_config.h -MMD -MP
ASM_FLAGS = -x assembler-with-cpp -c -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=softfp -Os
CC_SYMBOLS = -DTARGET_STM32F746NG -D__MBED__=1 -DDEVICE_I2CSLAVE=1 -D__FPU_PRESENT=1 -DDEVICE_PORTINOUT=1 -D__MBED_CMSIS_RTOS_CM -DTARGET_DISCO_F746NG -DTARGET_STM32F7 -DTOOLCHAIN_object -D__CMSIS_RTOS -DTOOLCHAIN_GCC -DDEVICE_CAN=1 -DARM_MATH_CM7 -DTARGET_CORTEX_M -DTARGET_LIKE_CORTEX_M7 -DDEVICE_ANALOGOUT=1 -DTARGET_UVISOR_UNSUPPORTED -DTARGET_M7 -DDEVICE_PWMOUT=1 -DDEVICE_INTERRUPTIN=1 -DDEVICE_I2C=1 -DDEVICE_PORTOUT=1 -DDEVICE_STDIO_MESSAGES=1 -D__CORTEX_M7 -DTARGET_STM32F746 -DTARGET_LIKE_MBED -DDEVICE_PORTIN=1 -DTARGET_RELEASE -DTARGET_STM -DMBED_BUILD_TIMESTAMP=1470899297.53 -DTARGET_RTOS_M4_M7 -DDEVICE_SLEEP=1 -DTOOLCHAIN_GCC_ARM -DDEVICE_SPI=1 -DDEVICE_SPISLAVE=1 -DDEVICE_ANALOGIN=1 -DDEVICE_SERIAL=1 -DDEVICE_RTC=1 

LD_FLAGS = $(CPU) -Wl,--gc-sections --specs=nano.specs -u _printf_float -u _scanf_float -Wl,--wrap,main -Wl,-Map=$(PROJECT).map,--cref
LD_SYS_LIBS = -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys


ifeq ($(DEBUG), 1)
  CC_FLAGS += -DDEBUG -Og
else
  CC_FLAGS += -DNDEBUG -Os
endif

.PHONY: all clean lst size

all: $(PROJECT).bin $(PROJECT).hex size


clean:
	rm -f $(PROJECT).bin $(PROJECT).elf $(PROJECT).hex $(PROJECT).map $(PROJECT).lst $(OBJECTS) $(DEPS)


.asm.o:
	$(CC) $(CPU) -c -x assembler-with-cpp -o $@ $<
.s.o:
	$(CC) $(CPU) -c -x assembler-with-cpp -o $@ $<
.S.o:
	$(CC) $(CPU) -c -x assembler-with-cpp -o $@ $<

.c.o:
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	$(CPP) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu++98 -fno-rtti $(INCLUDE_PATHS) -o $@ $<


$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS)
	$(LD) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $^ $(LIBRARIES) $(LD_SYS_LIBS) $(LIBRARIES) $(LD_SYS_LIBS)


$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

$(PROJECT).hex: $(PROJECT).elf
	@$(OBJCOPY) -O ihex $< $@

$(PROJECT).lst: $(PROJECT).elf
	@$(OBJDUMP) -Sdh $< > $@

lst: $(PROJECT).lst

size: $(PROJECT).elf
	$(SIZE) $(PROJECT).elf

DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)
