######################################
# Makefile by CubeMX2Makefile.py
######################################

######################################
# target
######################################
TARGET = stm32f031f4

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Os -flto

#######################################
# pathes
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
C_SOURCES = \
  Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_crc_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_smartcard_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tsc.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2s.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dac_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dac.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_smbus.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_comp.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_wwdg.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_can.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_iwdg.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_spi_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_smartcard.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_irda.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_usart.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_spi.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rtc_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rtc.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_crc.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cec.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.c \
  Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c \
  Src/stm32f0xx_it.c \
  Src/stm32f0xx_hal_msp.c \
  Src/main.c  
ASM_SOURCES = \
  Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f031x6.s \

#######################################
# binaries
#######################################
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
AR = arm-none-eabi-ar
SZ = arm-none-eabi-size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# macros for gcc
AS_DEFS =
C_DEFS = -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F031x6
# includes for gcc
AS_INCLUDES =
C_INCLUDES = -IInc
C_INCLUDES += -IDrivers/CMSIS/Device/ST/STM32F0xx/Include
C_INCLUDES += -IDrivers/CMSIS/Include
C_INCLUDES += -IDrivers/CMSIS/RTOS/Template
C_INCLUDES += -IDrivers/CMSIS/DSP_Lib/Examples/arm_signal_converge_example/ARM
C_INCLUDES += -IDrivers/CMSIS/DSP_Lib/Examples/arm_linear_interp_example/ARM
C_INCLUDES += -IDrivers/CMSIS/DSP_Lib/Examples/arm_matrix_example/ARM
C_INCLUDES += -IDrivers/CMSIS/DSP_Lib/Examples/arm_graphic_equalizer_example/ARM
C_INCLUDES += -IDrivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC
C_INCLUDES += -IDrivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/ARM
C_INCLUDES += -IDrivers/CMSIS/DSP_Lib/Examples/arm_fir_example/ARM
C_INCLUDES += -IDrivers/STM32F0xx_HAL_Driver/Inc
C_INCLUDES += -IDrivers/STM32F0xx_HAL_Driver/Inc/Legacy
# compile gcc flags
ASFLAGS = -mthumb -mcpu=cortex-m0 $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = -mthumb -mcpu=cortex-m0 $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2 -O0
endif
# Generate dependency information
CFLAGS += -MD -MP -MF .dep/$(@F).d

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F031F4Px_FLASH.ld
# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = -mthumb -mcpu=cortex-m0 -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir -p $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

.PHONY: clean all

# *** EOF ***
