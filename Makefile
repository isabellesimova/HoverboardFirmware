######################################
# target
######################################
TARGET = hoverboard


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################

# firmware library path
PERIFLIB_PATH = 

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_adc.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_tim_ex.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_cortex.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_gpio.c \
./src/system_stm32f1xx.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_tim.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_adc_ex.c \
./src/stm32f1xx_it.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_rcc.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_dma.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_uart.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_pwr.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_rcc_ex.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_gpio_ex.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_flash_ex.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_flash.c \
./drivers/stm32f1xx_hal_driver/src/stm32f1xx_hal_iwdg.c \
./src/adc.c \
./src/motor.c \
./src/uart.c \
./src/power.c \
./src/debug.c \
./src/delay.c \
./src/main.c \
./src/stm32f1xx_hal_msp.c  

# ASM sources
ASM_SOURCES =  \
startup_stm32f103xe.s


######################################
# firmware library
######################################
PERIFLIB_SOURCES = 


#######################################
# binaries
#######################################
BINPATH = /usr/local/bin
PREFIX = arm-none-eabi-
CC = $(BINPATH)/$(PREFIX)gcc
AS = $(BINPATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(BINPATH)/$(PREFIX)objcopy
AR = $(BINPATH)/$(PREFIX)ar
SZ = $(BINPATH)/$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# for windows, use:
#######################################
# BINPATH = "C:\Program Files (x86)\GNU Tools Arm Embedded\7 2018-q2-update\bin
# PREFIX = arm-none-eabi-
# CC = $(BINPATH)/$(PREFIX)gcc"
# AS = $(BINPATH)/$(PREFIX)gcc" -x assembler-with-cpp
# CP = $(BINPATH)/$(PREFIX)objcopy"
# AR = $(BINPATH)/$(PREFIX)ar"
# SZ = $(BINPATH)/$(PREFIX)size"
# HEX = $(CP) -O ihex
# BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F103xE


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-I./inc \
-I./drivers/stm32f1xx_hal_driver/inc \
-I./drivers/stm32f1xx_hal_driver/inc/legacy \
-I./drivers/cmsis/device/st/stm32f1xx/inc \
-I./drivers/cmsis/inc


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = stm32f103rctx_flash.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

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
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)

#######################################
# upload
#######################################
upload:
	st-flash write build/hoverboard.bin 0x8000000

#######################################
# unlock
#######################################
unlock:
	openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0" -c "reset halt" -c "exit"

#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
