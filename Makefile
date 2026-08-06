##########################################################################################################################
# Makefile - STM32G431RB Nucleo (phase0-obd-simulator)
# CubeMX compatible structure, can build independently without CubeMX
##########################################################################################################################

# ============================================
# Target
# ============================================
TARGET = phase0-obd-simulator

# ============================================
# Build directory
# ============================================
BUILD_DIR = build

# ============================================
# C source files
# ============================================
C_SOURCES = \
Core/Src/main.c \
Core/Src/obd2_simulator.c \
Core/Src/fdcan_config.c \
Core/Src/fdcan_loopback_test.c \
Core/Src/uart_debug.c \
Core/Src/iso_tp.c \
Core/Src/uds_service.c \
Core/Src/diag_session.c \
Core/Src/rs485.c \
Core/Src/ota_flash.c \
Core/Src/stm32g4xx_hal_msp.c \
Core/Src/stm32g4xx_it.c \
Core/Src/system_stm32g4xx.c

# ============================================
# FreeRTOS kernel source files
# ============================================
FREERTOS_DIR = Middlewares/FreeRTOS-Kernel

FREERTOS_SOURCES = \
$(FREERTOS_DIR)/tasks.c \
$(FREERTOS_DIR)/queue.c \
$(FREERTOS_DIR)/list.c \
$(FREERTOS_DIR)/timers.c \
$(FREERTOS_DIR)/event_groups.c \
$(FREERTOS_DIR)/stream_buffer.c \
$(FREERTOS_DIR)/portable/GCC/ARM_CM4F/port.c \
$(FREERTOS_DIR)/portable/MemMang/heap_4.c

# ============================================
# HAL driver source files (only required modules)
# ============================================
HAL_SOURCES = \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_iwdg.c

# ============================================
# Assembly source files
# ============================================
ASM_SOURCES = startup_stm32g431xx.s

# ============================================
# Toolchain
# ============================================
PREFIX = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
CP      = $(PREFIX)objcopy
SZ      = $(PREFIX)size
OBJDUMP = $(PREFIX)objdump

# ============================================
# MCU flags
# ============================================
CPU       = -mcpu=cortex-m4
FPU       = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=hard
MCU       = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

# ============================================
# C macro definitions
# ============================================
C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32G431xx

# ============================================
# Header include paths
# ============================================
C_INCLUDES = \
-ICore/Inc \
-IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/STM32G4xx_HAL_Driver/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
-I$(FREERTOS_DIR)/include \
-I$(FREERTOS_DIR)/portable/GCC/ARM_CM4F

# ============================================
# Optimization options
# ============================================
OPT = -O0 -g3

# ============================================
# Compile flags
# ============================================
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fno-common

# ============================================
# Assembly flags
# ============================================
ASFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall

# ============================================
# Linker script
# ============================================
LDSCRIPT = STM32G431RBTX_FLASH.ld

# ============================================
# Libraries
# ============================================
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# ============================================
# Default target: all
# ============================================
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex
	@echo ' '
	@echo 'Build complete:'
	@$(SZ) $<

# ============================================
# Object file list
# ============================================
C_OBJECTS   = $(addprefix $(BUILD_DIR)/, $(C_SOURCES:.c=.o))
HAL_OBJECTS = $(addprefix $(BUILD_DIR)/, $(HAL_SOURCES:.c=.o))
FREERTOS_OBJECTS = $(addprefix $(BUILD_DIR)/, $(FREERTOS_SOURCES:.c=.o))
ASM_OBJECTS = $(addprefix $(BUILD_DIR)/, $(ASM_SOURCES:.s=.o))
OBJECTS     = $(C_OBJECTS) $(HAL_OBJECTS) $(FREERTOS_OBJECTS) $(ASM_OBJECTS)

# ============================================
# Dependency file list
# ============================================
DEPS = $(OBJECTS:.o=.d)

# ============================================
# ELF linking
# ============================================
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) | $(BUILD_DIR)
	@echo 'Linking: $@'
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS) -lc -lm -lnosys

# ============================================
# Binary generation (.bin, .hex)
# ============================================
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	@echo 'Generating BIN: $@'
	$(CP) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	@echo 'Generating HEX: $@'
	$(CP) -O ihex $< $@

# ============================================
# C source compile rule (Core/Src)
# ============================================
$(BUILD_DIR)/Core/Src/%.o: Core/Src/%.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo 'Compiling: $<'
	$(CC) -std=gnu11 $(CFLAGS) -MMD -MP -MF $(BUILD_DIR)/Core/Src/$*.d -c -o $@ $<

# ============================================
# HAL driver compile rule
# ============================================
$(BUILD_DIR)/Drivers/STM32G4xx_HAL_Driver/Src/%.o: Drivers/STM32G4xx_HAL_Driver/Src/%.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo 'Compiling (HAL): $<'
	$(CC) -std=gnu11 $(CFLAGS) -MMD -MP -MF $(BUILD_DIR)/Drivers/STM32G4xx_HAL_Driver/Src/$*.d -c -o $@ $<

# ============================================
# FreeRTOS compile rule
# ============================================
$(BUILD_DIR)/Middlewares/%.o: Middlewares/%.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo 'Compiling (FreeRTOS): $<'
	$(CC) -std=gnu11 $(CFLAGS) -MMD -MP -c -o $@ $<

# ============================================
# Assembly compile rule
# ============================================
$(BUILD_DIR)/startup_stm32g431xx.o: startup_stm32g431xx.s | $(BUILD_DIR)
	@echo 'Assembling: $<'
	$(AS) -c $(ASFLAGS) -o $@ $<

# ============================================
# Build directory creation
# ============================================
$(BUILD_DIR):
	@mkdir -p $@

# ============================================
# Include dependency files (if they exist)
# ============================================
-include $(DEPS)

# ============================================
# clean target
# ============================================
clean:
	@echo 'Removing build artifacts...'
	-rm -fR $(BUILD_DIR)

# ============================================
# flash target (using st-flash)
# ============================================
flash: $(BUILD_DIR)/$(TARGET).bin
	@echo 'Flashing...'
	st-flash write $< 0x08000000

# ============================================
# Debug info output
# ============================================
size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $<

disasm: $(BUILD_DIR)/$(TARGET).elf
	$(OBJDUMP) -d $< > $(BUILD_DIR)/$(TARGET).asm

# ============================================
# Helper targets
# ============================================
.PHONY: all clean flash size disasm
