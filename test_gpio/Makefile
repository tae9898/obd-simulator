##########################################################################################################################
# Makefile - GPIO 핀 테스트 (PA11/PA12 토글 검증)
# 기존 프로젝트 건드리지 않고 독립 빌드
##########################################################################################################################

TARGET  = test-gpio
BUILDDIR = build

# 상위 프로젝트 경로
PARENT = ..

# C 소스 (테스트 메인 + 최소 HAL만)
C_SOURCES = \
main.c \
$(PARENT)/Core/Src/system_stm32g4xx.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c

ASM_SOURCES = $(PARENT)/startup_stm32g431xx.s

# 툴체인
PREFIX = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
CP      = $(PREFIX)objcopy
SZ      = $(PREFIX)size

# MCU
CPU       = -mcpu=cortex-m4
FPU       = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=hard
MCU       = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

# 매크로
C_DEFS = -DUSE_HAL_DRIVER -DSTM32G431xx

# 헤더 경로
C_INCLUDES = \
-I. \
-I$(PARENT)/Core/Inc \
-I$(PARENT)/Drivers/CMSIS/Device/ST/STM32G4xx/Include \
-I$(PARENT)/Drivers/CMSIS/Include \
-I$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Inc \
-I$(PARENT)/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy

OPT    = -O0 -g3
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fno-common
ASFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall

LDSCRIPT = $(PARENT)/STM32G431RBTX_FLASH.ld
LDFLAGS  = $(MCU) -specs=nano.specs -T$(LDSCRIPT) \
           -Wl,-Map=$(BUILDDIR)/$(TARGET).map,--cref -Wl,--gc-sections

# 오브젝트
C_OBJECTS   = $(addprefix $(BUILDDIR)/, $(notdir $(C_SOURCES:.c=.o)))
ASM_OBJECTS = $(addprefix $(BUILDDIR)/, $(notdir $(ASM_SOURCES:.s=.o)))
OBJECTS     = $(C_OBJECTS) $(ASM_OBJECTS)

all: $(BUILDDIR)/$(TARGET).elf $(BUILDDIR)/$(TARGET).bin $(BUILDDIR)/$(TARGET).hex
	@echo ' '
	@echo '=== GPIO 테스트 빌드 완료 ==='
	@$(SZ) $<

$(BUILDDIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) | $(BUILDDIR)
	@echo '링킹: $@'
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS) -lc -lm -lnosys

$(BUILDDIR)/$(TARGET).bin: $(BUILDDIR)/$(TARGET).elf
	$(CP) -O binary $< $@

$(BUILDDIR)/$(TARGET).hex: $(BUILDDIR)/$(TARGET).elf
	$(CP) -O ihex $< $@

# 컴파일 규칙
$(BUILDDIR)/%.o: %.c | $(BUILDDIR)
	@mkdir -p $(dir $@)
	@echo '컴파일: $<'
	$(CC) -std=gnu11 $(CFLAGS) -c -o $@ $<

$(BUILDDIR)/%.o: $(PARENT)/%.c | $(BUILDDIR)
	@mkdir -p $(dir $@)
	@echo '컴파일: $<'
	$(CC) -std=gnu11 $(CFLAGS) -c -o $@ $<

$(BUILDDIR)/%.o: $(PARENT)/Drivers/STM32G4xx_HAL_Driver/Src/%.c | $(BUILDDIR)
	@mkdir -p $(dir $@)
	@echo '컴파일 (HAL): $<'
	$(CC) -std=gnu11 $(CFLAGS) -c -o $@ $<

$(BUILDDIR)/%.o: $(PARENT)/Core/Src/%.c | $(BUILDDIR)
	@mkdir -p $(dir $@)
	@echo '컴파일 (Core): $<'
	$(CC) -std=gnu11 $(CFLAGS) -c -o $@ $<

$(BUILDDIR)/startup_stm32g431xx.o: $(PARENT)/startup_stm32g431xx.s | $(BUILDDIR)
	@echo '어셈블: $<'
	$(AS) -c $(ASFLAGS) -o $@ $<

$(BUILDDIR):
	@mkdir -p $@

flash: $(BUILDDIR)/$(TARGET).bin
	@echo '플래시 다운로드...'
	st-flash write $< 0x08000000

clean:
	-rm -fR $(BUILDDIR)

.PHONY: all clean flash
