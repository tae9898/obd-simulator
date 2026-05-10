##########################################################################################################################
# Makefile - STM32G431RB Nucleo (phase0-obd-simulator)
# CubeMX 호환 구조, CubeMX 없이도 독립 빌드 가능
##########################################################################################################################

# ============================================
# 타겟
# ============================================
TARGET = phase0-obd-simulator

# ============================================
# 빌드 디렉토리
# ============================================
BUILD_DIR = build

# ============================================
# C 소스 파일
# ============================================
C_SOURCES = \
Core/Src/main.c \
Core/Src/obd2_simulator.c \
Core/Src/fdcan_config.c \
Core/Src/uart_debug.c \
Core/Src/stm32g4xx_hal_msp.c \
Core/Src/stm32g4xx_it.c \
Core/Src/system_stm32g4xx.c

# ============================================
# HAL 드라이버 소스 파일 (필요한 모듈만 포함)
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
# 어셈블리 소스 파일
# ============================================
ASM_SOURCES = startup_stm32g431xx.s

# ============================================
# 툴체인
# ============================================
PREFIX = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
CP      = $(PREFIX)objcopy
SZ      = $(PREFIX)size
OBJDUMP = $(PREFIX)objdump

# ============================================
# MCU 플래그
# ============================================
CPU       = -mcpu=cortex-m4
FPU       = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=hard
MCU       = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

# ============================================
# C 매크로 정의
# ============================================
C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32G431xx

# ============================================
# 헤더 포함 경로
# ============================================
C_INCLUDES = \
-ICore/Inc \
-IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/STM32G4xx_HAL_Driver/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy

# ============================================
# 최적화 옵션
# ============================================
OPT = -O0 -g3

# ============================================
# 컴파일 플래그
# ============================================
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fno-common

# ============================================
# 어셈블리 플래그
# ============================================
ASFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall

# ============================================
# 링커 스크립트
# ============================================
LDSCRIPT = STM32G431RBTX_FLASH.ld

# ============================================
# 라이브러리
# ============================================
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# ============================================
# 기본 타겟: all
# ============================================
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex
	@echo ' '
	@echo '빌드 완료:'
	@$(SZ) $<

# ============================================
# 오브젝트 파일 목록 생성
# ============================================
C_OBJECTS   = $(addprefix $(BUILD_DIR)/, $(C_SOURCES:.c=.o))
HAL_OBJECTS = $(addprefix $(BUILD_DIR)/, $(HAL_SOURCES:.c=.o))
ASM_OBJECTS = $(addprefix $(BUILD_DIR)/, $(ASM_SOURCES:.s=.o))
OBJECTS     = $(C_OBJECTS) $(HAL_OBJECTS) $(ASM_OBJECTS)

# ============================================
# 의존성 파일 목록
# ============================================
DEPS = $(OBJECTS:.o=.d)

# ============================================
# ELF 링킹
# ============================================
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) | $(BUILD_DIR)
	@echo '링킹 중: $@'
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS) -lc -lm -lnosys

# ============================================
# 바이너리 생성 (.bin, .hex)
# ============================================
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	@echo 'BIN 생성 중: $@'
	$(CP) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	@echo 'HEX 생성 중: $@'
	$(CP) -O ihex $< $@

# ============================================
# C 소스 컴파일 규칙 (Core/Src)
# ============================================
$(BUILD_DIR)/Core/Src/%.o: Core/Src/%.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo '컴파일 중: $<'
	$(CC) -std=gnu11 $(CFLAGS) -MMD -MP -MF $(BUILD_DIR)/Core/Src/$*.d -c -o $@ $<

# ============================================
# HAL 드라이버 컴파일 규칙
# ============================================
$(BUILD_DIR)/Drivers/STM32G4xx_HAL_Driver/Src/%.o: Drivers/STM32G4xx_HAL_Driver/Src/%.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo '컴파일 중 (HAL): $<'
	$(CC) -std=gnu11 $(CFLAGS) -MMD -MP -MF $(BUILD_DIR)/Drivers/STM32G4xx_HAL_Driver/Src/$*.d -c -o $@ $<

# ============================================
# 어셈블리 컴파일 규칙
# ============================================
$(BUILD_DIR)/startup_stm32g431xx.o: startup_stm32g431xx.s | $(BUILD_DIR)
	@echo '어셈블 중: $<'
	$(AS) -c $(ASFLAGS) -o $@ $<

# ============================================
# 빌드 디렉토리 생성
# ============================================
$(BUILD_DIR):
	@mkdir -p $@

# ============================================
# 의존성 파일 포함 (존재하는 경우)
# ============================================
-include $(DEPS)

# ============================================
# clean 타겟
# ============================================
clean:
	@echo '빌드 산출물 삭제 중...'
	-rm -fR $(BUILD_DIR)

# ============================================
# flash 타겟 (st-flash 사용)
# ============================================
flash: $(BUILD_DIR)/$(TARGET).bin
	@echo '플래시 다운로드 중...'
	st-flash write $< 0x08000000

# ============================================
# 디버그 정보 출력
# ============================================
size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $<

disasm: $(BUILD_DIR)/$(TARGET).elf
	$(OBJDUMP) -d $< > $(BUILD_DIR)/$(TARGET).asm

# ============================================
# 헬퍼 타겟
# ============================================
.PHONY: all clean flash size disasm
