# Firmware build: WeAct MiniSTM32H723 (STM32H723VGT6), SPI link to the host.
#
#   make                 build build/cncboard-weact-h723-spi.{elf,bin}
#   make flash           flash over USB-C (board in DFU mode, needs dfu-util)
#
# Needs the GNU Arm toolchain and the STM32CubeH7 package (HAL + CMSIS):
#   CROSS    prefix of the toolchain, default arm-none-eabi-
#   CUBE_H7  path of STM32Cube_FW_H7_Vx.y.z

CROSS   ?= arm-none-eabi-
CUBE_H7 ?= $(HOME)/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1

NAME  := cncboard-weact-h723-spi
BUILD := build
LIB   := ../../lib
HAL   := $(CUBE_H7)/Drivers/STM32H7xx_HAL_Driver
CMSIS := $(CUBE_H7)/Drivers/CMSIS

SRCS := src/main.c st/system_stm32h7xx.c st/startup_stm32h723vgtx.s \
        $(LIB)/cnc_core.c $(LIB)/stepgen.c \
        $(addprefix $(HAL)/Src/stm32h7xx_hal, .c _cortex.c _rcc.c _rcc_ex.c _gpio.c \
          _dma.c _dma_ex.c _pwr.c _pwr_ex.c _spi.c _spi_ex.c)

INCS := -Isrc -Ist -I$(LIB) -I$(HAL)/Inc -I$(CMSIS)/Device/ST/STM32H7xx/Include -I$(CMSIS)/Include
ARCH := -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb
CFLAGS := $(ARCH) -std=gnu11 -O2 -g -Wall -ffunction-sections -fdata-sections \
          -DSTM32H723xx -DUSE_HAL_DRIVER $(INCS)
LDFLAGS := $(ARCH) -Tst/STM32H723VGTX_FLASH.ld --specs=nano.specs --specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(NAME).map

OBJS := $(addprefix $(BUILD)/, $(addsuffix .o, $(notdir $(basename $(SRCS)))))
vpath %.c $(sort $(dir $(SRCS)))
vpath %.s $(sort $(dir $(SRCS)))

all: $(BUILD)/$(NAME).bin

$(BUILD)/%.o: %.c src/board.h | $(BUILD)
	$(CROSS)gcc $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s | $(BUILD)
	$(CROSS)gcc $(CFLAGS) -c $< -o $@

$(BUILD)/$(NAME).elf: $(OBJS)
	$(CROSS)gcc $(LDFLAGS) $^ -o $@
	$(CROSS)size $@

$(BUILD)/$(NAME).bin: $(BUILD)/$(NAME).elf
	$(CROSS)objcopy -O binary $< $@

$(BUILD):
	mkdir -p $@

flash: $(BUILD)/$(NAME).bin
	dfu-util -a 0 -s 0x08000000:leave -D $<

clean:
	rm -rf $(BUILD)

.PHONY: all flash clean
