
PREFIX		?= arm-none-eabi
SIZE		:= $(PREFIX)-size
CXX			:= $(PREFIX)-g++
CC			:= $(PREFIX)-gcc
LD			:= $(PREFIX)-ld
OBJCOPY		:= $(PREFIX)-objcopy
OBJDUMP		:= $(PREFIX)-objdump
MKDIR_P		:= mkdir -p

ARCH_FLAGS = -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd
CFLAGS = -I. -Isrc -Isx126x -Irtos -Ilibopencm3/include -nostdlib $(ARCH_FLAGS) -Os -DSTM32F1
CFLAGS += -fno-common -ffunction-sections -fdata-sections


LDSCRIPT = gd32f103.ld


OUT_DIR = obj

ifneq ($(V), 1)
	Q := @
endif

LIBFREERTOS = libfreertos.a

RTOS_OBJS = heap_4.o list.o port.o tasks.o queue.o opencm3.o
RTOS_OBJS := $(patsubst %.o, obj/%.o, $(RTOS_OBJS))

LIBSX126X = libsx126x.a

SX126X_OBJS = sx126x.o sx126x_driver_version.o lr_fhss_mac.o sx126x_lr_fhss.o sx126x_hal.o
SX126X_OBJS := $(patsubst %.o, obj/%.o, $(SX126X_OBJS))

FIRMWARE = fimrware.elf
BINARY = firmware.bin

OBJS = main.o init.o radio.o serial.o message.o crc16.o
OBJS := $(patsubst %.o, obj/%.o, $(OBJS))

vpath %.c src/ rtos/ sx126x/

all: $(BINARY)

$(BINARY): $(FIRMWARE)
	@echo [BN]	$@
	$(Q)$(OBJCOPY) -O binary $< $@

$(FIRMWARE): $(OUT_DIR) libopencm3 $(LIBFREERTOS) $(LIBSX126X) $(OBJS)
	@echo [CC]	$@
	$(Q)$(CC) -o $@ $(OBJS) -T gd32f103.ld -L. -Llibopencm3/lib --specs=nosys.specs -nostartfiles -lc -lgcc -lnosys -lsx126x -lfreertos -lopencm3_stm32f1

libopencm3: libopencm3/lib/libopencm3_stm32f1.a

libopencm3/lib/libopencm3_stm32f1.a:
	@echo [MK] $@
	$(Q)$(MAKE) -C libopencm3 TARGETS=stm32/f1

$(LIBFREERTOS): $(RTOS_OBJS)
	@echo [LD] 	$@
	$(Q)$(LD) -r -o $@ $^

$(LIBSX126X): $(SX126X_OBJS)
	@echo [LD]	$@
	$(Q)$(LD) -r -o $@ $^

$(OUT_DIR):
	$(Q)$(MKDIR_P) $(OUT_DIR)

$(OUT_DIR)/%.o: %.c Makefile
	@echo [CC]	$<
	$(Q)$(CC) $(CFLAGS) -c -o $@ $<

clean:
	@echo [RM] $(BINARY)
	$(Q)rm -f $(BINARY)
	@echo [RM] $(FIRMWARE)
	$(Q)rm -f $(FIRMWARE)
	@echo [RM] $(LIBSX126X)
	$(Q)rm -f $(LIBSX126X)
	@echo [RM] $(LIBFREERTOS)
	$(Q)rm -f $(LIBFREERTOS)
	@echo [RM] $(OUT_DIR)
	$(Q)rm -rf $(OUT_DIR)
