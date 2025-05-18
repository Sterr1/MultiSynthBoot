#
# Makefile
#

CIRCLE_STDLIB_DIR = ../MiniDexed/circle-stdlib
SYNTH_DEXED_DIR = ../MiniDexed/Synth_Dexed/src
CMSIS_DIR = ../MiniDexed/CMSIS_5/CMSIS

# Инструменты сборки
PREFIX = aarch64-none-elf-
CC = $(PREFIX)g++
LD = $(PREFIX)ld
OBJCOPY = $(PREFIX)objcopy

# Флаги компиляции
CFLAGS = -O3 -nostdinc -ffreestanding \
         -I$(CIRCLE_CORE)/include \
         -I$(CIRCLE_ADDONS)/SDCard \
         -I$(CIRCLE_STDLIB)/include \
         -Isrc \
         -DUSE_PWM_AUDIO_ON_ZERO

# Флаги линковки
LDFLAGS = -T $(CIRCLE_CORE)/lib/armstub64.ld -nostdlib
LIBS = -L$(CIRCLE_CORE)/lib -lcircle \
       -L$(CIRCLE_ADDONS)/SDCard -lsdcard

# Цели сборки
OBJS = main.o
TARGET = kernel8.img

include Rules.mk

main.o: main.cpp main.h
	$(CC) $(CFLAGS) -c $< -o $@

all: $(TARGET)

$(TARGET): $(OBJS)
	$(LD) $(LDFLAGS) -o kernel8.elf $(OBJS) $(LIBS)
	$(OBJCOPY) kernel8.elf -O binary $@

main.o: main.cpp main.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET) *.elf *.lst *.map
