#
# Makefile
#

CIRCLE_STDLIB_DIR = ../MiniDexed/circle-stdlib
SYNTH_DEXED_DIR = ../MiniDexed/Synth_Dexed/src
CMSIS_DIR = ../MiniDexed/CMSIS_5/CMSIS
NEWLIB_ARCH ?= aarch64-none-circle

CFLAGS += -g0
CXXFLAGS += -g0

OBJS = main.o kernel.o
#TARGET = kernel8.img

include Rules.mk

# Clean target
.PHONY: clean

clean:
	@echo "Cleaning up..."
	rm -f $(OBJS) *.o *.d *~ core
