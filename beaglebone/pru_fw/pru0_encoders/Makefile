FW_NAME := am335x-pru0-fw
FW_BIN := $(FW_NAME).bin
PRU_SSP ?= /usr/lib/pru-software-support-package
PRU_CC ?= pru-gcc
PRU_CLPRU ?= clpru
PRU_LNKPRU ?= lnkpru
PRU_CMD_FILE ?= $(PRU_SSP)/include/am335x-pru.cmd
PRU_OBJCOPY ?= pru-objcopy
TOOLCHAIN ?= gcc
PRU_TI_CGT_LIBDIR ?=
PRU_TI_CGT_INCLUDEDIR ?=

CFLAGS_GCC := -O2 -Wall -Wextra -Werror -I$(PRU_SSP)/include -I$(PRU_SSP)/include/am335x -I../common
LDFLAGS_GCC := -T$(PRU_CMD_FILE)

CFLAGS_TI := --c99 --endian=little --hardware_mac=on \
	--silicon_version=3 --abi=eabi \
	--include_path=$(PRU_SSP)/include \
	--include_path=$(PRU_SSP)/include/am335x \
	$(if $(strip $(PRU_TI_CGT_INCLUDEDIR)),--include_path=$(PRU_TI_CGT_INCLUDEDIR),) \
	--include_path=../common

all: $(FW_NAME) $(FW_BIN)

ifeq ($(TOOLCHAIN),ti)
TI_OBJS := main.obj

TI_LINK_DIRS :=
ifneq ($(strip $(PRU_TI_CGT_LIBDIR)),)
TI_LINK_DIRS += -i$(PRU_TI_CGT_LIBDIR)
endif

$(FW_NAME): $(TI_OBJS)
	$(PRU_LNKPRU) --abi=eabi $(PRU_CMD_FILE) $(TI_LINK_DIRS) $^ --library=libc.a -o $@

main.obj: main.c
	$(PRU_CLPRU) $(CFLAGS_TI) --output_file=$@ $<
else
$(FW_NAME): main.c
	$(PRU_CC) $(CFLAGS_GCC) $(LDFLAGS_GCC) -o $@ $<
endif

$(FW_BIN): $(FW_NAME)
	$(PRU_OBJCOPY) -O binary $< $@

clean:
	rm -f $(FW_NAME) $(FW_BIN) main.obj
