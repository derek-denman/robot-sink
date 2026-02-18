FW_NAME := am335x-pru0-fw
PRU_SSP ?= /usr/lib/pru-software-support-package
PRU_CC ?= pru-gcc
PRU_CLPRU ?= clpru
PRU_LNKPRU ?= lnkpru
PRU_CMD_FILE ?= $(PRU_SSP)/include/am335x-pru.cmd
TOOLCHAIN ?= gcc
PRU_TI_CGT_LIBDIR ?=
PRU_TI_RPMSG_LIB ?=
PRU_TI_RPMSG_SRC ?=

CFLAGS_GCC := -O2 -Wall -Wextra -Werror -I$(PRU_SSP)/include -I../common
LDFLAGS_GCC := -T$(PRU_CMD_FILE)

CFLAGS_TI := --c99 --endian=little --hardware_mac=on \
	--include_path=$(PRU_SSP)/include \
	--include_path=$(PRU_SSP)/include/am335x \
	--define=BBB_RPMSG_CHANNEL_NO_DESC=1 \
	--include_path=../common

all: $(FW_NAME)

ifeq ($(TOOLCHAIN),ti)
TI_OBJS := main.obj
TI_EXTRA_LINK :=
ifneq ($(strip $(PRU_TI_RPMSG_SRC)),)
TI_OBJS += rpmsg.obj
else ifneq ($(strip $(PRU_TI_RPMSG_LIB)),)
TI_EXTRA_LINK += $(PRU_TI_RPMSG_LIB)
endif

TI_LINK_DIRS :=
ifneq ($(strip $(PRU_TI_CGT_LIBDIR)),)
TI_LINK_DIRS += -i$(PRU_TI_CGT_LIBDIR)
endif

$(FW_NAME): $(TI_OBJS)
	$(PRU_LNKPRU) $(PRU_CMD_FILE) $(TI_LINK_DIRS) $^ $(TI_EXTRA_LINK) --library=libc.a -o $@

main.obj: main.c
	$(PRU_CLPRU) $(CFLAGS_TI) --output_file=$@ $<

rpmsg.obj: $(PRU_TI_RPMSG_SRC)
	$(PRU_CLPRU) $(CFLAGS_TI) --output_file=$@ $<
else
$(FW_NAME): main.c
	$(PRU_CC) $(CFLAGS_GCC) $(LDFLAGS_GCC) -o $@ $<
endif

clean:
	rm -f $(FW_NAME) main.obj rpmsg.obj
