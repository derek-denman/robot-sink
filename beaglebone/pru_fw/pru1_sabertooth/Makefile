FW_NAME := am335x-pru1-fw
PRU_SSP ?= /usr/lib/pru-software-support-package
PRU_CC ?= pru-gcc
CFLAGS := -O2 -Wall -Wextra -Werror -I$(PRU_SSP)/include -I../common
LDFLAGS := -T$(PRU_SSP)/include/am335x-pru.cmd

all: $(FW_NAME)

$(FW_NAME): main.c
	$(PRU_CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

clean:
	rm -f $(FW_NAME)
