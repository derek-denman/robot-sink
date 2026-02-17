.PHONY: all clean deploy

SUBDIRS := pru0_encoders pru1_sabertooth

all:
	@for d in $(SUBDIRS); do \
		$(MAKE) -C $$d all || exit $$?; \
	done

clean:
	@for d in $(SUBDIRS); do \
		$(MAKE) -C $$d clean || exit $$?; \
	done

deploy: all
	@echo "Run ../../scripts/deploy_firmware.sh to install firmware on target"
