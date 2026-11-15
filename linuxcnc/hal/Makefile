# Builds and installs the HAL driver on the LinuxCNC host:
#   make install        (needs linuxcnc-uspace-dev, uses halcompile)
#
# halcompile takes a single source file, so the shared protocol header from
# firmware/lib is merged into one file first.

PROTO := ../../firmware/lib/cncproto.h
BUILD := build

$(BUILD)/cncboard_spi.c: cncboard_spi.c $(PROTO)
	mkdir -p $(BUILD)
	awk -v proto=$(PROTO) '/#include "cncproto.h"/ { while ((getline line < proto) > 0) print line; next } { print }' \
	    cncboard_spi.c > $@

install: $(BUILD)/cncboard_spi.c
	cd $(BUILD) && sudo halcompile --install cncboard_spi.c

clean:
	rm -rf $(BUILD)

.PHONY: install clean
