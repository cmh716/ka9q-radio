# -*- makefile -*-
# sub-make for ka9q-radio systemd .service files
# Copyright 2025, Phil Karn, KA9Q

UNAME_S := $(shell uname -s)

all:

clean:

install: install-$(UNAME_S)

install-Linux:
ifeq ($(IN_DOCKER),1)
	@echo "Skipping systemctl tuning in Docker"
else
	rsync $(BLACKLIST) /etc/modprobe.d
	rsync * /etc/systemd/system
	systemctl daemon-reload
endif

install-Darwin:
	# TODO
	@exit 0

install-%:
	@echo "Unsupported operating system: $*"
	@exit 1

.PHONY: clean all install install-Linux install-Darwin install-%
