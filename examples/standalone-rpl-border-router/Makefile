CONTIKI_PROJECT = standalone-rpl-border-router
all: $(CONTIKI_PROJECT)
CONTIKI = ../..

PLATFORMS_ONLY = native

MODULES += os/net/nullnet
MODULES += os/services/shell
MODULES += os/services/slip-cmd

PROJECT_SOURCEFILES += slip-config.c slip-dev.c

include $(CONTIKI)/Makefile.include
