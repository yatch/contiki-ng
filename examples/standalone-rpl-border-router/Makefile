CONTIKI_PROJECT = standalone-rpl-border-router
all: $(CONTIKI_PROJECT)
CONTIKI = ../..

PLATFORMS_ONLY = native

MODULES += os/net/nullnet
MODULES += os/services/shell

PROJECT_SOURCEFILES += slip-config.c
.INTERMEDIATE: slip-config.c
slip-config.c: $(CONTIKI)/os/services/rpl-border-router/native/slip-config.c
	@cp -p $< $@

include $(CONTIKI)/Makefile.include
