CONTIKI_PROJECT = 6lr-with-slip
all: $(CONTIKI_PROJECT)
CONTIKI = ../../..

PLATFORMS_EXCLUDE = native

MAKE_MAC = MAKE_MAC_TSCH
MODULES += os/net/mac/tsch/sixtop
MODULES += os/services/msf
MODULES += os/services/demux/mac-demux

CFLAGS += -I..

include $(CONTIKI)/Makefile.include
