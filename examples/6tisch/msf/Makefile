CONTIKI_PROJECT = msf-node msf-root
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = sky z1 nrf52dk native simplelink

ifdef APP_SEND_INTERVAL
CFLAGS += -DAPP_SEND_INTERVAL=$(APP_SEND_INTERVAL)
endif

CONTIKI = ../../..

MAKE_MAC = MAKE_MAC_TSCH
MODULES += os/net/mac/tsch/sixtop
MODULES += os/services/msf
MODULES += os/services/shell

include $(CONTIKI)/Makefile.include
