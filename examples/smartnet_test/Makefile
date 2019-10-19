TARGET = simplelink
BOARD = smartnet

CFLAGS += -g

MODULES += os/services/shell
MODULES += os/net/app-layer/mqtt

CONTIKI_PROJECT = smartnet_test
all: $(CONTIKI_PROJECT)

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

