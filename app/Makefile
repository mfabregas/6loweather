PROJECT ?= mqtt-client

all: $(PROJECT)

CONTIKI ?= ../6lbr
PROJECTDIR ?= .

TARGET ?= zoul

WITH_LLSEC ?= 0
CONTIKI_WITH_RPL = 1

WITH_RELAYR ?= 0
CFLAGS += -DWITH_RELAYR_PLATFORM=$(WITH_RELAYR)

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
CONTIKI_TARGET_SOURCEFILES += sht25.c bmpx8x.c weather-meter.c
PROJECT_SOURCEFILES += sixloweather.c

APPS += mqtt
WITH_IP64 = 1

include $(CONTIKI)/Makefile.include
