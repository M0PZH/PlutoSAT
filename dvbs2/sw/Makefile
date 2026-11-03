# dvbs2-tx / dvbs2-rx user-space tools. Cross compile with CC=arm-none-linux-gnueabihf-gcc.
CC      ?= gcc
CFLAGS  ?= -O2
CFLAGS  += -Wall -Wextra -std=gnu11
LDLIBS  += -lm

all: dvbs2-tx dvbs2-rx bbframe-cli bbdeframe-cli

dvbs2-tx: dvbs2_tx.c bbframe.c bbframe.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ dvbs2_tx.c bbframe.c $(LDLIBS)

dvbs2-rx: dvbs2_rx.c bbdeframe.c bbdeframe.h bbframe.c bbframe.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ dvbs2_rx.c bbdeframe.c bbframe.c $(LDLIBS)

bbdeframe-cli: bbdeframe_cli.c bbdeframe.c bbdeframe.h bbframe.c bbframe.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ bbdeframe_cli.c bbdeframe.c bbframe.c

bbframe-cli: bbframe_cli.c bbframe.c bbframe.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ bbframe_cli.c bbframe.c

install: dvbs2-tx dvbs2-rx
	install -D -m 0755 dvbs2-tx $(DESTDIR)/usr/bin/dvbs2-tx
	install -D -m 0755 dvbs2-rx $(DESTDIR)/usr/bin/dvbs2-rx

clean:
	rm -f dvbs2-tx dvbs2-rx bbframe-cli bbdeframe-cli

.PHONY: all install clean
