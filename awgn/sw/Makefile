# Self-test tools. Cross compile with CC=arm-none-linux-gnueabihf-gcc.
CC      ?= gcc
CFLAGS  ?= -O2
CFLAGS  += -Wall -Wextra -std=gnu11
LDLIBS  += -lm

all: rx-awgn ts-prbs

rx-awgn: rx_awgn.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ rx_awgn.c $(LDLIBS)

ts-prbs: ts_prbs.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ ts_prbs.c

install: rx-awgn ts-prbs
	install -D -m 0755 rx-awgn $(DESTDIR)/usr/bin/rx-awgn
	install -D -m 0755 ts-prbs $(DESTDIR)/usr/bin/ts-prbs
	install -D -m 0755 dvbs2-selftest $(DESTDIR)/usr/bin/dvbs2-selftest

clean:
	rm -f rx-awgn ts-prbs

.PHONY: all install clean
