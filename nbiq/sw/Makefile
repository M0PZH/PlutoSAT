# nbiq user-space tool. Cross compile with CC=arm-none-linux-gnueabihf-gcc.
CC      ?= gcc
CFLAGS  ?= -O2
CFLAGS  += -Wall -Wextra -std=gnu11
LDLIBS  += -lm

all: nbiq

nbiq: nbiq.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ nbiq.c $(LDLIBS)

install: nbiq
	install -D -m 0755 nbiq $(DESTDIR)/usr/bin/nbiq

clean:
	rm -f nbiq

.PHONY: all install clean
