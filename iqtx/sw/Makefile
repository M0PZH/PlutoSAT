# iqtx user-space tool. Cross compile with CC=arm-none-linux-gnueabihf-gcc.
CC      ?= gcc
CFLAGS  ?= -O2
CFLAGS  += -Wall -Wextra -std=gnu11
LDLIBS  += -lm

all: iqtx

iqtx: iqtx.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ iqtx.c $(LDLIBS)

install: iqtx
	install -D -m 0755 iqtx $(DESTDIR)/usr/bin/iqtx

clean:
	rm -f iqtx

.PHONY: all install clean
