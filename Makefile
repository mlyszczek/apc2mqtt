CFLAGS ?= -Wall -Wextra
LDFLAGS ?= -Wall -Wextra
LIBS = -lmosquitto -lbsd -lembedlog
PREFIX ?= /usr/local
DESTDIR ?= /

apc2mqtt: main.c
	cc $(CFLAGS) $(LDFLAGS) $(LIBS) $^ -o $@

install: apc2mqtt
	install apc2mqtt -m755 $(DESTDIR)/$(PREFIX)/bin/

clean:
	rm -f apc2mqtt
