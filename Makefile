CFLAGS ?= -Wall -Wextra
LDFLAGS ?=
LIBS = -lmosquitto -lbsd -lembedlog
PREFIX ?= /usr/local
DESTDIR ?= /


apc2mqtt: main.o
	cc $(LDFLAGS) -o $@ $^ $(LIBS)

main.o: main.c
	cc $(CFLAGS) -c $^ -o $@

install: apc2mqtt
	install apc2mqtt -m755 $(DESTDIR)/$(PREFIX)/bin/

clean:
	rm -f apc2mqtt
	rm -f main.o
