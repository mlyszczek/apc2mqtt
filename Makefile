CFLAGS ?= -Wall -Wextra
LDFLAGS ?= -Wall -Wextra -O0 -ggdb -g3
LIBS = -lmosquitto -lbsd -lembedlog

apc2mqtt: main.c
	cc $(CFLAGS) $(LDFLAGS) $(LIBS) $^ -o $@
