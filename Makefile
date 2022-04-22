apc2mqtt: main.c
	cc -Wall -Wextra $^ -o $@ -lmosquitto -lembedlog
