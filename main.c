#include <embedlog.h>
#include <errno.h>
#include <fcntl.h>
#include <mosquitto.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "valid.h"

/* ==========================================================================
                     ____ _ / /____   / /_   ____ _ / /_____
                    / __ `// // __ \ / __ \ / __ `// // ___/
                   / /_/ // // /_/ // /_/ // /_/ // /(__  )
                   \__, //_/ \____//_.___/ \__,_//_//____/
                  /____/
   ========================================================================== */


static struct mosquitto  *mqtt;


/* ==========================================================================
                   ____ ___   ____ _ _____ _____ ____   _____
                  / __ `__ \ / __ `// ___// ___// __ \ / ___/
                 / / / / / // /_/ // /__ / /   / /_/ /(__  )
                /_/ /_/ /_/ \__,_/ \___//_/    \____//____/
   ========================================================================== */


#define goto_perror(L, S, ...) do { \
	el_perror(ELE, S, ##__VA_ARGS__); goto L; } while(0)
#define return_perror(...) do { \
	el_perror(ELE, __VA_ARGS__); return -1; } while(0)
#define return_print(R, ...) do { \
	el_print(ELW, __VA_ARGS__); return R; } while(0)
#define apc_buflen (32)


/* ==========================================================================
              / __/__  __ ____   _____ / /_ (_)____   ____   _____
             / /_ / / / // __ \ / ___// __// // __ \ / __ \ / ___/
            / __// /_/ // / / // /__ / /_ / // /_/ // / / /(__  )
           /_/   \__,_//_/ /_/ \___/ \__//_/ \____//_/ /_//____/
   ========================================================================== */


/* ==========================================================================
   ========================================================================== */
int open_serial_port(const char *dev, unsigned int speed)
{
	struct termios tty;
	int fd;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


	if ((fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC)) < 0)
		return_perror("open(%s)", dev);

	if (tcgetattr(fd, &tty) != 0)
		return_perror("tcgetattr(%s)", dev);

	if (cfsetispeed(&tty, (speed_t)speed) != 0)
		return_perror("cfsetispeed(%s, %d)", dev, speed);

	tty.c_cflag &= ~CSIZE;          /* apply size mask */
	tty.c_cflag |= CS8;             /* 8bit transmission */
	tty.c_cflag &= ~PARENB;         /* no parity check */
	tty.c_cflag &= ~CSTOPB;         /* set one stop bit */
	tty.c_cflag |= CLOCAL;          /* ignore modem lines */
	tty.c_cflag &= ~CREAD;          /* disable receiver - we only send data */
	tty.c_oflag |= OPOST | ONLCR;   /* enable output post-processing by OS */

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
		return_perror("tcsetattr(%s)");

	return fd;
}


/* ==========================================================================
   ========================================================================== */
void apc_init(int fd)
{
	char buf[16];
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


	el_print(ELN, "initializing apc");
	for (;;)
	{
		memset(buf, 0x00, sizeof(buf));
		write(fd, "Y", 1);
		sleep(1);
		read(fd, buf, sizeof(buf));

		if (strncmp(buf, "SM", 2) == 0)
			break;
		else
		{
			el_print(ELW, "failed to initialize, trying again");
			el_pmemory(ELW, buf, sizeof(buf));
		}
	}
}


/* ==========================================================================
   ========================================================================== */
int apc_cmd_to_mqtt(int fd, char cmd, const char *topic)
{
	int r;
	char buf[apc_buflen];
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


	memset(buf, 0x00, sizeof(buf));
	write(fd, &cmd, sizeof(cmd));

	r = read(fd, buf, apc_buflen);
	if (r == 0)
		return_print(-1, "read 0 bytes from serial");
	if (r < 0)
		return_perror("read()");

	/* apc will return us \r\n */
	buf[strcspn(buf, "\r")] = '\0';
	if (mosquitto_publish(mqtt, NULL, topic, strlen(buf), buf, 2, 0))
		return_perror("mosquitto_publish(%s, %s)", topic, buf);

	return 0;
}

/* ==========================================================================
                           ____ ___   ____ _ (_)____
                          / __ `__ \ / __ `// // __ \
                         / / / / / // /_/ // // / / /
                        /_/ /_/ /_/ \__,_//_//_/ /_/
   ========================================================================== */
int main(int argc, char *argv[])
{
	int fd, ret;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	if (argc != 2)
	{
		fprintf(stderr, "usage: apc2mqtt <apc-serial>\n");
		return 1;
	}

	ret = -1;
	/* logger init */
	el_init();
	el_option(EL_OUT, EL_OUT_FILE | EL_OUT_STDERR);
	el_option(EL_TS, EL_TS_LONG);
	el_option(EL_TS_TM, EL_TS_TM_REALTIME);
	el_option(EL_PRINT_LEVEL, 1);
	el_option(EL_FPATH, "/var/log/apc2mqtt.log");
	el_option(EL_FSYNC_EVERY, 0);
	el_print(ELN, "apc2mqtt starting");

	/* init mosquitto */
	el_print(ELN, "initializing mqtt");
	mosquitto_lib_init();
	mqtt = mosquitto_new(NULL, 1, NULL);
	if (!mqtt)
		goto_perror(mosq_new_error, "mosquitto_new(NULL, 1, NULL)");
	el_print(ELN, "connecting to server");
	if (mosquitto_connect(mqtt, "10.1.1.1", 1883, 60))
		goto_perror(mosq_connect_error, "mosquitto_connect(10.1.1.1, 1883)");
	el_print(ELN, "starting mosquitto loop");
	if (mosquitto_loop_start(mqtt))
		goto_perror(mosq_loop_start_error, "mosquitto_loop_start()");

	/* init serial to apc */
	el_print(ELN, "initializing serial");
	fd = open_serial_port(argv[1], B2400);
	if (fd == -1)
		goto_perror(open_serial_error, "open_serial_port(%s, B2400)", argv[1]);
	apc_init(fd);

	for (;;)
	{
		ret = 0;
		ret |= apc_cmd_to_mqtt(fd, 'L', "/apc/input/voltage");
		ret |= apc_cmd_to_mqtt(fd, '/', "/apc/power/load");
		ret |= apc_cmd_to_mqtt(fd, 'C', "/apc/temperature");
		ret |= apc_cmd_to_mqtt(fd, 'f', "/apc/battery/charge");
		ret |= apc_cmd_to_mqtt(fd, 'B', "/apc/battery/voltage");
		ret |= apc_cmd_to_mqtt(fd, 'j', "/apc/estimated-runtime");

		if (ret)
			apc_init(fd);

		sleep(1);
	}

	close(fd);
open_serial_error:
	mosquitto_loop_stop(mqtt, 0);
mosq_loop_start_error:
	mosquitto_disconnect(mqtt);
mosq_connect_error:
	mosquitto_destroy(mqtt);
mosq_new_error:
	mosquitto_lib_cleanup();
	el_cleanup();

	return ret;
}
