#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <time.h>
#include <mraa.h>

#define HEADERPIN 11

void sleepMillis(uint32_t millis) {
  struct timespec sleep;
  sleep.tv_sec = millis / 1000;
  sleep.tv_nsec = (millis % 1000) * 1000000L;
  while(clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep, &sleep) && errno == EINTR);
}

int main(int argc, char **argv)
{
	mraa_init();
	mraa_gpio_context gpio;

	if (!(gpio = mraa_gpio_init(HEADERPIN))) {
		fprintf(stderr, "Error exporting pin %d!\n", HEADERPIN);
		mraa_deinit();
		exit(1);
	}

	/* Check if the binary has root-permissions: if not,
	sleep for 100ms to give udev time to set the GPIO-permissions
	correctly for us to use the pin we just initialized above.
	!IMPORTANT! */
	/* Try uncommenting this or changing the amount of time
	we sleep and see what happens. */
	if(geteuid()) sleepMillis(100);

	if(mraa_gpio_dir(gpio, MRAA_GPIO_OUT) != MRAA_SUCCESS){
		fprintf(stderr, "Error setting pin-direction!\n");
		mraa_gpio_close(gpio);
		mraa_deinit();
		exit(1);
	}
	printf("Blink HIGH..\n");
	mraa_gpio_write(gpio, 1);
	sleepMillis(1000); //Sleep one second
	printf("Blink LOW..\n");
	mraa_gpio_write(gpio, 0);
	sleepMillis(1000); //Sleep one second
	mraa_gpio_dir(gpio, MRAA_GPIO_IN);
	mraa_gpio_close(gpio);
	mraa_deinit();
	exit(0);
}