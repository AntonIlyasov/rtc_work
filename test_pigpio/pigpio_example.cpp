#include <csignal>
#include <iostream>
#include <pigpio.h>

const int RedLED = 21;
volatile sig_atomic_t signal_received = 0;

void sigint_handler(int signal) {
   signal_received = signal;
}

int main() {
   if (gpioInitialise() == PI_INIT_FAILED) {
      std::cout << "ERROR: Failed to initialize the GPIO interface." << std::endl;
      return 1;
   }
   gpioSetMode(RedLED, PI_OUTPUT);
   signal(SIGINT, sigint_handler);
   std::cout << "Press CTRL-C to exit." << std::endl;
   while (!signal_received) {
      gpioWrite(RedLED, PI_HIGH);
      time_sleep(1);
      gpioWrite(RedLED, PI_LOW);
      time_sleep(1);
   }
   gpioSetMode(RedLED, PI_INPUT);
   gpioTerminate();
   std::cout << std::endl;
   return 0;
}