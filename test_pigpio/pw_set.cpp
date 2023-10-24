#include <csignal>
#include <iostream>
#include <pigpio.h>

const int pw_pin        = 21;
int cur_state_pw_pin    = 0;

int main() {
   if (gpioInitialise() == PI_INIT_FAILED) {
      std::cout << "ERROR: Failed to initialize the GPIO interface." << std::endl;
      return 1;
   }
   std::cout << "\nPI_HIGH\n";
   gpioSetMode(pw_pin, PI_OUTPUT);
   gpioWrite(pw_pin, PI_HIGH);
   time_sleep(10);
   //gpioSetMode(pw_pin, PI_INPUT);
   cur_state_pw_pin = gpioRead(pw_pin);
   std::cout << "\n[CURRENT PIN STATE] : [" << cur_state_pw_pin << "]\n";
   gpioTerminate();
   std::cout << std::endl;
   while(1);
   return 0;
}
