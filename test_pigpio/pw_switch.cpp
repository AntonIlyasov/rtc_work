#include <csignal>
#include <iostream>
#include <pigpio.h>
#include <ctime>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

const int pw_pin                      = 21;
int cur_state_pw_pin                  = 0;
volatile sig_atomic_t signal_received = 0;

void sigint_handler(int signal) {
   signal_received = signal;
}

bool iserInputSpace(){
   std::cout << "\n[CLICK SPACE]\n";
   char user_input = getchar();
   if (user_input == ' '){
      std::cout << "\n[USER ENTERED SPACE]\n";
      return true;
   } 
   return false;
}

void checkPinState(){
   cur_state_pw_pin = gpioRead(pw_pin);
   std::cout << "\n[CURRENT PIN STATE] : [" << cur_state_pw_pin << "]\n";
}

void changePinState(){
   gpioSetMode(pw_pin, PI_OUTPUT);
   if      (cur_state_pw_pin == 0) gpioWrite(pw_pin, PI_HIGH);
   else if (cur_state_pw_pin == 1) gpioWrite(pw_pin, PI_LOW);
}

void printCurrentPinState(){
   cur_state_pw_pin = gpioRead(pw_pin);
   std::cout << "\n[CURRENT PIN STATE CHANGED ON] : [" << cur_state_pw_pin << "]\n";
}

int main() {

   if (gpioInitialise() == PI_INIT_FAILED) {
      std::cout << "ERROR: Failed to initialize the GPIO interface." << std::endl;
      return 1;
   }
   signal(SIGINT, sigint_handler);
   std::cout << "Press CTRL-C to exit." << std::endl;
   std::cout << "Press SPACE to change pin " << pw_pin <<" state." << std::endl;
   while(!signal_received){
      if (iserInputSpace()){
         checkPinState();
         changePinState();
         printCurrentPinState();
      }
   }
   gpioSetMode(pw_pin, PI_INPUT);
   gpioTerminate();
   std::cout << std::endl;
   return 0;
}
