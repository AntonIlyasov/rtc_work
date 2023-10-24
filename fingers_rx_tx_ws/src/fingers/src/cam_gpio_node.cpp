/*
  Задачи данной программы:
  1) Прием данных из ROS-топика(1) для последующей отправки на пин GPIO
  2) Отправка  полученных данных на пин GPIO
  3) Получение актуальных данных c пина GPIO
  4) Отправка  полученных данных c пина GPIO в ROS-топик(2) для последующей отправки по UDP
*/

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <boost/asio.hpp>
#include <boost/chrono.hpp>
#include <unistd.h>
#include <pigpio.h>

// используемый пин для вкл/выкл камеры (по умолчанию 23. Нужное значение передается через launch файл)
int gpio_cam_pin = 23;

// имена ROS-топиков для обмена данными c программой "master_eth_receiver"
#define TO_cam_gpio_state_TOPIC_NAME      "toCamGPIOTopic"
#define FROM_cam_gpio_state_TOPIC_NAME    "fromCamGPIOTopic"

/*
  Класс Cam_GPIO_Node предназначен для реализации функционала обмена данными между ROS-топиками и пином GPIO.
  Класс Cam_GPIO_Node включает следующие методы:
  * gpio_process()                  - основной цикл программы, который состоит из последовательности действий:
      1) setChangedStateToCamGPIO()       - отправка измененных полученных с ROS-топика(1) данных на пин GPIO
      2) getCurrentCamGPIOStatus()        - получение актуальных данных с пина GPIO
      3) sendMsgFromCamGPIO()             - отправка полученных данных c пина GPIO в ROS-топик(2) для последующей отправки по UDP
  * to_cam_gpio_state_callback()    - обработчик сообщений от ROS-топика(1) на пин GPIO
*/
class Cam_GPIO_Node
{
public:
  Cam_GPIO_Node(){
    cam_gpio_state_sub = node.subscribe<std_msgs::UInt8>(TO_cam_gpio_state_TOPIC_NAME,   0, &Cam_GPIO_Node::to_cam_gpio_state_callback,this);
    cam_gpio_state_pub = node.advertise<std_msgs::UInt8>(FROM_cam_gpio_state_TOPIC_NAME, 0);
  }

  // основной цикл программы
  void gpio_process(){
    if (getMsgToCamGPIO){
      getMsgToCamGPIO = false;
      setChangedStateToCamGPIO();
      getCurrentCamGPIOStatus();
      sendMsgFromCamGPIO();
    }
  }

private:
  ros::NodeHandle node;
  ros::Subscriber cam_gpio_state_sub;
  ros::Publisher  cam_gpio_state_pub;
  uint8_t cam_gpio_state                = 0;
  uint8_t cam_gpio_state_prev           = 0;
  uint8_t cam_status_from_gpio          = 0;
  bool getMsgToCamGPIO                  = false;
  bool data_for_gpio_is_changed         = false;

  // обработчик сообщений от ROS-топика(1) на пин GPIO
  void to_cam_gpio_state_callback(const std_msgs::UInt8::ConstPtr& cam_gpio_state_msg){
    getMsgToCamGPIO = true;
    cam_gpio_state  = cam_gpio_state_msg->data;
  }

  // отправка измененных полученных с ROS-топика(1) данных на пин GPIO
  void setChangedStateToCamGPIO(){
    if (cam_gpio_state != cam_gpio_state_prev){
      std::cout << "CHANGED CAM GPIO STATUS ";
      printf("[%u]\n", cam_gpio_state);
      cam_gpio_state_prev = cam_gpio_state;
      data_for_gpio_is_changed = true;
      gpioSetMode(gpio_cam_pin, PI_OUTPUT);
      if      (cam_gpio_state == 0) gpioWrite(gpio_cam_pin, PI_HIGH);
      else if (cam_gpio_state == 1) gpioWrite(gpio_cam_pin, PI_LOW);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // получение актуальных данных с пина GPIO
  void getCurrentCamGPIOStatus(){
    if (!data_for_gpio_is_changed) return;
    data_for_gpio_is_changed = false;
    cam_status_from_gpio = gpioRead(gpio_cam_pin);
    std::cout << "CAM GPIO STATUS from PI";
    printf("[%u]\n", cam_status_from_gpio);
    if      (cam_status_from_gpio == 0) cam_status_from_gpio = 1;
    else if (cam_status_from_gpio == 1) cam_status_from_gpio = 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // отправка полученных данных c пина GPIO в ROS-топик(2) для последующей отправки по UDP
  void sendMsgFromCamGPIO(){
    std_msgs::UInt8 sendMsgFromCamGPIOTopic;
    sendMsgFromCamGPIOTopic.data = cam_status_from_gpio;
    cam_gpio_state_pub.publish(sendMsgFromCamGPIOTopic);
  }
  
};

int main(int argc, char** argv)
{

  if (gpioInitialise() == PI_INIT_FAILED) {
    std::cout << "ERROR: Failed to initialize the GPIO interface." << std::endl;
    return 1;
  }

  ros::param::get("/_gpio_cam_pin", gpio_cam_pin);

  try{
    std::cout << "\n\033[1;32m╔═══════════════════════════════════════╗\033[0m"
              << "\n\033[1;32m║       Cam GPIO Node is running!       ║\033[0m" 
              << "\n\033[1;32m╚═══════════════════════════════════════╝\033[0m\n\n";
    ros::init(argc, argv, "Сam_GPIO_Node");
    Cam_GPIO_Node camGPIONode;
    while(ros::ok()){
      camGPIONode.gpio_process();
      ros::spinOnce();
    }
    gpioSetMode(gpio_cam_pin, PI_INPUT);
    gpioTerminate();
  } catch (std::exception e){
      std::cerr << "\nExeption: " << e.what() << std::endl;
  }
  return 0;
};