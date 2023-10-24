cmake_minimum_required(VERSION 2.8)
PROJECT (test_pigpio)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14")

find_package(OpenCV REQUIRED )

add_executable( pigpio_example pigpio_example.cpp )
target_link_libraries(pigpio_example pigpio)

add_executable( pw_switch pw_switch.cpp )
target_link_libraries(pw_switch ${OpenCV_LIBS} )
target_link_libraries(pw_switch pigpio)

add_executable( pw_set pw_set.cpp )
target_link_libraries(pw_set pigpio)