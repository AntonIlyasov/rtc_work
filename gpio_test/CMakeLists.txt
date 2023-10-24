cmake_minimum_required(VERSION 2.8)
PROJECT (gpio_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14")
add_executable( gpio_test gpio_test.cpp )

