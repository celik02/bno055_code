cmake_minimum_required(VERSION 3.10)

## setting the project name
project(READ_IMU)

#find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )

## add the executable
add_executable(READ_IMU  main.cpp imu.h imu.cpp i2c.c i2c.h)

#target_link_libraries(READ_IMU  ${OpenCV_LIBS} )
# target_link_libraries (READ_IMU OpenCV)