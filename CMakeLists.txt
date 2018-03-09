cmake_minimum_required(VERSION 3.5)
project(Linjefollow)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package( OpenCV REQUIRED )

set(SOURCE_FILES main.cpp)

IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
add_executable(Linjefollow main.cpp motor.cpp motor.hpp)
ELSE(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
add_executable(Linjefollow main.cpp motor_virt.cpp motor_virt.hpp)
ENDIF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries( Linjefollow ${OpenCV_LIBS} )
