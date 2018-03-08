cmake_minimum_required(VERSION 3.8)
project(Linjefollow)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package( OpenCV REQUIRED )

set(SOURCE_FILES main.cpp)

add_executable(Linjefollow main.cpp motor_virt.cpp motor_virt.hpp)
#add_executable(Linjefollow main.cpp motor.cpp motor.hpp)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries( Linjefollow ${OpenCV_LIBS} )
