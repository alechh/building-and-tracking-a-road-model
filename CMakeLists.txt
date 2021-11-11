cmake_minimum_required(VERSION 3.20)
project(test_functions_for_vkr)

set(CMAKE_CXX_STANDARD 14)


add_executable(test_functions_for_vkr main.cpp)


find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( test_functions_for_vkr ${OpenCV_LIBS} )