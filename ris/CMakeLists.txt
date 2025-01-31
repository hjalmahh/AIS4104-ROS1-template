cmake_minimum_required(VERSION 3.10)
project(ris)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE ON)

add_compile_options(-fPIC -Wall -pedantic)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    roslib
    ris_msgs
    std_msgs
    message_runtime
)

catkin_package(
    CATKIN_DEPENDS message_runtime ris_msgs
)

add_subdirectory(src)
