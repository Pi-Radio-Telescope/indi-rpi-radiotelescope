cmake_minimum_required(VERSION 2.6)

PROJECT(RT-Scripts)

include(GNUInstallDirs)

#SET(CMAKE_VERBOSE_MAKEFILE ON)

#SET(CMAKE_BUILD_TYPE Debug)

# the executables are shell scripts and not built by cmake
#ADD_EXECUTABLE(rt_transitscan.sh IMPORTED)
#ADD_EXECUTABLE(rt_tracking.sh IMPORTED)
#ADD_EXECUTABLE(rt_scan_hor.sh IMPORTED)
#ADD_EXECUTABLE(rt_scan_equ.sh IMPORTED)

# tell cmake where to install our executables
install(PROGRAMS 
rt_transitscan 
rt_tracking
rt_scan_hor
rt_scan_equ
rt_ads1115_measurement
rt_mqtt_to_ratsche
rt_ratsche_to_mqtt
DESTINATION bin)
