cmake_minimum_required(VERSION 3.14)

PROJECT(SYSFSPWM CXX)

include(GNUInstallDirs)

#SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(CMAKE_BUILD_TYPE Debug)

ADD_DEFINITIONS(-std=gnu++17)
ADD_DEFINITIONS(-Wall)
ADD_DEFINITIONS(-Wno-psabi)


#SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

set(SYSFSPWM_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/pwm.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/pwmchip.cpp"
)

set(SYSFSPWM_HEADER
    "${CMAKE_CURRENT_SOURCE_DIR}/include/sysfspwm.hpp"
)

# add libsmip library as target
add_library(sysfspwm STATIC ${SYSFSPWM_SOURCE_FILES} ${SYSFSPWM_HEADER})

add_dependencies(sysfspwm udevpp)

# Include the header 
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(sysfspwm
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>  # For smip_export.h
)

target_link_libraries(sysfspwm udevpp)
