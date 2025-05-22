cmake_minimum_required(VERSION 3.14)

# define the project name
project(indi-pirt C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

add_compile_options(-Wall -Wextra -Wno-shadow -Wpedantic -Wno-psabi -O3)

# add our cmake_modules folder
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# find our required packages
find_package(PkgConfig REQUIRED)
find_package(INDI REQUIRED)
find_package(Nova REQUIRED)
find_package(ZLIB REQUIRED)
find_package(GSL REQUIRED)
find_library(GPIODCXX gpiodcxx REQUIRED)
find_library(RT rt REQUIRED)

# Ensure the public interface header is visible
#target_include_directories(sysfspwm PUBLIC ${sysfspwm_SOURCE_DIR}/include)

# these will be used to set the version number in config.h and our driver's xml file
set(CDRIVER_VERSION_MAJOR 0)
set(CDRIVER_VERSION_MINOR 3)

# do the replacement in the config.h
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

# do the replacement in the driver's xml file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_pirt.xml.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/indi_pirt.xml
)

# set our include directories to look for header files
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${NOVA_INCLUDE_DIR})
include_directories( ${EV_INCLUDE_DIR})

include(CMakeCommon)

set(SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/axis.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gpioif.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/encoder.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/motordriver.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/i2cdevice.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ads1115.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/rpi_temperatures.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/voltage_monitor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ads1115_measurement.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pirt.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/spidevice.cpp"
)

set(HEADER_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/axis.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/gpioif.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/encoder.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/motordriver.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/i2cdevice.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/ads1115.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/rpi_temperatures.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/voltage_monitor.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/ads1115_measurement.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/pirt.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/utility.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/spidevice.h"
)

set(PIRT_SOURCE_FILES
    "${SOURCE_FILES}"
    PARENT_SCOPE
)

set(PIRT_HEADER_FILES
    "${HEADER_FILES}"
    PARENT_SCOPE
)

# tell cmake to build our executable
add_executable(
    indi_pirt
	${SOURCE_FILES}
)

add_dependencies(indi_pirt udevpp sysfspwm)

add_executable(
    encodertest
	encodertest.cpp
	spidevice.cpp
	encoder.cpp
)

# and link it to these libraries
target_link_libraries(
    indi_pirt
    ${INDI_LIBRARIES}
    ${NOVA_LIBRARIES}
    ${GSL_LIBRARIES}
    rt
    pthread
    udevpp
    sysfspwm
    gpiodcxx
)

target_link_libraries(
    encodertest
    rt
    pthread
)

# tell cmake where to install our executable
install(TARGETS indi_pirt RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/indi_pirt.xml
    DESTINATION ${INDI_DATA_DIR}
)
