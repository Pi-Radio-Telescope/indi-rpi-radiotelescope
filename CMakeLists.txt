cmake_minimum_required(VERSION 3.14)

PROJECT(PiRaTe)

#SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(CMAKE_BUILD_TYPE Debug)

#ADD_DEFINITIONS(-std=gnu++17)
#ADD_DEFINITIONS(-Wall)
#ADD_DEFINITIONS(-Wno-psabi)

include(GNUInstallDirs)
include(FetchContent)
find_package(PkgConfig REQUIRED)

pkg_check_modules(UDEV REQUIRED libudev)

if(UDEV_FOUND)
    message(STATUS "Found libudev via pkg-config")
    include_directories(${UDEV_INCLUDE_DIRS})
    link_directories(${UDEV_LIBRARY_DIRS})
    add_definitions(${UDEV_CFLAGS_OTHER})
#    target_link_libraries(your_target PRIVATE ${UDEV_LIBRARIES})
else()
    message(FATAL_ERROR "libudev not found via pkg-config")
endif()

# Fetch udevpp
FetchContent_Declare(
    udevpp
    GIT_REPOSITORY https://github.com/danjperron/libudevpp.git
    GIT_TAG        cmakesubmod # or use a specific commit hash or tag
)
FetchContent_MakeAvailable(udevpp)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.8.0 # or use a specific commit hash or tag
)
FetchContent_MakeAvailable(googletest)


SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_subdirectory(sysfspwm)
add_subdirectory(pirt)
add_subdirectory(ratsche)
add_subdirectory(macros)
add_subdirectory(config)

set(ALL_FILES
    "${PIRT_SOURCE_FILES}"
    "${PIRT_HEADER_FILES}"
)

add_custom_target(clangformat COMMAND clang-format -style=WebKit -i ${ALL_FILES})
