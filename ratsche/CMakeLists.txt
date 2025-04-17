cmake_minimum_required(VERSION 2.6)

PROJECT(Ratsche)

include(GNUInstallDirs)

#SET(CMAKE_VERBOSE_MAKEFILE ON)

#The following lines are required to use Dart
#ENABLE_TESTING()
#INCLUDE(Dart)

SET(CMAKE_BUILD_TYPE Debug)

#ADD_DEFINITIONS(-mno-sse2)
ADD_DEFINITIONS(-std=gnu++17)
ADD_DEFINITIONS(-Wall)
ADD_DEFINITIONS(-Wno-psabi)
#ADD_DEFINITIONS(-ffast-math)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

# tell cmake to build our executable
ADD_EXECUTABLE(ratsche 
	ratsche_main.cpp
	basic.cpp
	rttask.cpp
	time.cpp
	astro.cpp
)

TARGET_LINK_LIBRARIES(ratsche
 pthread
)

# tell cmake where to install our executable
install(TARGETS ratsche RUNTIME DESTINATION bin)
install(CODE "execute_process(COMMAND mkdir -p /var/ratsche)")
install(CODE "execute_process(COMMAND chown pi:users /var/ratsche)")
install(CODE "execute_process(COMMAND chmod g+w /var/ratsche)")
