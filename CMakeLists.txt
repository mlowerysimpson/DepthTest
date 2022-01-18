cmake_minimum_required(VERSION 2.8)
project(DepthTest)
# Locate libraries and headers
find_library(wiringPi_LIB wiringPi)
find_package(Threads REQUIRED)

# Include headers
include_directories(${WIRINGPI_INCLUDE_DIRS}) 

#the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "./*.cpp")
add_executable(DepthTest ${SOURCES} )


# Link against libraries
target_link_libraries(DepthTest ${wiringPi_LIB} )
target_link_libraries(DepthTest ${CMAKE_THREAD_LIBS_INIT})
