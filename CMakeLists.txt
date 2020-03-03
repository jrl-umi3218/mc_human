cmake_minimum_required(VERSION 3.1)

project(mc-human)

set(CMAKE_CXX_STANDARD 11)

find_package(human_description REQUIRED)
find_package(mc_rtc REQUIRED)

if("${human_description_INSTALL_PREFIX}" STREQUAL "")
  if("${human_description_SOURCE_PREFIX}" STREQUAL "")
    message(FATAL_ERROR "Your human_description package does not define where to find the data")
  endif()
  set(HUMAN_DESCRIPTION_PATH "${human_description_SOURCE_PREFIX}")
else()
  set(HUMAN_DESCRIPTION_PATH "${human_description_INSTALL_PREFIX}")
endif()

message("HUMAN_DESCRIPTION_PATH: ${HUMAN_DESCRIPTION_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_robot(human src/human.cpp src/human.h)
target_include_directories(human PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")

