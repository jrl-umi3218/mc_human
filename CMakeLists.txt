cmake_minimum_required(VERSION 3.1)
project(mc_human LANGUAGES CXX VERSION 1.0.0)
enable_testing()
set(CMAKE_CXX_STANDARD 11)

include(CMakePackageConfigHelpers)
set(VERSION_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(PROJECT_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
set(CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}")
write_basic_package_version_file(
  ${VERSION_CONFIG}
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in" "${PROJECT_CONFIG}" INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION "${CONFIG_INSTALL_DIR}")

find_package(mc_rtc 1.4.0 REQUIRED)
find_description_package(human_description REQUIRED)

message("HUMAN_DESCRIPTION_PATH: ${HUMAN_DESCRIPTION_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_robot(human src/human.cpp src/human.h)
target_include_directories(human PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
