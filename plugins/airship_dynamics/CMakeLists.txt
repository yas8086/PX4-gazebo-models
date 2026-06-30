cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(AirshipDynamicsPlugin VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(sdformat14 REQUIRED)

add_library(AirshipDynamics SHARED
  AirshipDynamics.cc
)

target_include_directories(AirshipDynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(AirshipDynamics
  gz-sim8::core
  gz-plugin2::loader
  gz-math7::gz-math7
  gz-transport13::gz-transport13
)

install(TARGETS AirshipDynamics
  LIBRARY DESTINATION lib
)
