cmake_minimum_required(VERSION 2.8.3)
project(rtt_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    roscpp
    rtt_ros
    rtt_rosparam
    rtt_rosnode
    trac_ik_lib
)

find_package(orocos_kdl REQUIRED)

include_directories(
      include
    ${catkin_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${TINYXML_INCLUDE_DIRS}
)

## Orocos library
orocos_library(${PROJECT_NAME} src/rtt_tools.cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${USE_OROCOS_LIBRARIES}
   ${trac_ik_lib_LIBRARIES}
   ${TINYXML_LIBRARIES}
)

## Generate install targets for header files
orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package(INCLUDE_DIRS include)

