cmake_minimum_required(VERSION 2.8.3)

project(rtt_gazebo_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    roscpp
    rtt_tools
    rtt_ros
    rtt_roscomm
    rtt_rosnode
    std_msgs
    geometry_msgs
    rtt_geometry_msgs
    rtt_std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

if(gazebo_VERSION_MAJOR EQUAL 7 OR LESS)
  message(STATUS "Gazebo version 8 or greater is required. Package will not be built.")
  return()
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

ros_generate_rtt_typekit(rtt_gazebo_interface)

orocos_component(${PROJECT_NAME} src/rtt_gazebo_interface.cpp)

target_link_libraries(${PROJECT_NAME}
    ${GAZEBO_LIBRARIES} 
    ${CATKIN_LIBRARIES} 
    ${orocos_kdl_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
)

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package(INCLUDE_DIRS include)

