cmake_minimum_required(VERSION 2.8.3)
project(rtt_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    message_generation
    rtt_tools
    rtt_trajectory_generators
    roscpp
    rtt_ros
    rtt_roscomm
    rtt_rosclock
    rtt_rosnode
    eigen_conversions
    kdl_conversions
    std_msgs
    geometry_msgs
    rtt_geometry_msgs
    rtt_std_msgs
    trac_ik_lib
    tf2_kdl
    tf2_ros
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
      include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

add_message_files(
   FILES
   JointStates.msg
   TorqueCommands.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

ros_generate_rtt_typekit(rtt_controllers)

set(COMPONENT_SOURCES
    src/chainfksolveracc_recursive.cpp
    src/pid_joint_space.cpp
    src/pid_task_space.cpp
    src/computed_torque_joint_space.cpp
    src/operational_space_control.cpp
)

orocos_component(${PROJECT_NAME} ${COMPONENT_SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${USE_OROCOS_LIBRARIES}
)

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package(INCLUDE_DIRS include)

install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

