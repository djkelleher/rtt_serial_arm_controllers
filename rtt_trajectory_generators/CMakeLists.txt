cmake_minimum_required(VERSION 2.8.3)

project(rtt_trajectory_generators)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    message_generation
    roscpp
    rospy
    rtt_tools
    rtt_ros
    rtt_roscomm
    rtt_rosclock
    rtt_rosnode
    rtt_geometry_msgs
    rtt_std_msgs
    trac_ik_lib
    kdl_conversions
    geometry_msgs
    tf2_kdl
    tf2_ros
    tf2_geometry_msgs
)

catkin_python_setup()

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
      include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

add_message_files(
   FILES
   CartesianTrajectory.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

ros_generate_rtt_typekit(rtt_trajectory_generators)

set(COMPONENT_SOURCES
	src/random_waypoint_generator.cpp
	src/random_waypoint_array_generator.cpp
        src/cartesian_trajectory_generator.cpp
        src/joint_space_velocity_profiles.cpp
)
set(PYTHON_SCRIPTS
    src/Pose_publisher1.py
    src/Pose_publisher2.py
    src/PoseArray_publisher1.py
    src/PoseArray_publisher2.py
)

orocos_component(${PROJECT_NAME} ${COMPONENT_SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${USE_OROCOS_LIBRARIES}
   ${trac_ik_lib_LIBRARIES}
)

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package(INCLUDE_DIRS include)

catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

