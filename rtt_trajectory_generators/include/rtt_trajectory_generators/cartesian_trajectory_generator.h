#ifndef CARTESIAN_TRAJECTORY_GENERATOR_H_
#define CARTESIAN_TRAJECTORY_GENERATOR_H_

#include <rtt_tools/rtt_tools.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_point.hpp>
#include <kdl/framevel.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/utilities/error.h>

#include "rtt_trajectory_generators/CartesianTrajectory.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf_conversions/tf_kdl.h>
#include <tf2/transform_datatypes.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>

#include <rtt_rosclock/rtt_rosclock.h>
#include <memory>
#include <chrono>
#include <thread>

class CartesianTrajectory : public RTT::TaskContext
{
public:
  
    CartesianTrajectory(const std::string &name);

    bool configureHook();
    bool startHook();
    void updateHook();
    bool computeTrajectory();

    virtual ~CartesianTrajectory(){}

protected:
    
    RTT::InputPort<geometry_msgs::PoseArray> inPort_WaypointVector;
    RTT::OutputPort<rtt_trajectory_generators::CartesianTrajectory> outPort_CartesianTrajectory;
    
    geometry_msgs::PoseArray waypoint_array;
    rtt_trajectory_generators::CartesianTrajectory cart_trajectory_msg;
    geometry_msgs::TransformStamped frame_transformation;

    std::string current_frame_id, target_frame_id, task_space_velocity_profile;
    double current_trajec_time, remaining_trajec_time, task_space_vel_limit, task_space_acc_limit;
    double task_trajectory_corner_radius, task_trajectory_equivalent_radius;
    KDL::Path_RoundedComposite *path;
    KDL::Trajectory *traject;
    KDL::Trajectory_Composite *comp_trajec;
    KDL::VelocityProfile *vel_profile;
    KDL::RotationalInterpolation_SingleAxis *interpolator;
    KDL::Frame current_pos, waypoint_frame;
    KDL::Twist current_vel, current_acc;
    std::vector<KDL::Frame> frame_waypoint_vector;
    tf2_ros::Buffer tf2Buffer;
    tf2_ros::TransformListener tf2_listener;
};

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(CartesianTrajectory)

#endif

