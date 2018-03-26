#ifndef PID_JOINT_SPACE_H_
#define PID_JOINT_SPACE_H_

#include <rtt_tools/rtt_tools.h>
#include <rtt_trajectory_generators/joint_space_velocity_profiles.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include <kdl/jntarrayacc.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>

#include "rtt_controllers/JointStates.h"
#include "rtt_controllers/TorqueCommands.h"
#include <geometry_msgs/PoseStamped.h>

#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Core>
#include <memory>
#include <thread>
#include <chrono>

class PIDjointSpace : public RTT::TaskContext{

public:

   PIDjointSpace(const std::string &name);

   bool configureHook();
   bool startHook();
   void updateHook();

   virtual ~PIDjointSpace(){}

protected:

   RTT::InputPort<rtt_controllers::JointStates> inPort_JointStates;
   RTT::InputPort<geometry_msgs::PoseStamped> inPort_TrajectoryGoalPose;

   RTT::OutputPort<rtt_controllers::TorqueCommands> outPort_TorqueCommands;
   
   rtt_controllers::JointStates joint_states_msg;
   rtt_controllers::TorqueCommands torque_commands;
   geometry_msgs::PoseStamped trajectory_goal_pose;
   geometry_msgs::TransformStamped frame_transformation;

   int count;
   bool use_joint_vel_lims, use_joint_acc_lims, transform_points;
   std::vector<double> Kp, Ki, Kd, joint_vel_lims, joint_acc_lims;
   KDL::JntArray joint_start_pos, joint_final_pos;
   KDL::JntArrayVel joint_states, desired_joint_states;
   double current_trajec_time, duration, max_solve_time, error;
   KDL::Chain kdl_chain;
   KDL::Frame trajectory_goal_frame;
   std::unique_ptr<joint_space_vel_prof::velocityProfile> vel_prof;
   std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver;
   std::string robot_description, root_link, tip_link, joint_space_velocity_profile;
   std::string current_frame_id, target_frame_id;
   tf2_ros::Buffer tf2Buffer;
   tf2_ros::TransformListener tf2_listener;
};

ORO_LIST_COMPONENT_TYPE(PIDjointSpace)

#endif 
