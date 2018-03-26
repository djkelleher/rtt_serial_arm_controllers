#ifndef COMPUTED_TORQUE_JOINT_SPACE_H_
#define COMPUTED_TORQUE_JOINT_SPACE_H_

#include <rtt_tools/rtt_tools.h>
#include <rtt_trajectory_generators/joint_space_velocity_profiles.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>

#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_listener.h>

#include "rtt_controllers/JointStates.h"
#include "rtt_controllers/TorqueCommands.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/Core>
#include <memory>
#include <thread>
#include <chrono>

class ComputedTorqueJointSpace : public RTT::TaskContext{

public:

   ComputedTorqueJointSpace(const std::string &name);

   bool configureHook();
   bool startHook();
   void updateHook();

   virtual ~ComputedTorqueJointSpace(){}

protected:
   
   RTT::InputPort<geometry_msgs::PoseStamped> inPort_TrajectoryGoalPose;
   RTT::InputPort<rtt_controllers::JointStates> inPort_JointStates;

   RTT::OutputPort<rtt_controllers::TorqueCommands> outPort_TorqueCommands;
   
   rtt_controllers::TorqueCommands torque_commands_msg;
   rtt_controllers::JointStates joint_states_msg;
   geometry_msgs::TransformStamped frame_transformation;
   geometry_msgs::PoseStamped trajectory_goal_pose;

   int count;
   bool use_joint_vel_lims, use_joint_acc_lims;
   double trajectory_timer, trajectory_duration, max_solve_time, error;
   std::vector<double> Kp, Kd, joint_vel_lims, joint_acc_lims;
   KDL::Vector grav_vec;
   KDL::JntArrayVel joint_states;
   KDL::JntArrayAcc desired_joint_states;
   KDL::JntArray joint_start_pos, joint_final_pos;
   KDL::JntArray coriolis, gravity, dynam_commands, PID_commands;
   KDL::Chain kdl_chain;
   std::unique_ptr<KDL::ChainDynParam> chain_dynamic_params;
   KDL::JntSpaceInertiaMatrix mass_mat;
   KDL::Frame trajectory_goal_frame;
   std::unique_ptr<joint_space_vel_prof::velocityProfile> vel_prof;
   std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver;
   std::string robot_description, root_link, tip_link, current_frame_id, target_frame_id;
   std::string vel_prof_name, joint_space_velocity_profile;
   Eigen::VectorXd torque_commands;
   tf2_ros::Buffer tf2Buffer;
   tf2_ros::TransformListener tf2_listener;
};

ORO_LIST_COMPONENT_TYPE(ComputedTorqueJointSpace)

#endif 
