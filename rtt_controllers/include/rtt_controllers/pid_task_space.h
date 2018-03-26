#ifndef PID_TASK_SPACE_H_
#define PID_TASK_SPACE_H_

#include <rtt_tools/rtt_tools.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt_rosclock/rtt_rosclock.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>

#include "rtt_controllers/JointStates.h"
#include "rtt_controllers/TorqueCommands.h"
#include <rtt_trajectory_generators/CartesianTrajectory.h>
#include <geometry_msgs/PoseArray.h>

#include <Eigen/Core>
#include <memory>
#include <thread>
#include <chrono>

class PIDtaskSpace : public RTT::TaskContext{

public:

   PIDtaskSpace(const std::string &name);

   bool configureHook();
   bool startHook();
   void updateHook();

   virtual ~PIDtaskSpace(){}

protected:

   RTT::InputPort<rtt_trajectory_generators::CartesianTrajectory> inPort_CartesianTrajectory;
   RTT::InputPort<rtt_controllers::JointStates> inPort_CurrentJointStates;

   RTT::OutputPort<rtt_controllers::TorqueCommands> outPort_TorqueCommands;
   RTT::OutputPort<geometry_msgs::PoseArray> outPort_TrajectoryGenerator;
   
   rtt_controllers::JointStates joint_states_msg;
   rtt_controllers::TorqueCommands torque_commands_msg;
   rtt_trajectory_generators::CartesianTrajectory cart_trajectory;
   geometry_msgs::PoseArray ee_start_pose_to_port;
   geometry_msgs::Pose ee_start_pose_msg;

   KDL::Chain kdl_chain;
   std::string robot_description, root_link, tip_link, ee_frame_name;
   int ee_frame_index;
   bool new_trajectory;
   std::vector<double> Kp, Ki, Kd;
   KDL::JntArray joint_start_pos;
   KDL::JntArrayVel joint_states;
   KDL::Frame current_task_pos, desired_task_pos, ee_start_pose_frame;
   KDL::Twist current_task_vel, desired_task_vel, pose_error, PID_commands;
   KDL::FrameVel current_task_vel_solver;
   std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
   std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
   std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
   KDL::Jacobian jacobian;
   Eigen::MatrixXd jacobian_transpose;
   Eigen::VectorXd torque_commands;
   Eigen::Matrix<double,6,1> pid_commands;
};

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(PIDtaskSpace)

#endif 
