#ifndef OPERATIONAL_SPACE_CONTROL_H_
#define OPERATIONAL_SPACE_CONTROL_H_

#include <rtt_tools/rtt_tools.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include "rtt_controllers/chainfksolveracc_recursive.hpp"
#include <kdl/jntarrayacc.hpp>
#include <kdl/frames.hpp>
#include <kdl/frameacc.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>

#include "rtt_controllers/JointStates.h"
#include "rtt_controllers/TorqueCommands.h"
#include <rtt_trajectory_generators/CartesianTrajectory.h>
#include <geometry_msgs/PoseArray.h>

#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>

#include <Eigen/Core>
#include <memory>
#include <thread>
#include <chrono>

class OperationalSpaceControl : public RTT::TaskContext{

public:

   OperationalSpaceControl(const std::string &name);

   bool configureHook();
   bool startHook();
   void updateHook();

   virtual ~OperationalSpaceControl(){}

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

   std::string robot_description, ee_frame_name, root_link, tip_link;
   KDL::JntArray joint_start_pose, joint_pos, gravity, coriolis;
   KDL::JntArrayVel joint_vel_array, last_joint_vel_array;
   KDL::JntArrayAcc joint_acc_array;
   KDL::FrameVel current_task_vel_solver;
   KDL::FrameAcc current_task_acc_solver;
   int ee_frame_index;
   bool new_trajectory;
   std::vector<double> Kp, Ki, Kd;
   KDL::Vector grav_vec;
   KDL::Chain kdl_chain;
   std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
   std::unique_ptr<KDL::ChainDynParam> chain_dynamic_params;
   std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
   std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
   std::unique_ptr<KDL::ChainFkSolverAcc_recursive> fk_acc_solver;
   KDL::JntSpaceInertiaMatrix joint_space_mass_mat, joint_space_mass_mat_inv;
   KDL::Frame current_task_pos, desired_task_pos, ee_start_pose;
   KDL::Twist current_task_vel, current_task_acc, pose_error;
   KDL::Twist desired_task_vel, desired_task_acc, PID_commands;
   Eigen::VectorXd torque_commands;
   KDL::Jacobian jacobian;
   Eigen::MatrixXd jacobian_transpose, op_space_mass_mat;
   Eigen::Matrix<double,6,1> pid_commands;
};

ORO_LIST_COMPONENT_TYPE(OperationalSpaceControl)

#endif
