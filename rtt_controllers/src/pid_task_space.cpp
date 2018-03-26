#include "rtt_controllers/pid_task_space.h"

// Create a new instance of PIDtaskSpace. The PreOperational argument forces you to call configureHook().
PIDtaskSpace::PIDtaskSpace(const std::string &name) : RTT::TaskContext(name, PreOperational)
{
   // Add RTT ports.
   addPort("current_joint_states", inPort_CurrentJointStates);
   addPort("desired_cartesian_trajectory", inPort_CartesianTrajectory);
   addPort("torque_commands", outPort_TorqueCommands);
   addPort("start_pose_to_trajec_gen", outPort_TrajectoryGenerator);

   // Add RTT properties.
   addProperty("task_P_gains", Kp).doc("propertional gains");
   addProperty("task_I_gains", Ki).doc("integral gains");
   addProperty("task_D_gains", Kd).doc("derivative gians");
   addProperty("robot_description", robot_description).doc("URDF or SDF");
   addProperty("root_link",root_link).doc("first link in KDL chain");
   addProperty("tip_link",tip_link).doc("last link in KDL chain");
   addProperty("end_effector_frame_name", ee_frame_name).doc("name of end effector frame");
   addProperty("joint_start_pose", joint_start_pos).doc("array of joint start positions");
}

// Configure this component.
bool PIDtaskSpace::configureHook()
{
   // Abort if any ports are not connected.
   if(!inPort_CurrentJointStates.connected()){
      RTT::log(RTT::Error) << "inPort_CurrentJointStates is not connected!" << RTT::endlog();
      return false;
   }
   if(!inPort_CartesianTrajectory.connected()){
      RTT::log(RTT::Error) << "inPort_CartesianTrajectory is not connected!" << RTT::endlog();
      return false;
   }
   if(!outPort_TorqueCommands.connected()){
      RTT::log(RTT::Error) << "outPort_TorqueCommands is not connected!" << RTT::endlog();
      return false;
   }
   if(!outPort_TrajectoryGenerator.connected()){
      RTT::log(RTT::Error) << "outPort_TrajectoryGenerator is not connected!" << RTT::endlog();
      return false;
   }
   // Parse parameters.
   rtt_tools::getKDLChain(kdl_chain, this);
   rtt_tools::getTaskSpacePGains(Kp, this);
   rtt_tools::getTaskSpaceIGains(Ki, this);
   rtt_tools::getTaskSpaceDGains(Kd, this);
   rtt_tools::getJointStartPose(kdl_chain, joint_start_pos);
   rtt_tools::getEndEffectorName(ee_frame_name, this);
   rtt_tools::getSegmentIndex(ee_frame_index, ee_frame_name, kdl_chain);

   // Create solvers.
   fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
   fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain));
   jacobian_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));

   // Find end effector start pose from joint start pose array.
   fk_pos_solver->JntToCart(joint_start_pos, ee_start_pose_frame, ee_frame_index);

   // Convert KDL frame to pose message.
   tf::poseKDLToMsg(ee_start_pose_frame, ee_start_pose_msg);

   // Push pose message into pose array for port compatibility.
   ee_start_pose_to_port.poses.push_back(ee_start_pose_msg);

   // Resize.
   Kp.resize(kdl_chain.getNrOfJoints());
   Ki.resize(kdl_chain.getNrOfJoints());
   Kd.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.position.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.velocity.resize(kdl_chain.getNrOfJoints());
   joint_states.q.resize(kdl_chain.getNrOfJoints());
   joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   joint_start_pos.resize(kdl_chain.getNrOfJoints());
   torque_commands_msg.torque.resize(kdl_chain.getNrOfJoints());
   torque_commands.resize(kdl_chain.getNrOfJoints());
   jacobian.resize(kdl_chain.getNrOfJoints());
   jacobian_transpose(jacobian.rows(), jacobian.columns());

   return true;
}

// Start this component.
bool PIDtaskSpace::startHook()
{
   return true;
}

// Run update loop.
void PIDtaskSpace::updateHook()
{
   // Use new data if available.
   if(inPort_CartesianTrajectory.read(cart_trajectory) == RTT::NewData
      && !cart_trajectory.is_last_trajectory_msg.data){

      tf::poseMsgToKDL(cart_trajectory.position, desired_task_pos);
      tf::twistMsgToKDL(cart_trajectory.velocity, desired_task_vel);
   }
   else{
      // Pause briefly to wait for new trajectory.
      new_trajectory = false;
      for(int i=0; i<25; ++i){
	 std::this_thread::sleep_for(std::chrono::microseconds(2));
	 if(inPort_CartesianTrajectory.read(cart_trajectory) == RTT::NewData
	    && !cart_trajectory.is_last_trajectory_msg.data){
	    new_trajectory = true;
	    break;
	 }
      }
      // If we still don't have a new trajectory, set the robot to its starting position.
      if(!new_trajectory)
	 outPort_TrajectoryGenerator.write(ee_start_pose_to_port);
   }
   // Read current joint states from robot or Gazebo.
   inPort_CurrentJointStates.read(joint_states_msg);

   // Map data from messages to JntArrayVel for compatibility with fk velocity solvers.
   joint_states.q.data = Eigen::VectorXd::Map(joint_states_msg.position.data(), joint_states_msg.position.size());
   joint_states.qdot.data = Eigen::VectorXd::Map(joint_states_msg.velocity.data(), joint_states_msg.velocity.size());

   // Get current end effector pose.
   fk_pos_solver->JntToCart(joint_states.q, current_task_pos, ee_frame_index);

   // Solve for current end effector velocity.
   fk_vel_solver->JntToCart(joint_states, current_task_vel_solver, ee_frame_index);

   // Get current end effector velocity.
   current_task_vel = current_task_vel_solver.GetTwist();

   // Get current position error.
   pose_error = diff(desired_task_pos, current_task_pos);

   // Make a vector of PID commands.
   for(int i=0; i<6; ++i)
      PID_commands(i) = pose_error(i)*Kp[i] + pose_error(i)*Ki[i] + (desired_task_vel(i) - current_task_vel(i))*Kd[i];

   // Convert KDL::Twist to Eigen::Matrix so we can use matrix multiplications.
   tf::twistKDLToEigen(PID_commands, pid_commands);

   // Get jacobian.
   jacobian_solver->JntToJac(joint_states.q, jacobian, ee_frame_index);

   // Convert jacobian to Eigen::Matrix so transpose operation can be used.
   jacobian_transpose = jacobian.data;

   // transpose jacobian.
   jacobian_transpose.transposeInPlace();

   // Get torque commands.
   torque_commands = jacobian_transpose * pid_commands;
   
   // Convert to messaage.
   for(int i=0; i<torque_commands.rows(); ++i)
      torque_commands_msg.torque[i] = torque_commands[i];
   
   // Set timestamp.
   torque_commands_msg.header.stamp = rtt_rosclock::rtt_now();

   // Write torque commands to port.
   outPort_TorqueCommands.write(torque_commands_msg);
}
