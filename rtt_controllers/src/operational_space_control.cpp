#include "rtt_controllers/operational_space_control.h"

// Create a new instance of OperationalSpaceControl. The PreOperational argument forces you to call configureHook().
OperationalSpaceControl::OperationalSpaceControl(const std::string &name) : RTT::TaskContext(name, PreOperational)
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
   addProperty("root_link", root_link).doc("first link of KDL chain");
   addProperty("tip_link", tip_link).doc("last link of KDL chain");
   addProperty("end_effector_frame_name", ee_frame_name).doc("name of end effector frame");
   addProperty("joint_start_pose", joint_start_pose).doc("array of joint start positions");
}

// Configure this component.
bool OperationalSpaceControl::configureHook()
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
   rtt_tools::getEndEffectorName(ee_frame_name, this);
   rtt_tools::getSegmentIndex(ee_frame_index, ee_frame_name, kdl_chain);
   rtt_tools::getJointStartPose(kdl_chain, joint_start_pose);

   // Set gravity
   grav_vec.Zero();
   grav_vec(2)=-9.81; 
   
   // Create solvers.
   chain_dynamic_params.reset(new KDL::ChainDynParam(kdl_chain, grav_vec));
   jacobian_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));
   fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
   fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain));
   fk_acc_solver.reset(new KDL::ChainFkSolverAcc_recursive(kdl_chain));

   // Find end effector start pose from joint start pose array.
   fk_pos_solver->JntToCart(joint_start_pose, ee_start_pose, ee_frame_index);
   
   // Convert KDL frame to pose message.
   tf::poseKDLToMsg(ee_start_pose, ee_start_pose_msg);
   
   // Push pose message into pose array for port compatibility.
   ee_start_pose_to_port.poses.push_back(ee_start_pose_msg);

   // Resize.
   torque_commands.resize(kdl_chain.getNrOfJoints());
   torque_commands_msg.torque.reserve(kdl_chain.getNrOfJoints());
   joint_states_msg.position.reserve(kdl_chain.getNrOfJoints());
   joint_states_msg.velocity.reserve(kdl_chain.getNrOfJoints());
   joint_pos.resize(kdl_chain.getNrOfJoints());
   joint_vel_array.q.resize(kdl_chain.getNrOfJoints());
   joint_vel_array.qdot.resize(kdl_chain.getNrOfJoints());
   last_joint_vel_array.qdot.data.setZero(kdl_chain.getNrOfJoints());
   joint_acc_array.q.resize(kdl_chain.getNrOfJoints());
   joint_acc_array.qdot.resize(kdl_chain.getNrOfJoints());
   joint_acc_array.qdotdot.resize(kdl_chain.getNrOfJoints());
   joint_space_mass_mat.resize(kdl_chain.getNrOfJoints());
   joint_space_mass_mat_inv.resize(kdl_chain.getNrOfJoints());
   gravity.resize(kdl_chain.getNrOfJoints());
   jacobian.resize(kdl_chain.getNrOfJoints());
   
   return true;
}

// Start this component.
bool OperationalSpaceControl::startHook()
{
   return true;
}

// Run update loop.
void OperationalSpaceControl::updateHook()
{
   // Use new data if available.
   if(inPort_CartesianTrajectory.read(cart_trajectory) == RTT::NewData &&
      !cart_trajectory.is_last_trajectory_msg.data){
      
      tf::poseMsgToKDL(cart_trajectory.position, desired_task_pos);
      tf::twistMsgToKDL(cart_trajectory.velocity, desired_task_vel);
      tf::twistMsgToKDL(cart_trajectory.acceleration, desired_task_acc);
   }
   else{
      // Pause briefly to wait for new trajectory.
      new_trajectory = false;
      for(int i=0; i<25; ++i){
	 std::this_thread::sleep_for(std::chrono::milliseconds(2));
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
   
   // Map data from joint_states to JntArrayVel for compatibility with FK velocity solver.
   joint_vel_array.q.data = Eigen::VectorXd::Map(joint_states_msg.position.data(), joint_states_msg.position.size());
   joint_vel_array.qdot.data = Eigen::VectorXd::Map(joint_states_msg.velocity.data(), joint_states_msg.velocity.size());
   
   // Assign data from joint_vel_array to JntArrayAcc for compatibility with FK acceleration solver.
   joint_acc_array.q.data = joint_vel_array.q.data;
   joint_acc_array.qdot.data = joint_vel_array.qdot.data;
   
   // Use numerical differentiation to find acceleration.
   for(int i=0; i<joint_vel_array.qdot.rows(); ++i)
      joint_acc_array.qdotdot.data[i] = (joint_vel_array.qdot.data[i] - last_joint_vel_array.qdot.data[i]) / getPeriod();
   
   // Save for next loop.
   last_joint_vel_array.qdot.data = joint_vel_array.qdot.data;
   
   // Get current end effector pose.
   fk_pos_solver->JntToCart(joint_pos, current_task_pos, ee_frame_index);

   // Solve for current end effector velocity.
   fk_vel_solver->JntToCart(joint_vel_array, current_task_vel_solver, ee_frame_index);
   
   // Get current end effector velocity.
   current_task_vel = current_task_vel_solver.GetTwist();

   // Solve for current end effector acceleration.
   fk_acc_solver->JntToCart(joint_acc_array, current_task_acc_solver, ee_frame_index);

   // Get current end effector acceleration;
   current_task_acc = current_task_acc_solver.GetTwist();

   // Get current position error.
   pose_error = diff(desired_task_pos, current_task_pos);

   // Get gravity forces.
   chain_dynamic_params->JntToGravity(joint_pos, gravity);

   // Get joint space mass matrix.
   chain_dynamic_params->JntToMass(joint_pos, joint_space_mass_mat);

   // Get the operational space mass matrix.
   joint_space_mass_mat.data =  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>(joint_space_mass_mat.data.diagonal().asDiagonal());

   // Invert mass matrix.
   joint_space_mass_mat_inv.data = joint_space_mass_mat.data.inverse();

   // Sove jacobian.
   jacobian_solver->JntToJac(joint_pos, jacobian, ee_frame_index);
   
   // Find operational space mass matrix.
   op_space_mass_mat = (jacobian.data * joint_space_mass_mat_inv.data * jacobian.data.transpose()).inverse();

   // Convert jacobian to Eigen::Matrix so transpose operation can be used.
   jacobian_transpose = jacobian.data;
   
   // transpose jacobian.
   jacobian_transpose.transposeInPlace();

   // Make a vector of PID commands.
   for(int i = 0; i < 6; ++i)
      PID_commands(i) = pose_error(i)*Kp[i] + (desired_task_vel(i) - current_task_vel(i))*Kd[i]
      + (desired_task_acc(i) - current_task_acc(i))*Ki[i];
   
   // Convert KDL::Twist to Eigen::Matrix so we can use matrix multiplications.
   tf::twistKDLToEigen(PID_commands, pid_commands);
   
   // Get torque commands.
   torque_commands = jacobian_transpose * op_space_mass_mat * pid_commands + gravity.data;
   
   // Convert to message.
   for(int i=0; i<torque_commands.rows(); ++i)
      torque_commands_msg.torque[i] = torque_commands[i];
   
   // Write torque commands to port.
   outPort_TorqueCommands.write(torque_commands_msg);
}
