#include "rtt_controllers/pid_joint_space.h"

// Create a new instance of PIDjointSpace. The PreOperational argument forces you to call configureHook().
PIDjointSpace::PIDjointSpace(const std::string &name) : RTT::TaskContext(name, PreOperational), tf2_listener(tf2Buffer)
{
   // Add RTT ports.
   addPort("joint_states", inPort_JointStates);
   addPort("trajectory_goal_pose", inPort_TrajectoryGoalPose);
   addPort("torque_commands", outPort_TorqueCommands);

   // Add RTT properties.
   addProperty("joint_P_gains", Kp).doc("propertional gains");
   addProperty("joint_I_gains", Ki).doc("integral gains");
   addProperty("joint_D_gains", Kd).doc("derivative gians");
   addProperty("robot_description", robot_description).doc("URDF or SDF");
   addProperty("root_link", root_link).doc("first link in KDL chain");
   addProperty("tip_link", tip_link).doc("last link in KDL chain");
   addProperty("joint_start_pose", joint_start_pos).doc("array of joint start positions");
   addProperty("joint_space_velocity_profile", joint_space_velocity_profile).doc("selected joint space velocity profile");
   addProperty("max_solve_time", max_solve_time).doc("max allowable solve time for IK solver.");
   addProperty("error", error).doc("max allowable error in IK solution.");
   addProperty("use_joint_vel_lims", use_joint_vel_lims).doc("use velocity limits to calculate velocity profile");
   addProperty("use_joint_acc_lims", use_joint_acc_lims).doc("use acceleration limits to calculate velocity profile");
   addProperty("joint_vel_lims", joint_vel_lims).doc("maximum joint velocities");
   addProperty("joint_accel_lims", joint_acc_lims).doc("maximum joint accelerations");
   addProperty("frame_id", target_frame_id).doc("reference frame for trajectory waypoints");
}

// Configure this component.
bool PIDjointSpace::configureHook()
{
   // Abort if any ports are not connected.
   if(!inPort_JointStates.connected()){
      RTT::log(RTT::Error) << "inPort_JointStates is not connected!" << RTT::endlog();
      return false;
   }
   if(!inPort_TrajectoryGoalPose.connected()){
      RTT::log(RTT::Error) << "inPort_TrajectoryGoalPose is not connected!" << RTT::endlog();
      return false;
   }
   if(!outPort_TorqueCommands.connected()){
      RTT::log(RTT::Error) << "outPort_TorqueCommands is not connected!" << RTT::endlog();
      return false;
   }
   // Get KDL chain
   rtt_tools::getKDLChain(kdl_chain, this);

   // Resize.
   Kp.reserve(kdl_chain.getNrOfJoints());
   Ki.reserve(kdl_chain.getNrOfJoints());
   Kd.reserve(kdl_chain.getNrOfJoints());
   joint_vel_lims.reserve(kdl_chain.getNrOfJoints());
   joint_acc_lims.reserve(kdl_chain.getNrOfJoints());
   joint_start_pos.resize(kdl_chain.getNrOfJoints());
   joint_final_pos.resize(kdl_chain.getNrOfJoints());
   joint_states.q.resize(kdl_chain.getNrOfJoints());
   joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.q.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.position.reserve(kdl_chain.getNrOfJoints()); 
   joint_states_msg.velocity.reserve(kdl_chain.getNrOfJoints());
   torque_commands.torque.reserve(kdl_chain.getNrOfJoints());
   
   // Parse parameters.
   rtt_tools::getJointSpaceVelProfileName(joint_space_velocity_profile, this);
   joint_space_vel_prof::getJointSpaceVelProfile(joint_space_velocity_profile, kdl_chain, vel_prof, this);
   rtt_tools::getJointPGains(kdl_chain, Kp, this);
   rtt_tools::getJointIGains(kdl_chain, Ki, this);
   rtt_tools::getJointDGains(kdl_chain, Kd, this);
   rtt_tools::getIKSolver(ik_solver, this);
   rtt_tools::getJointStartPose(kdl_chain, joint_start_pos);
   rtt_tools::transformWaypointsToNewFrame(transform_points, this);
   rtt_tools::getFrameID(target_frame_id, this);

   return true;
}

// Start this component.
bool PIDjointSpace::startHook()
{
   current_trajec_time = 0.0;
   duration = 0.0;
   return true;
}

// Run update loop.
void PIDjointSpace::updateHook()
{
   // Read current joint states.
   inPort_JointStates.read(joint_states_msg);
   
   // Map joint state message to KDL JntArrayVel.
   joint_states.q.data = Eigen::VectorXd::Map(joint_states_msg.position.data(), joint_states_msg.position.size());
   joint_states.qdot.data = Eigen::VectorXd::Map(joint_states_msg.velocity.data(), joint_states_msg.velocity.size());
   
   // If the current trajectory duration is over, check for new goal poses.
   if(current_trajec_time >= duration){

      if(inPort_TrajectoryGoalPose.read(trajectory_goal_pose) == RTT::NewData){
	 
	 // Get transformation to target frame.
	 frame_transformation = tf2Buffer.lookupTransform(target_frame_id, current_frame_id, ros::Time(0), ros::Duration(1.0));
	 
	 // Convert transform to KDL frame.
	 trajectory_goal_frame = tf2::transformToKDL(frame_transformation);
	 
	 // Find an IK solution.
	 ik_solver->CartToJnt(joint_states.q, trajectory_goal_frame, joint_final_pos);
      }
      // Set to start position if there are no new goal poses.
      else{
	 // Pause briefly to wait for new trajectory.
	 count=0;

	 while(inPort_TrajectoryGoalPose.read(trajectory_goal_pose) != RTT::NewData){
	    
	    // Sleep.
	    std::this_thread::sleep_for(std::chrono::microseconds(2));
	    
	    // If we still don't have a new trajectory, set the robot to its starting position.
	    if(count++ > 25){
	       joint_final_pos.data = joint_start_pos.data;
	       break;
	    }
	 }
      }
      // Set up velocity profile.
      vel_prof->solve(joint_states.q, joint_final_pos, duration);
      
      // Reset time before beginning new trajectory.
      current_trajec_time = 0.0;
   }
   // Get desired joint positions.
   vel_prof->getDesiredJointPos(desired_joint_states.q, current_trajec_time);
   
   // Get desired joint velocities.
   vel_prof->getDesiredJointVel(desired_joint_states.qdot, current_trajec_time);
   
   // Increase timer.
   current_trajec_time += getPeriod();

   // Create torque commands message.
   for(int i=0; i<joint_states.q.rows(); ++i){
      torque_commands.torque[i] = (desired_joint_states.q(i) - joint_states.q(i))*Kp[i]
      + (desired_joint_states.qdot(i) - joint_states.qdot(i))*Kd[i]
      + (desired_joint_states.q(i) - joint_states.q(i))*Ki[i];
   }
   // Write torque commands.
   outPort_TorqueCommands.write(torque_commands);
}
