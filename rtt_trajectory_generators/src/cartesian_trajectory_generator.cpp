#include "rtt_trajectory_generators/cartesian_trajectory_generator.h"

// Create a new instance of CartesianTrajectory. The PreOperational argument forces you to call configureHook().
CartesianTrajectory::CartesianTrajectory(const std::string &name) : RTT::TaskContext(name, PreOperational), tf2_listener(tf2Buffer)
{
   // Add RTT ports.
   addPort("trajectory_waypoint_vector", inPort_WaypointVector);
   addPort("cartesian_trajectory_commands", outPort_CartesianTrajectory);

   // Add RTT properties.
   addProperty("frame_id", target_frame_id).doc("target reference frame for trajectory waypoints");
   addProperty("task_space_vel_limit", task_space_vel_limit).doc("max allowable end effector velocity");
   addProperty("task_space_acc_limit", task_space_acc_limit).doc("max allowable end effector acceleration");
   addProperty("task_trajectory_corner_radius", task_trajectory_corner_radius).doc("radius for path roundness");
   addProperty("task_trajectory_equivalent_radius", task_trajectory_equivalent_radius).doc("equivalent radius for path roundness");
   addProperty("task_space_velocity_profile", task_space_velocity_profile).doc("name of the velocity profile");
}

// Configure this component.
bool CartesianTrajectory::configureHook()
{
   // Abort if ports are not connected.
   if(!inPort_WaypointVector.connected()){
      RTT::log(RTT::Error) << "inPort_WaypointVector is not connected!" << RTT::endlog();
      return false;
   }
   if(!outPort_CartesianTrajectory.connected()){
      RTT::log(RTT::Error) << "outPort_CartesianTrajectory is not connected." << RTT::endlog();
      return false;
   }
   // Parse parameters.
   rtt_tools::getTaskSpaceVelLim(task_space_vel_limit, this);
   rtt_tools::getTaskSpaceAccLim(task_space_acc_limit, this);
   rtt_tools::getTaskTrajectoryCornerRadius(task_trajectory_corner_radius, this);
   rtt_tools::getTaskTrajectoryEquivalentRadius(task_trajectory_equivalent_radius, this);
   rtt_tools::getTaskSpaceVelProfile(vel_profile, this);
   rtt_tools::getFrameID(target_frame_id, this);
   
   waypoint_array.poses.reserve(20);

   return true;
}

// Start this component.
bool CartesianTrajectory::startHook(){
   
   return computeTrajectory();
}

// Run update loop.
void CartesianTrajectory::updateHook(){
   
   if(current_trajec_time < comp_trajec->Duration()){
      
      // Get desired states for the current time.
      current_pos = comp_trajec->Pos(current_trajec_time);
      current_vel = comp_trajec->Vel(current_trajec_time);
      current_acc = comp_trajec->Acc(current_trajec_time);
      
      tf::poseKDLToMsg(current_pos, cart_trajectory_msg.position);
      tf::twistKDLToMsg(current_vel, cart_trajectory_msg.velocity);
      tf::twistKDLToMsg(current_acc, cart_trajectory_msg.acceleration);
      
      // Increase timer
      current_trajec_time += getPeriod();
      
      cart_trajectory_msg.is_last_trajectory_msg.data = false;
      cart_trajectory_msg.header.stamp = rtt_rosclock::rtt_now();
      
      
      // Write desired cartesian position.
      outPort_CartesianTrajectory.write(cart_trajectory_msg);
   }
   else{
      cart_trajectory_msg.is_last_trajectory_msg.data = true;
      cart_trajectory_msg.header.stamp = rtt_rosclock::rtt_now();
      
      outPort_CartesianTrajectory.write(cart_trajectory_msg);
      
      computeTrajectory();
   }
}

bool CartesianTrajectory::computeTrajectory(){
   
   if(inPort_WaypointVector.read(waypoint_array) == RTT::NewData){
      
      try{

	 current_frame_id = waypoint_array.header.frame_id;
	 
	 for(auto &pose : waypoint_array.poses){
	    
	    frame_transformation = tf2Buffer.lookupTransform(target_frame_id, current_frame_id, ros::Time(0), ros::Duration(1.0));
	    
	    waypoint_frame = tf2::transformToKDL(frame_transformation);
	    
	    frame_waypoint_vector.push_back(waypoint_frame);
	 }
      }catch(tf2::TransformException tf_exception){
	 std::cout << tf_exception.what() << std::endl;
	 return false;
      }
      try{
	 // Rotates a frame over the existing single rotation axis formed by start and end rotation.
	 // If more than one rotational axis exist, an arbitrary one will be choosen.
	 interpolator = new KDL::RotationalInterpolation_SingleAxis();
	 
	 // Trajectory_Composite implements a trajectory that is composed of underlying trajectoria. Call Add to add a trajectory.
	 comp_trajec = new KDL::Trajectory_Composite();
	 
	 // The specification of a path, composed of way-points with rounded corners.
	 path = new KDL::Path_RoundedComposite(task_trajectory_corner_radius, task_trajectory_equivalent_radius, interpolator);
	 
	 // Trapezoidal vel profile constructed from max_vel and max_acc.
	 vel_profile = new KDL::VelocityProfile_Trap(task_space_vel_limit, task_space_acc_limit);
	 
	 if(frame_waypoint_vector.size() > 1){
	    
	    // Add each frame to the path.
	    std::for_each(frame_waypoint_vector.begin(), frame_waypoint_vector.end(), [&](KDL::Frame waypoint) {path->Add(waypoint);});
	    
	    // Finish creating the path.
	    path->Finish();
	    
	    // Configure velocity profile based on trajectory start position and end position.
	    vel_profile->SetProfile(0, path->PathLength());
	    
	    // Trajectory_Segment combines a VelocityProfile and a Path into a trajectory.
	    traject = new KDL::Trajectory_Segment(path, vel_profile);
	    
	    // Add trajectory segment to the composite trajectory.
	    comp_trajec->Add(traject);
	 }
	 else comp_trajec->Add(new KDL::Trajectory_Segment(new KDL::Path_Point(waypoint_frame), vel_profile));
	 
	 // Wait 0.5s at the end of the trajectory.
	 comp_trajec->Add(new KDL::Trajectory_Stationary(0.5, waypoint_frame));
      }
      catch(KDL::Error &error){
	 std::cout << "Planning was attempted with waypoints: " << std::endl;
	 for(auto &point : frame_waypoint_vector){std::cout << point << std::endl;}
	 std::cout <<  error.Description() << std::endl;
	 std::cout <<  error.GetType() << std::endl;
	 return false;
      }
      // Set trajectory time to 0 before starting new trajectory.
      current_trajec_time = 0.0;
      
      return true;
   }
}
