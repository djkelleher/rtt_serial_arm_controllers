#include "rtt_gazebo_interface/rtt_gazebo_interface.h"

// Create a new instance of GazeboInterface. The PreOperational argument forces you to make a call to configureHook().
GazeboInterface::GazeboInterface(const std::string &name) : RTT::TaskContext(name, PreOperational)
{
   // Add RTT ports.
   addPort("torque_commands_to_gazebo", inPort_TorqueCommandsToGazebo);
   addPort("joint_states_from_gazebo", outPort_JointStatesGazebo);
   
   // Add RTT properties.
   addProperty("gazebo_world", gazebo_world).doc("Gazebo world file");
   addProperty("robot_description", robot_description).doc("URDF or SDF");
   addProperty("root_link", root_link).doc("first link in KDL chain");
   addProperty("tip_link", tip_link).doc("last link in KDL chain");
   
   // Add RTT operations.
   addOperation("add_plugin", &GazeboInterface::addPlugin, this, RTT::OwnThread).doc("path to a plugin file");
   addOperation("reset_world", &GazeboInterface::resetWorld, this, RTT::OwnThread).doc("resets entire world and time");
   addOperation("spawn_model", &GazeboInterface::spawnModel, this, RTT::OwnThread).doc("spawn model from URDF or SDF");
}

bool GazeboInterface::configureHook()
{
   // Abort if ports are connected.
   if(!inPort_TorqueCommandsToGazebo.connected()){
      RTT::log(RTT::Error) << "inPort_TorqueCommandsToGazebo is not connected!" << RTT::endlog();
      return false;
   }
   if(!outPort_JointStatesGazebo.connected()){
      RTT::log(RTT::Error) << "outPort_JointStatesGazebo is not connected!" << RTT::endlog();
      return false;
   }
   // Parse parameters.
   rtt_tools::getKDLChain(kdl_chain, this);
   rtt_tools::getGazeboWorldFile(gazebo_world, this);
   rtt_tools::getRobotDescription(robot_description, this);

   // Resize.
   joint_states.position.resize(kdl_chain.getNrOfJoints());
   joint_states.velocity.resize(kdl_chain.getNrOfJoints());
   torque_commands.effort.resize(kdl_chain.getNrOfJoints());
   
   // No command line arguments are needed.
   argv.clear();
   
   // Set up Gazebo server.
   if(!gazebo::setupServer(argv)){
      RTT::log(RTT::Error) << "Could not start Gazebo server!" << RTT::endlog();
      return false;
   }
   // Load world file.
   loaded_world = gazebo::loadWorld(gazebo_world);
   
   if(loaded_world.get() == nullptr) 
      RTT::log(RTT::Error) << "Failed to load Gazebo world!" << RTT::endlog();
   
   else RTT::log(RTT::Info) << "Loaded Gazebo world: " << loaded_world->Name() << RTT::endlog();
   
   // Spawn model from robot_description.
   spawnModel(robot_description);
   
   // Get the robot's joints from Gazebo.
   gazebo_joints = model->GetJoints();
   
   // Get the names of the joints from Gazebo.
   for(auto &joint : gazebo_joints)
      gazebo_joint_names.push_back(joint->GetName());
   
   // Get the name of first joint in KDL chain.
   first_joint = kdl_chain.getSegment(0).getJoint().getName();
   
   // Get the name of last joint in KDL chain.
   last_joint = kdl_chain.getSegment(kdl_chain.getNrOfSegments()-1).getJoint().getName();

   // Find the joint index in Gazebo that correstponds to the first joint in the KDL chain
   first_joint_index = std::distance(gazebo_joint_names.begin(),
				     std::find(gazebo_joint_names.begin(), gazebo_joint_names.end(), first_joint));
   
   RTT::log(RTT::Info) << "Index of chain's first joint in Gazebo: " << first_joint_index << RTT::endlog();
   
   // Find the joint index in Gazebo that correstponds to the last joint in the KDL chain.
   last_joint_index = std::distance(gazebo_joint_names.begin(),
				    std::find(gazebo_joint_names.begin(), gazebo_joint_names.end(), last_joint));
   
   RTT::log(RTT::Info) << "Index of chain's last joint in Gazebo: " << last_joint_index << RTT::endlog();
   
   // Make sure the indices found are feasable.
   assert(last_joint_index - first_joint_index == kdl_chain.getNrOfJoints());
   
   // Connect to Gazebo world update events.
   gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboInterface::worldUpdateBegin,this));

   gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&GazeboInterface::worldUpdateEnd,this));
   
   return true;
}

bool GazeboInterface::startHook()
{
   // Run world.
   gz_thread = std::thread(gazebo::runWorld, loaded_world, 0);
   
   if(!gz_thread.joinable()){
      RTT::log(RTT::Error) << "Could not start Gazebo world" << RTT::endlog();
      return false;
   }
   return true;
}

// All updates are done in Gazebo hooks. (do not use setActivity in ops script)
void GazeboInterface::updateHook()
{
}

void GazeboInterface::worldUpdateBegin()
{
   gazebo::sensors::run_once();
   
   // Write torque commands to Gazebo.
   if(inPort_TorqueCommandsToGazebo.read(torque_commands) == RTT::NewData)
      for(int i = first_joint_index; i<last_joint_index; ++i)
         gazebo_joints[i]->SetForce(0, torque_commands.effort[i]);
}

void GazeboInterface::worldUpdateEnd()
{
   // Read joint states from Gazebo.
   for(int i = first_joint_index; i<last_joint_index; ++i){
      for(int c = 0; c < kdl_chain.getNrOfJoints(); ++c){
         joint_states.position[c] =  gazebo_joints[i]->Position(0);
         joint_states.velocity[c] = gazebo_joints[i]->GetVelocity(0);
      }
   }
   outPort_JointStatesGazebo.write(joint_states);
}

bool GazeboInterface::addPlugin(const std::string& filename)
{
   // Check for file.
   if(!std::ifstream(filename)){
      RTT::log(RTT::Error) << "Could not find Gazebo plugin: " << filename << RTT::endlog();
      return false;
   }
   // Load plugin.
   gazebo::addPlugin(filename);
   return true;
}

bool GazeboInterface::spawnModel(const std::string& model_description)
{
   // Model count before new model is added.
   int modelCountBeforeSpawn = loaded_world->ModelCount();

   RTT::log(RTT::Info) << "Spawning model..." << RTT::endlog();
   
   // Attempt to load the model.
   loaded_world->InsertModelString(model_description);

   // Count load attempts.
   int load_attempt = 0;
   
   while(loaded_world->ModelCount() != modelCountBeforeSpawn+1){
      
      // Need to advange the physics engine in order for the model count to increase.
      gazebo::runWorld(loaded_world, 1);
      
      // Sleep for 0.5 sec.
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
      // Increase counter and check count.
      if(load_attempt++ > 10){
	 RTT::log(RTT::Error) << "Failed to spawn model!" << RTT::endlog();
	 return false;
      }
   }
   RTT::log(RTT::Info) << "Successfuly spawned model: " << model.get()->GetName() << RTT::endlog();
   return true;
}

// Reset world.
bool GazeboInterface::resetWorld()
{
   RTT::log(RTT::Info) << "Resetting Gazebo world." << RTT::endlog();
   
   auto pre_reset_time = loaded_world->RealTime();

   loaded_world->Reset();
   
   auto post_reset_time = loaded_world->RealTime();
   
   if(post_reset_time >= pre_reset_time){
      RTT::log(RTT::Info) << "Failed to reset Gazebo world!" << RTT::endlog();
      return false;
   }
   return true;
}

// Shutdown Gazebo.
void GazeboInterface::cleanupHook()
{
   RTT::log(RTT::Info) << "Stoping Simulation." << RTT::endlog();

   gazebo::event::Events::sigInt.Signal();

   loaded_world->Fini();
   
   RTT::log(RTT::Info) << "Shutting down Gazebo" << RTT::endlog();

   gazebo::shutdown();
}

GazeboInterface::~GazeboInterface()
{
   if(!gazebo::shutdown()){
      
      RTT::log(RTT::Info) << "Stoping Simulation." << RTT::endlog();
      
      gazebo::event::Events::sigInt.Signal();
      
      loaded_world->Fini();
      
      RTT::log(RTT::Info) << "Shutting down Gazebo" << RTT::endlog();
      
      gazebo::shutdown();
   }
}
   
