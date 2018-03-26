#ifndef RTT_GAZEBO_INTERFACE_H_
#define RTT_GAZEBO_INTERFACE_H_

#include <rtt_tools/rtt_tools.h>

#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <kdl/chain.hpp>
#include <sensor_msgs/JointState.h>

#include <functional>
#include <thread>
#include <fstream>
#include <assert.h>

class GazeboInterface : public RTT::TaskContext{

public:

   GazeboInterface(const std::string &name);

   bool configureHook();
   bool startHook();
   void updateHook();
   void cleanupHook();

   void worldUpdateBegin();
   void worldUpdateEnd();

   bool addPlugin(const std::string &filename);
   bool resetWorld();
   bool spawnModel(const std::string &model_description);

   virtual ~GazeboInterface();

protected:

   RTT::InputPort<sensor_msgs::JointState> inPort_TorqueCommandsToGazebo;

   RTT::OutputPort<sensor_msgs::JointState> outPort_JointStatesGazebo;

   gazebo::physics::ModelPtr model;
   gazebo::physics::WorldPtr loaded_world;
   std::vector<std::string> gazebo_joint_names;
   std::vector<gazebo::physics::JointPtr> gazebo_joints;
   std::vector<std::string> argv;
   std::string first_joint, last_joint, root_link, tip_link;
   std::string robot_description, model_name;
   std::string world_file_string, gazebo_world, gazebo_model_description;
   int first_joint_index, last_joint_index;
   std::thread gz_thread;
   KDL::Chain kdl_chain;
   sensor_msgs::JointState joint_states, torque_commands;
};

ORO_CREATE_COMPONENT(GazeboInterface)

#endif
