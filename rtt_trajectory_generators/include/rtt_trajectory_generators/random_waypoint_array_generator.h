#ifndef RANDOM_WAYPOINT_ARRAY_GENERATOR_H
#define RANDOM_WAYPOINT_ARRAY_GENERATOR_H

#include <rtt_tools/rtt_tools.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/Activity.hpp>
#include <rtt_rosclock/rtt_rosclock.h>

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <tf_conversions/tf_kdl.h>

#include <memory>
#include <random>
#include <chrono>

class RandomWaypointArrayGenerator : public RTT::TaskContext{

public:

    RandomWaypointArrayGenerator(const string &name);

    bool configureHook();
    bool startHook();
    void updateHook();
    
    virtual ~RandomWaypointArrayGenerator(){}

protected:
   
    RTT::OutputPort<geometry_msgs::PoseArray> outPort_WaypointVector;
   
    std::string robot_description, root_link, tip_link, frame_id;
    KDL::Frame goal_pose_frame;
    geometry_msgs::PoseArray waypoint_vector;
    KDL::Chain kdl_chain;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
    KDL::JntArray rand_joint_angles;
    std::vector<double> joint_lower_lims;
    std::vector<double> joint_upper_lims;
    int num_of_points;
    double lower_lim, upper_lim;
    std::random_device random;
    std::uniform_real_distribution<double> angles;
};

ORO_LIST_COMPONENT_TYPE(RandomWaypointArrayGenerator)

#endif
