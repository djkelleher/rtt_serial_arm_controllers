#ifndef RTT_TOOLS_H_
#define RTT_TOOLS_H_

#include <ros/ros.h>
#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt_rosparam/rosparam.h>

#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/velocityprofile.hpp>
#include <kdl/velocityprofile_dirac.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <trac_ik/trac_ik.hpp>
#include <urdf/model.h>

#include <unordered_map>
#include <memory>
#include <assert.h>

namespace rtt_tools {

   bool getKDLChain(KDL::Chain &kdl_chain, RTT::TaskContext *this_taskcontext);
   
   bool getJointNames(const KDL::Chain &kdl_chain, std::vector<std::string> &joint_names);
   
   bool getJointPosLims(std::vector<double> &lower_limits, std::vector<double> &upper_limits, const KDL::Chain &kdl_chain, RTT::TaskContext *this_taskcontext);
   
   bool getJointVelLims(const KDL::Chain &kdl_chain, std::vector<double> &joint_vel_lims, RTT::TaskContext *this_taskcontext);
   
   bool getJointAccLims(const KDL::Chain &kdl_chain, std::vector<double> &joint_acc_lims, RTT::TaskContext *this_taskcontext);
   
   bool getJointEffortLims(const KDL::Chain &kdl_chain, std::vector<double> &joint_effort_lims, RTT::TaskContext *this_taskcontext);
   
   bool getJointDynamics(std::vector<double> &friction, std::vector<double> &damping, const KDL::Chain &kdl_chain, RTT::TaskContext *this_taskcontext);
   
   bool getJointPGains(const KDL::Chain &kdl_chain, std::vector<double> &joint_p_gains, RTT::TaskContext *this_taskcontext);
   
   bool getJointIGains(const KDL::Chain &kdl_chain, std::vector<double> &joint_i_gains, RTT::TaskContext *this_taskcontext);
   
   bool getJointDGains(const KDL::Chain &kdl_chain, std::vector<double> &joint_d_gains, RTT::TaskContext *this_taskcontext);
   
   bool getJointSpaceVelProfileName(std::string &vel_prof_name, RTT::TaskContext *this_taskcontext);
   
   bool useJointVelLims(bool &use_vel_lims, RTT::TaskContext *this_taskcontext);
   
   bool useJointAccLims(bool &use_acc_lims, RTT::TaskContext *this_taskcontext);
   
   bool getJointStartPose(const KDL::Chain &kdl_chain, KDL::JntArray &joint_start_pose);
   
   bool getIKSolver(std::unique_ptr<TRAC_IK::TRAC_IK> &IK_solver, RTT::TaskContext *this_taskcontext);
   
   bool getTaskSpaceVelLim(double &vel_lim, RTT::TaskContext *this_taskcontext);
   
   bool getTaskSpaceAccLim(double &acc_lim, RTT::TaskContext *this_taskcontext);
   
   bool getTaskTrajectoryEquivalentRadius(double &eq_radius, RTT::TaskContext *this_taskcontext);
   
   bool getTaskTrajectoryCornerRadius(double &radius, RTT::TaskContext *this_taskcontext);
   
   bool getTaskSpaceVelProfile(KDL::VelocityProfile *vel_prof, RTT::TaskContext *this_taskcontext);
   
   bool getTaskSpacePGains(std::vector<double> &task_p_gains, RTT::TaskContext *this_taskcontext);
   
   bool getTaskSpaceIGains(std::vector<double> &task_i_gains, RTT::TaskContext *this_taskcontext);
   
   bool getTaskSpaceDGains(std::vector<double> &task_d_gains, RTT::TaskContext *this_taskcontext);

   bool getSegmentIndex(int &seg_index, std::string segment_name, const KDL::Chain &kdl_chain);

   bool getRootLink(std::string &root_link, RTT::TaskContext *this_taskcontext);

   bool getTipLink(std::string &tip_link, RTT::TaskContext *this_taskcontext);

   bool getEndEffectorName(std::string &frame_name, RTT::TaskContext *this_taskcontext);
   
   bool transformWaypointsToNewFrame(bool &transform, RTT::TaskContext *this_taskcontext);
   
   bool getFrameID(std::string &frame_id, RTT::TaskContext *this_taskcontext);
   
   bool getGazeboWorldFile(std::string &gz_world, RTT::TaskContext *this_taskcontext);
   
   bool getRobotDescription(std::string &robot_description, RTT::TaskContext *this_taskcontext);
}

#endif

