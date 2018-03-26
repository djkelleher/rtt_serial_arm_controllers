#ifndef JOINT_SPACE_VELOCITY_PROFILES_H_
#define JOINT_SPACE_VELOCITY_PROFILES_H_

#include <rtt_tools/rtt_tools.h>

#include <rtt/TaskContext.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>

#include <cmath>
#include <memory>
#include <array>

namespace joint_space_vel_prof{

   class velocityProfile{

   public:

      virtual void solve(const KDL::JntArray &qi, const KDL::JntArray &qf, double &duration) = 0;
      virtual void getDesiredJointPos(KDL::JntArray &desired_joint_pos, double &current_trajec_time) = 0;
      virtual void getDesiredJointVel(KDL::JntArray &desired_joint_vel, double &current_trajec_time) = 0;
      virtual void getDesiredJointAcc(KDL::JntArray &desired_joint_acc, double &current_trajec_time) = 0;

      virtual ~velocityProfile(){}

   };

   class cubicPolynomial : public velocityProfile
   {
   private:
      
      bool use_vel_limits, use_acc_limits;
      double duration;
      std::vector<double> tf, tfv, tfa;
      std::vector<double> max_vel, max_acc;
      std::vector<double> p0, p2, p3;
      std::vector<double> v1, v2;
      std::vector<double> a0, a1;
      KDL::JntArray qf_qi;

   public:

      cubicPolynomial(const KDL::Chain &kdl_chain, RTT::TaskContext *this_taskcontext);

      virtual void solve(const KDL::JntArray &qi, const KDL::JntArray &qf, double &duration);
      virtual void getDesiredJointPos(KDL::JntArray &desired_joint_pos, double &current_trajec_time);
      virtual void getDesiredJointVel(KDL::JntArray &desired_joint_vel, double &current_trajec_time);
      virtual void getDesiredJointAcc(KDL::JntArray &desired_joint_acc, double &current_trajec_time);

      virtual ~cubicPolynomial(){}
   };

   class quinticPolynomial : public velocityProfile
   {
   private:
      
      bool use_vel_limits, use_acc_limits;
      double duration;
      std::vector<double> tf, tfv, tfa;
      std::vector<double> max_vel, max_acc;
      std::vector<double> p0, p3, p4, p5;
      std::vector<double> v2, v3, v4;
      std::vector<double> a1, a2, a3;
      KDL::JntArray qf_qi;

   public:

      quinticPolynomial(const KDL::Chain &kdl_chain, RTT::TaskContext *this_taskcontext);

      virtual void solve(const KDL::JntArray &qi, const KDL::JntArray &qf, double &duration);
      virtual void getDesiredJointPos(KDL::JntArray &desired_joint_pos, double &current_trajec_time);
      virtual void getDesiredJointVel(KDL::JntArray &desired_joint_vel, double &current_trajec_time);
      virtual void getDesiredJointAcc(KDL::JntArray &desired_joint_acc, double &current_trajec_time);

      virtual ~quinticPolynomial(){}
   };

   class bangBang : public velocityProfile
   {
   private:
      
      bool use_vel_limits, use_acc_limits;
      double duration;
      std::vector<double> tf, t_half, tfv, tfa; 
      std::vector<double> max_vel, max_acc;
      KDL::JntArray qi_, qf_qi;

   public:

      bangBang(const KDL::Chain &kdl_chain, RTT::TaskContext *this_taskcontext);

      virtual void solve(const KDL::JntArray &qi, const KDL::JntArray &qf, double &duration);
      virtual void getDesiredJointPos(KDL::JntArray &desired_joint_pos, double &current_trajec_time);
      virtual void getDesiredJointVel(KDL::JntArray &desired_joint_vel, double &current_trajec_time);
      virtual void getDesiredJointAcc(KDL::JntArray &desired_joint_acc, double &current_trajec_time);

      virtual ~bangBang(){}
   };

   class trapezoidal : public velocityProfile
   {
   private:
      
      double duration;
      std::vector<double> max_vel, max_acc, switch_time, tf;
      KDL::JntArray qi_, qf_, qf_qi;

   public:

      trapezoidal(const KDL::Chain &kdl_chain, RTT::TaskContext *this_taskcontext);

      virtual void solve(const KDL::JntArray &qi, const KDL::JntArray &qf, double &duration);
      virtual void getDesiredJointPos(KDL::JntArray &desired_joint_pos, double &current_trajec_time);
      virtual void getDesiredJointVel(KDL::JntArray &desired_joint_vel, double &current_trajec_time);
      virtual void getDesiredJointAcc(KDL::JntArray &desired_joint_acc, double &current_trajec_time);

      virtual ~trapezoidal(){}
   };
   
   bool getJointSpaceVelProfile(const std::string vel_prof_name, const KDL::Chain &kdl_chain,
				std::unique_ptr<joint_space_vel_prof::velocityProfile>& vel_prof, RTT::TaskContext *this_taskcontext);
}
   
#endif
