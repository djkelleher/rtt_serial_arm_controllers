#include <rtt_tools/rtt_tools.h>

namespace rtt_tools{

   bool getKDLChain(KDL::Chain &kdl_chain, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("root_link")){
         RTT::log(RTT::Error) << "Could not find parameter \"root_link\". (string)" << RTT::endlog();
         return false;
      }
      if(!rosparam->getAbsolute("tip_link")){
         RTT::log(RTT::Error) << "Could not find parameter \"tip_link\". (string)" << RTT::endlog();
         return false;
      }
      if(!rosparam->getAbsolute("robot_description")){
         RTT::log(RTT::Error) << "Could not find parameter \"robot_description\". (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> root_link_property = this_taskcontext->getProperty("root_link");
      RTT::Property<string> tip_link_property = this_taskcontext->getProperty("tip_link");
      RTT::Property<string> robot_description_property = this_taskcontext->getProperty("robot_description");

      KDL::Tree kdl_tree;
      
      if(!kdl_parser::treeFromString(robot_description_property.get(), kdl_tree)){
         RTT::log(RTT::Error) << "Failed to construct a KDL tree." << RTT::endlog();
         return false;
      }
      const KDL::SegmentMap& segments(kdl_tree.getSegments());
      RTT::log(RTT::Info) << "Created a KDL tree with segments: " << RTT::endlog();
      for(auto &seg : segments) RTT::log(RTT::Info) << seg.first << RTT::endlog();

      if(segments.find(root_link_property.get()) == segments.end()){
         RTT::log(RTT::Error) << "root_link '" << root_link_property.get()
	 << "' was not found in the KDL tree." << RTT::endlog();
         return false;
      }
      if(segments.find(tip_link_property.get()) == segments.end()){
         RTT::log(RTT::Error) << "tip_link '" << tip_link_property.get()
	 << "' was not found in the KDL tree." << RTT::endlog();
         return false;
      }
      if(!kdl_tree.getChain(root_link_property.get(), tip_link_property.get(), kdl_chain)){
         RTT::log(RTT::Error) << "Could not build a KDL chain with root_link: " << root_link_property.get()
	 << " and tip_link: " << tip_link_property.get() << RTT::endlog();
         return false;
      }
      RTT::log(RTT::Info) << "Building a KDL chain from " << root_link_property.get()
      << " to " << tip_link_property.get() << RTT::endlog();
      
      RTT::log(RTT::Info) << "chain has " << kdl_chain.getNrOfJoints() <<" joints and "
      << kdl_chain.getNrOfSegments() <<" segments." << RTT::endlog();
      
      RTT::log(RTT::Info) << "Segments Names:" << RTT::endlog();
      for(int i=0; i<kdl_chain.getNrOfSegments(); ++i)
	 RTT::log(RTT::Info) << kdl_chain.getSegment(i).getName() << RTT::endlog();
      
      RTT::log(RTT::Info) << "Joint Names:" << RTT::endlog();
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
	 RTT::log(RTT::Info) << kdl_chain.getSegment(i).getJoint().getName() << RTT::endlog();
      
      return true;
   }
   
   bool getGazeboWorldFile(std::string &gazebo_world_path, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");
      
      if(!rosparam->getAbsolute("gazebo_world_path")){
	 RTT::log(RTT::Warning) << "Could not find parameter \"gazebo_world_path\". Using empty world." << RTT::endlog();
	 gazebo_world_path = "worlds/empty.world";
      }
      else{
	 RTT::Property<string> gazebo_world_path_property = this_taskcontext->getProperty("gazebo_world_path");
	 gazebo_world_path = gazebo_world_path_property.get();
	 RTT::log(RTT::Info) << "Found parameter \"gazebo_world_path\": " << gazebo_world_path << RTT::endlog();
      }
      std::ifstream worldfile(gazebo_world_path);
      
      if(worldfile){
	 RTT::log(RTT::Info) << "Found world file on gazebo_world_path." << RTT::endlog();
	 return true;
      }
      else RTT::log(RTT::Error) << "gazebo_world_path is not a valid file path!" << RTT::endlog();
      return false;
   }

   bool getRobotDescription(std::string &robot_description, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("robot_description")){
         RTT::log(RTT::Error) << "Could not find parameter \"robot_description\". (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> robot_description_property = this_taskcontext->getProperty("robot_description");
      RTT::log(RTT::Info) << "Found parameter \"robot_description\"." << RTT::endlog();
      robot_description = robot_description_property.get();
      return true;
   }

   bool getSegmentIndex(int &seg_index, std::string segment_name, const KDL::Chain &kdl_chain)
   {
      std::unordered_map<std::string, int> numbered_segs;
      
      for(int i = 0; i<kdl_chain.getNrOfSegments(); ++i)
         numbered_segs.insert(std::make_pair(kdl_chain.getSegment(i).getName(), i));
      
      auto search = numbered_segs.find(segment_name);
      if(search == numbered_segs.end()){
         RTT::log(RTT::Error) << "Segment '" << segment_name << "' was not found in KDL chain." << RTT::endlog();
         return false;
      }
      RTT::log(RTT::Info) << "Found chain segment '" << search->first << "' with index '"
      << search->second << "'" << RTT::endlog();
      
      seg_index = search->second;
      return true;
   }

   bool getJointNames(const KDL::Chain &kdl_chain, std::vector<std::string> &joint_names)
   {
      joint_names.clear();
      joint_names.reserve(kdl_chain.getNrOfJoints());
     
      for(int i = 0; i<kdl_chain.getNrOfJoints(); ++i)
         joint_names.push_back(kdl_chain.getSegment(i).getJoint().getName());
      
      RTT::log(RTT::Info) << "Created a vector of joint names: " << RTT::endlog();
      for(auto &joint : joint_names) RTT::log(RTT::Info) << joint << RTT::endlog();
      return true;
   }

   bool getRootLink(std::string &root_link, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("root_link")){
         RTT::log(RTT::Error) << "Could not find parameter \"root_link\". (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> root_link_property = this_taskcontext->getProperty("root_link");
      RTT::log(RTT::Info) << "Found parameter \"root_link\": " << root_link_property.get() << RTT::endlog();
      root_link = root_link_property.get();
      return true;
   }

   bool getTipLink(std::string &tip_link, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("tip_link")){
         RTT::log(RTT::Error) << "Could not find parameter \"tip_link\". (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> tip_link_property = this_taskcontext->getProperty("tip_link");
      RTT::log(RTT::Info) << "Found parameter \"tip_link\": " << tip_link_property.get() << RTT::endlog();
      tip_link = tip_link_property.get();
      return true;
   }

   bool getEndEffectorName(std::string &frame_name, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("end_effector_frame_name")){
         RTT::log(RTT::Error) << "Could not find parameter \"end_effector_frame_name\". (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> end_effector_frame_name_property = this_taskcontext->getProperty("end_effector_frame_name");
      
      RTT::log(RTT::Info) << "Found parameter \"end_effector_frame_name\": "
      << end_effector_frame_name_property.get() << RTT::endlog();
      
      frame_name = end_effector_frame_name_property.get();
      return true;
   }
   
   bool getFrameID(string &frame_id, RTT::TaskContext *this_taskcontext)
   {   
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getParam(this_taskcontext->getName() + "/frame_id", "frame_id")){
	 RTT::log(RTT::Error) << "Could not find parameter " << this_taskcontext->getName()
	 + "/frame_id. (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> frame_id_property = this_taskcontext->getProperty("frame_id");
      
      RTT::log(RTT::Info) << "Found parameter " << this_taskcontext->getName() + "/" + "frame_id: "
      << frame_id_property.get() << RTT::endlog();
      
      frame_id = frame_id_property.get();
      return true;
   }
   
   bool transformWaypointsToNewFrame(bool &transform, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");
      
      if(!rosparam->getParam(this_taskcontext->getName() + "/tranform_waypoints_to_new_frame", "tranform_waypoints_to_new_frame")){
	 RTT::log(RTT::Error) << "Could not find parameter " << this_taskcontext->getName() 
	 + "/tranform_waypoints_to_new_frame. (bool)" << RTT::endlog();
	 return false;
      }
      RTT::Property<bool> transform_points_property = this_taskcontext->getProperty("tranform_waypoints_to_new_frame");
      
      RTT::log(RTT::Info) << "Found parameter " << this_taskcontext->getName() + "/"
      + "tranform_waypoints_to_new_frame: " << transform_points_property.get() << RTT::endlog();
      
      transform = transform_points_property.get();
      return true;
   }
     
   bool getJointPosLims(std::vector<double> &lower_limits, std::vector<double> &upper_limits, const KDL::Chain &kdl_chain, RTT::TaskContext *this_taskcontext)
   {
      lower_limits.clear();
      upper_limits.clear();
      lower_limits.reserve(kdl_chain.getNrOfJoints());
      upper_limits.reserve(kdl_chain.getNrOfJoints());
      double noLimits(0.0);

      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("robot_description")){
         RTT::log(RTT::Error) << "Could not find parameter \"robot_description\". (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> robot_description_property = this_taskcontext->getProperty("robot_description");
      urdf::Model model;
      boost::shared_ptr<const urdf::Joint> joint;

      if(!model.initString(robot_description_property.get())){
         RTT::log(RTT::Error) << "Could not create model from \"robot_description\"." << RTT::endlog();
         return false;
      }
      for(int i=0; i<kdl_chain.getNrOfSegments(); ++i)
      {
         joint = model.getJoint(kdl_chain.getSegment(i).getJoint().getName());

         if(joint->safety && joint->limits) {
            lower_limits.push_back(std::max(joint->limits->lower, joint->safety->soft_lower_limit));
            upper_limits.push_back(std::min(joint->limits->upper, joint->safety->soft_upper_limit));
         }
         else if(!joint->safety && joint->limits)
         {
            lower_limits.push_back(joint->limits->lower);
            upper_limits.push_back(joint->limits->upper);
         }
         else if(joint->safety && !joint->limits)
         {
            lower_limits.push_back(joint->safety->soft_lower_limit);
            upper_limits.push_back(joint->safety->soft_upper_limit);
         }
         else
         {
            lower_limits.push_back(noLimits);
            upper_limits.push_back(noLimits);
         }
      }
      RTT::log(RTT::Info) << "Created vectors of joint pose limits:" << RTT::endlog();
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i){
         RTT::log(RTT::Info) << kdl_chain.getSegment(i).getJoint().getName() << ": lower limit "
	 << lower_limits[i] << ", upper limit " << upper_limits[i] << RTT::endlog();
      }
      return true;
   }

   bool getJointDynamics(std::vector<double> &friction, std::vector<double> &damping, const KDL::Chain &kdl_chain, RTT::TaskContext *this_taskcontext)
   {
      friction.clear();
      damping.clear();
      friction.reserve(kdl_chain.getNrOfJoints());
      damping.reserve(kdl_chain.getNrOfJoints());
      double noLimits(0.0);

      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("robot_description")){
         RTT::log(RTT::Error) << "Could not find parameter \"robot_description\". (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> robot_description_property = this_taskcontext->getProperty("robot_description");
      urdf::Model model;
      boost::shared_ptr<const urdf::Joint> joint;

      if(!model.initString(robot_description_property.get())){
         RTT::log(RTT::Error) << "Could not create a model from \"robot_description\"." << RTT::endlog();
         return false;
      }
      for(int i=0; i<kdl_chain.getNrOfSegments(); ++i)
      {
         joint = model.getJoint(kdl_chain.getSegment(i).getJoint().getName());

         if(joint->dynamics)
         {
            damping.push_back(joint->dynamics->damping);
            friction.push_back(joint->dynamics->friction);
         }
         else
         {
            damping.push_back(noLimits);
            friction.push_back(noLimits);
         }
      }
      RTT::log(RTT::Info) << "Created vector of joint friction values:" << RTT::endlog();
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
         RTT::log(RTT::Info) << kdl_chain.getSegment(i).getJoint().getName() << ": friction value " << friction[i] << RTT::endlog();
      
      RTT::log(RTT::Info) << "Created vectors of joint damping values:" << RTT::endlog();
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
         RTT::log(RTT::Info) << kdl_chain.getSegment(i).getJoint().getName() << ": damping value " << damping[i] << RTT::endlog();
      
      return true;
   }

   bool getJointStartPose(const KDL::Chain &kdl_chain, KDL::JntArray &joint_start_pose)
   {
      joint_start_pose.resize(kdl_chain.getNrOfJoints());
      std::vector<double> joint_start_pose_vec;
      
      /* Components store joint_start_pose property as a KDL JntArray which is not compatible with the parameter server.
       * We must store joint_start_pose as an XMLRPC array, use ROS to parse it into a vector, than convert to KDL JntArray. */
      if(!ros::param::get("joint_start_pose", joint_start_pose_vec)){
	 RTT::log(RTT::Error) << "Could not find parameter \"joint_start_pose\". (array)" << RTT::endlog();
	 return false;
      }
      if(joint_start_pose_vec.size() != kdl_chain.getNrOfJoints()){
         RTT::log(RTT::Error) << "Size of array \"joint_start_pose\" must be equal to the number of joints in the KDL chain!" << RTT::endlog();
         return false;
      }
      for(int i=0; i<joint_start_pose.rows(); ++i)
         joint_start_pose(i) = joint_start_pose_vec[i];
      
      RTT::log(RTT::Info) << "Created a KDL JntArray for joint start poses:" << RTT::endlog();
      for(int i=0; i<joint_start_pose.rows(); ++i)
         RTT::log(RTT::Info) << kdl_chain.getSegment(i).getJoint().getName() << ": start angle " << joint_start_pose(i) << RTT::endlog();
      return true;
   }

   bool getJointEffortLims(const KDL::Chain &kdl_chain, std::vector<double> &joint_effort_lims, RTT::TaskContext *this_taskcontext)
   {
      joint_effort_lims.clear();
      joint_effort_lims.reserve(kdl_chain.getNrOfJoints());
      double noLimits = 0.0;

      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("robot_description")){
         RTT::log(RTT::Error) << "Could not find parameter \"robot_description\". (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> robot_description_property = this_taskcontext->getProperty("robot_description");

      urdf::Model model;
      boost::shared_ptr<const urdf::Joint> joint;

      if(!model.initString(robot_description_property.get())){
         RTT::log(RTT::Error) << "Could not create a model from \"robot_description\"." << RTT::endlog();
         return false;
      }
      for(int i=0; i<kdl_chain.getNrOfSegments(); ++i)
      {
         joint = model.getJoint(kdl_chain.getSegment(i).getJoint().getName());

         if(joint->limits->effort)
	    joint_effort_lims.push_back(joint->limits->effort);
	 
         else joint_effort_lims.push_back(noLimits);
      }
      RTT::log(RTT::Error) << "Created vector of joint torque limits:" << RTT::endlog();
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
         RTT::log(RTT::Error) << kdl_chain.getSegment(i).getJoint().getName() << ": torque limit " << joint_effort_lims[i] << RTT::endlog();
      return true;
   }

   bool getJointVelLims(const KDL::Chain &kdl_chain, std::vector<double> &joint_vel_lims, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");
      
      joint_vel_lims.clear();
      joint_vel_lims.reserve(kdl_chain.getNrOfJoints());
      double noLimits(0.0);

      if(!rosparam->getAbsolute("robot_description")){
         RTT::log(RTT::Error) << "Could not find parameter \"robot_description\". (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> robot_description_property = this_taskcontext->getProperty("robot_description");
      urdf::Model model;
      boost::shared_ptr<const urdf::Joint> joint;

      if(!model.initString(robot_description_property.get())){
         RTT::log(RTT::Error) << "Could not create a model from \"robot_description\"." << RTT::endlog();
         return false;
      }
      for(int i=0; i<kdl_chain.getNrOfSegments(); ++i)
      {
         joint = model.getJoint(kdl_chain.getSegment(i).getJoint().getName());

         if(joint->limits->velocity)
	    joint_vel_lims.push_back(joint->limits->velocity);
         
         else joint_vel_lims.push_back(noLimits);
      }
      RTT::log(RTT::Info) << "Created vector of joint velocity limits:" << RTT::endlog();
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
         RTT::log(RTT::Info) << kdl_chain.getSegment(i).getJoint().getName() << ": velocity limit " << joint_vel_lims[i] << RTT::endlog();
      return true;
   }

   bool getJointAccLims(const KDL::Chain &kdl_chain, std::vector<double> &joint_accel_lims, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      joint_accel_lims.clear();
      joint_accel_lims.reserve(kdl_chain.getNrOfJoints());
      
      if(!rosparam->getAbsolute("joint_accel_lims")){
	 RTT::log(RTT::Error) << "Could not find parameter \"joint_accel_lims\". (array)" << RTT::endlog();
         return false;
      }
      RTT::Property<std::vector<double>> joint_accel_lims_property = this_taskcontext->getProperty("joint_accel_lims");
      if(joint_accel_lims_property.get().size() != kdl_chain.getNrOfJoints()){
         RTT::log(RTT::Error) << "Size of joint_accel_lims must equal the number of joints in the KDL chain." << RTT::endlog();
         return false;
      }
      joint_accel_lims = joint_accel_lims_property.get();
      RTT::log(RTT::Info) << "Created a vector of joint acceleration limits:" << RTT::endlog();
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
         RTT::log(RTT::Info) << kdl_chain.getSegment(i).getJoint().getName() << ": acceleration limit " << joint_accel_lims[i] << RTT::endlog();
      return true;
   }

   bool useJointVelLims(bool &use_vel_lims, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("use_joint_vel_lims")){
         RTT::log(RTT::Error) << "Could not find parameter \"use_joint_vel_lims\". (bool)" << RTT::endlog();
         return false;
      }
      RTT::Property<bool> use_joint_vel_lims_property = this_taskcontext->getProperty("use_joint_vel_lims");
      RTT::log(RTT::Info) << "Found param_error \"use_joint_vel_lims\": " << use_joint_vel_lims_property.get() << RTT::endlog();
      use_vel_lims = use_joint_vel_lims_property.get();
      return true;
   }

   bool useJointAccLims(bool &use_acc_lims, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("use_joint_acc_lims")){
         RTT::log(RTT::Error) << "Could not find parameter \"use_joint_acc_lims\". (bool)" << RTT::endlog();
         return false;
      }
      RTT::Property<bool> use_joint_acc_lims_property = this_taskcontext->getProperty("use_joint_acc_lims");
      RTT::log(RTT::Info) << "Found param_error \"use_joint_acc_lims\": " << use_joint_acc_lims_property.get() << RTT::endlog();
      use_acc_lims = use_joint_acc_lims_property.get();
      return true;
   }

   bool getTaskSpaceVelLim(double &vel_lim, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("task_space_vel_limit")){
         RTT::log(RTT::Error) << "Could not find parameter \"task_space_vel_limit\". (double)" << RTT::endlog();
         return false;
      }
      RTT::Property<double> task_space_vel_limit_propery = this_taskcontext->getProperty("task_space_vel_limit");
      RTT::log(RTT::Info) << "Found parameter \"task_space_vel_limit\": " << task_space_vel_limit_propery.get() << " m/s" << RTT::endlog();
      vel_lim = task_space_vel_limit_propery.get();
      return true;
   }

   bool getTaskSpaceAccLim(double &acc_lim, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("task_space_acc_limit")){
         RTT::log(RTT::Error) << "Could not find parameter \"task_space_acc_limit\". (double)" << RTT::endlog();
         return false;
      }
      RTT::Property<double> task_space_acc_limit_propery = this_taskcontext->getProperty("task_space_acc_limit");
      RTT::log(RTT::Info) << "Found parameter \"task_space_acc_limit\": " << task_space_acc_limit_propery.get() << " m/s^2" << RTT::endlog();
      acc_lim = task_space_acc_limit_propery.get();
      return true;
   }

   bool getTaskTrajectoryEquivalentRadius(double &eq_radius, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("task_trajectory_equivalent_radius")){
         RTT::log(RTT::Warning) << "Could not find parameter \"task_trajectory_equivalent_radius\". Using default value 0.05 m" << RTT::endlog();
         eq_radius = 0.05;
         return true;
      }
      RTT::Property<double> task_trajectory_equivalent_radius_property = this_taskcontext->getProperty("task_trajectory_equivalent_radius");
      
      RTT::log(RTT::Info) << "Found parameter \"task_trajectory_equivalent_radius\": "
      << task_trajectory_equivalent_radius_property.get() << " m" << RTT::endlog();
      
      eq_radius = task_trajectory_equivalent_radius_property.get();
      return true;
   }

   bool getTaskTrajectoryCornerRadius(double &radius, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("task_trajectory_corner_radius")){
         RTT::log(RTT::Warning) << "Could not find parameter \"task_trajectory_corner_radius\". Using default value 0.01 m" << RTT::endlog();
         radius = 0.01;
         return true;
      }
      RTT::Property<double> task_trajectory_corner_radius_property = this_taskcontext->getProperty("task_trajectory_corner_radius");
      RTT::log(RTT::Info) << "Found parameter \"task_trajectory_corner_radius\": " << task_trajectory_corner_radius_property.get() << " m" << RTT::endlog();
      radius = task_trajectory_corner_radius_property.get();
      return true;
   }

   bool getJointSpaceVelProfileName(std::string &vel_prof_name, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");

      if(!rosparam->getAbsolute("joint_space_velocity_profile")){
         RTT::log(RTT::Error) << "Could not find parameter \"joint_space_velocity_profile\". (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> joint_space_velocity_profile_property = this_taskcontext->getProperty("joint_space_velocity_profile");
      RTT::log(RTT::Info) << "Found parameter \"joint_space_velocity_profile\": " << joint_space_velocity_profile_property.get() << RTT::endlog();
      vel_prof_name = joint_space_velocity_profile_property.get();
      return true;
   }

   bool getTaskSpaceVelProfile(KDL::VelocityProfile *vel_prof, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");
      
      double max_vel, max_acc;

      if(!rosparam->getAbsolute("task_space_velocity_profile")){
         RTT::log(RTT::Error) << "Could not find parameter \"task_space_velocity_profile\". (string)" << RTT::endlog();
         return false;
      }
      RTT::Property<string> task_space_velocity_profile_property = this_taskcontext->getProperty("task_space_velocity_profile");

      RTT::log(RTT::Info) << "Found parameter \"task_space_velocity_profile\": " << task_space_velocity_profile_property.get() << RTT::endlog();

      if(task_space_velocity_profile_property.get() == "diracvel"){
         vel_prof = new KDL::VelocityProfile_Dirac();
         return true;
      }
      else if(task_space_velocity_profile_property.get() == "rectangular"){
	 getTaskSpaceVelLim(max_vel, this_taskcontext);
         vel_prof = new KDL::VelocityProfile_Rectangular(max_vel);
         return true;
      }
      else if(task_space_velocity_profile_property.get() == "trapezoidal"){
	 getTaskSpaceVelLim(max_vel, this_taskcontext);
	 getTaskSpaceAccLim(max_acc, this_taskcontext);
         vel_prof = new KDL::VelocityProfile_Trap(max_vel, max_acc);
         return true;
      }
      else if(task_space_velocity_profile_property.get() == "spline"){
	 getTaskSpaceVelLim(max_vel, this_taskcontext);
	 getTaskSpaceAccLim(max_acc, this_taskcontext);
         vel_prof = new KDL::VelocityProfile_Trap(max_vel, max_acc);
         return true;
      }
      else{ RTT::log(RTT::Info) << "function argument" << task_space_velocity_profile_property.get() << "does not match a velocity profile."
      "Options are \"diracvel\", \"rectangular\", \"trapezoidal\", \"spline\"" << RTT::endlog();
      return false;
      }
   }

   bool getIKSolver(std::unique_ptr<TRAC_IK::TRAC_IK> &ik_solver, RTT::TaskContext *this_taskcontext)
   {
      bool param_error(false);
      TRAC_IK::SolveType solver;
      RTT::Property<std::string> solver_type_property;
      RTT::Property<std::string> tip_link_property;
      RTT::Property<std::string> root_link_property;
      RTT::Property<double> max_solve_time_property("max_solve_time","",0.005);
      RTT::Property<double> error_property("error","",0.00001);
      
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");
      
      if(rosparam->getAbsolute("solver_type")){
	 
	 solver_type_property = this_taskcontext->getProperty("solver_type");
	 
	 if(solver_type_property.get() == "Speed")
	    solver = TRAC_IK::Speed;
	 
	 else if(solver_type_property.get() == "Manip")
	    solver = TRAC_IK::Manip1;
	 
	 else if(solver_type_property.get() == "Distance")
	    solver = TRAC_IK::Distance;
	 
	 else{
	    RTT::log(RTT::Warning) << "Could not resolve parameter \"solver_type\". Options are \"Speed\", \"Manip\", \"Distance\"." << RTT::endlog();
	    RTT::log(RTT::Warning) << "Using default solver mode \"Speed\"." << RTT::endlog();
	    solver = TRAC_IK::Speed;
	 }
      }
      else{
	 RTT::log(RTT::Warning) << "Could not find parameter \"solver_type\". Using default solver mode \"Speed\"." << RTT::endlog();
	 solver = TRAC_IK::Speed;
      }
      if(!rosparam->getAbsolute("max_solve_time"))
	 RTT::log(RTT::Warning) << "Could not find parameter \"max_solve_time\". Using default value 0.005 seconds." << RTT::endlog();
      
      else max_solve_time_property = this_taskcontext->getProperty("max_solve_time");
      
      if(!rosparam->getAbsolute("error"))
	 RTT::log(RTT::Warning) << "Could not find parameter \"error\". Using default value: 1e-5 m." << RTT::endlog();
      
      else error_property = this_taskcontext->getProperty("error");
	 
      if(!rosparam->getAbsolute("robot_description")){
	 RTT::log(RTT::Error) << "Could not find parameter \"robot_description\". (string)" << RTT::endlog();
	 param_error = true;
      }
      if(!rosparam->getAbsolute("root_link")){
	 RTT::log(RTT::Error) << "Could not find parameter \"root_link\". (string)" << RTT::endlog();
	 param_error = true;
      }
      if(!rosparam->getAbsolute("tip_link")){
	 RTT::log(RTT::Error) << "Could not find parameter \"tip_link\". (string)" << RTT::endlog();
	 param_error = true;
      }
      if(param_error) return false;
      
      root_link_property = this_taskcontext->getProperty("root_link");
      tip_link_property = this_taskcontext->getProperty("tip_link");
      
      ik_solver.reset(new TRAC_IK::TRAC_IK(root_link_property.get(), tip_link_property.get(), "robot_description",
					   max_solve_time_property.get(), error_property.get(), solver));
      
      RTT::log(RTT::Info) << "Created Trac-IK inverse kinematics solver with parameters -> root_link: "
      << root_link_property.get() << ", tip_link: " << tip_link_property.get()
      << ", robot_description, max_solve_time: " << max_solve_time_property.get()
      << ", error tolerance: "<< error_property.get() 
      << ", solver mode: " << solver << RTT::endlog();
      
      return true;
   }
   
   bool getJointPGains(const KDL::Chain &kdl_chain, std::vector<double> &joint_p_gains, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");
      
      joint_p_gains.clear();
      joint_p_gains.reserve(kdl_chain.getNrOfJoints());
      
      if(!rosparam->getAbsolute("joint_P_gains")){
	 RTT::log(RTT::Error) << "Could not find parameter \"joint_P_gains\". (array)" << RTT::endlog();
	 return false;
      }
      RTT::Property<std::vector<double>> joint_P_gains_property = this_taskcontext->getProperty("joint_P_gains");
      if(joint_P_gains_property.get().size() != kdl_chain.getNrOfJoints()){
	 RTT::log(RTT::Error) << "Size of joint_P_gains must be equal to the number of joints in the KDL chain!" << RTT::endlog();
	 return false;
      }
      joint_p_gains = joint_P_gains_property.get();
      RTT::log(RTT::Info) << "Created vector of joint space proportional gains:" << RTT::endlog();
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
	 RTT::log(RTT::Info) << kdl_chain.getSegment(i).getJoint().getName() << ": proportional gain " << joint_p_gains[i] << RTT::endlog();
      return true;
   }

   bool getJointIGains(const KDL::Chain &kdl_chain, std::vector<double> &joint_i_gains, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");
      
      joint_i_gains.clear();
      joint_i_gains.reserve(kdl_chain.getNrOfJoints());
      
      if(!rosparam->getAbsolute("joint_I_gains")){
	 RTT::log(RTT::Error) << "Could not find parameter \"joint_I_gains\". (array)" << RTT::endlog();
	 return false;
      }
      RTT::Property<std::vector<double>> joint_I_gains_property = this_taskcontext->getProperty("joint_I_gains");
      if(joint_I_gains_property.get().size() != kdl_chain.getNrOfJoints()){
	 RTT::log(RTT::Error) << "Size of joint_I_gains must be equal to the number of joints in the KDL chain!" << RTT::endlog();
	 return false;
      }
      joint_i_gains = joint_I_gains_property.get();
      RTT::log(RTT::Info) << "Created vector of joint space integral gains:" << RTT::endlog();
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
	 RTT::log(RTT::Info) << kdl_chain.getSegment(i).getJoint().getName() << ": integral gain " << joint_i_gains[i] << RTT::endlog();
      return true;
   }

   bool getJointDGains(const KDL::Chain &kdl_chain, std::vector<double> &joint_d_gains, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");
      
      joint_d_gains.clear();
      joint_d_gains.reserve(kdl_chain.getNrOfJoints());
      
      if(!rosparam->getAbsolute("joint_D_gains")){
	 RTT::log(RTT::Error) << "Could not find parameter \"joint_D_gains\". (array)" << RTT::endlog();
	 return false;
      }
      RTT::Property<std::vector<double>> joint_D_gains_property = this_taskcontext->getProperty("joint_D_gains");
      if(joint_D_gains_property.get().size() != kdl_chain.getNrOfJoints()){
	 RTT::log(RTT::Error) << "Size of joint_D_gains must be equal to the number of joints in the KDL chain!" << RTT::endlog();
	 return false;
      }
      joint_d_gains = joint_D_gains_property.get();
      RTT::log(RTT::Info) << "Created vector of joint space derivative gains:" << RTT::endlog();
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
	 RTT::log(RTT::Info) << kdl_chain.getSegment(i).getJoint().getName() << ": derivative gain " << joint_d_gains[i] << RTT::endlog();
      return true;
   }
   
   bool getTaskSpacePGains(std::vector<double> &task_p_gains, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");
      
      task_p_gains.clear();
      task_p_gains.resize(6);
      
      if(!rosparam->getAbsolute("task_P_gains")){
	 RTT::log(RTT::Error) << "Could not find parameter \"task_P_gains\". (array)" << RTT::endlog();
	 return false;
      }
      RTT::Property<std::vector<double>> task_P_gains_property = this_taskcontext->getProperty("task_P_gains");
      if(task_P_gains_property.get().size() != 6){
	 RTT::log(RTT::Error) << "task_P_gains array must have 6 elements. (KDL::Twist has 6 DOF)" << RTT::endlog();
	 return false;
      }
      task_p_gains = task_P_gains_property.get();
      RTT::log(RTT::Info) << "Created vector of task space proportional gains:" << RTT::endlog();
      RTT::log(RTT::Info) << "(Vx,Vy,Vz,Wx,Wy,Wz) -> (" << task_p_gains[0] << "," << task_p_gains[1]
      << "," << task_p_gains[2] << "," << task_p_gains[3]
      << "," << task_p_gains[4] << "," << task_p_gains[5] << ")" << RTT::endlog();
      return true;
   }

   bool getTaskSpaceIGains(std::vector<double> &task_i_gains, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");
      
      task_i_gains.clear();
      task_i_gains.resize(6);
      
      if(!rosparam->getAbsolute("task_I_gains")){
	 RTT::log(RTT::Error) << "Could not find parameter \"task_I_gains\". (array)" << RTT::endlog();
	 return false;
      }
      RTT::Property<std::vector<double>> task_I_gains_property = this_taskcontext->getProperty("task_I_gains");
      if(task_I_gains_property.get().size() != 6){
	 RTT::log(RTT::Error) << "task_I_gains array must have 6 elements. (KDL::Twist has 6 DOF)" << RTT::endlog();
	 return false;
      }
      task_i_gains = task_I_gains_property.get();
      RTT::log(RTT::Info) << "Created vector of task space integral gains:" << RTT::endlog();
      RTT::log(RTT::Info) << "(Vx,Vy,Vz,Wx,Wy,Wz) -> (" << task_i_gains[0] << "," << task_i_gains[1] << "," << task_i_gains[2]
      << "," << task_i_gains[3] << "," << task_i_gains[4] << "," << task_i_gains[5] << ")" << RTT::endlog();
      return true;
   }
   
   bool getTaskSpaceDGains(std::vector<double> &task_d_gains, RTT::TaskContext *this_taskcontext)
   {
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
	 this_taskcontext->getProvider<rtt_rosparam::ROSParam>("rosparam");
      
      task_d_gains.clear();
      task_d_gains.resize(6);
      
      if(!rosparam->getAbsolute("task_D_gains")){
	 RTT::log(RTT::Error) << "Could not find parameter \"task_D_gains\". (array)" << RTT::endlog();
	 return false;
      }
      RTT::Property<std::vector<double>> task_D_gains_property = this_taskcontext->getProperty("task_D_gains");
      if(task_D_gains_property.get().size() != 6){
	 RTT::log(RTT::Error) << "task_D_gains array must have 6 elements. (KDL::Twist has 6 DOF)" << RTT::endlog();
	 return false;
      }
      task_d_gains = task_D_gains_property.get();
      RTT::log(RTT::Info) << "Created vector of task space derivative gains:" << RTT::endlog();
      RTT::log(RTT::Info) << "(Vx,Vy,Vz,Wx,Wy,Wz) -> (" << task_d_gains[0] << "," << task_d_gains[1] << ","
      << task_d_gains[2] << "," << task_d_gains[3] << "," << task_d_gains[4] << "," << task_d_gains[5] << ")" << RTT::endlog();
      return true;
   }
}
