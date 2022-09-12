#include <botsandus_planner/botsandus_planner_ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(bup_local_planner::BotsAndUsPlannerROS, nav_core::BaseLocalPlanner)

namespace bup_local_planner
{
  BotsAndUsPlannerROS::BotsAndUsPlannerROS(): costmap_ros_(NULL), tf_(NULL), initialized_(false),
                                              is_initial_rotation_to_goal_completed_(false),
                                              is_final_rotation_to_goal_completed_(false),
                                              goal_reached_(false), has_next_point_(false)
  {}
  BotsAndUsPlannerROS::BotsAndUsPlannerROS(std::string name, tf2_ros::Buffer *tf,
                                     costmap_2d::Costmap2DROS *costmap_ros):
    costmap_ros_(NULL), tf_(NULL), initialized_(false), is_initial_rotation_to_goal_completed_(false),
    is_final_rotation_to_goal_completed_(false), goal_reached_(false), has_next_point_(false)
  {
    initialize(name, tf, costmap_ros);
  }

  BotsAndUsPlannerROS::~BotsAndUsPlannerROS()
  {
    if(bup_ != nullptr)
          bup_ = nullptr;
  }

  void BotsAndUsPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf,
                                    costmap_2d::Costmap2DROS *costmap_ros)
  {
    if(!initialized_)
    {
      tf_ = tf;
      costmap_ros_ = costmap_ros;

      //get config data from parameter server and setup publishers/subscribers
      ros::NodeHandle pn("~/" + name);
      global_plan_pub_ = pn.advertise<nav_msgs::Path>("global_plan", 1);
      local_plan_pub_ = pn.advertise<nav_msgs::Path>("local_plan", 1);

      double dvar = 0.0; bool bvar = false; int ivar = 0;
      pn.param("max_rot_vel", dvar, 0.8);
      robot_params_["max_rot_vel"] = dvar;
      pn.param("min_rot_vel", dvar, 0.35);
      robot_params_["min_rot_vel"] = dvar;

      pn.param("max_vel_x", dvar, 1.0);
      robot_params_["max_vel_x"] = dvar;
      pn.param("min_vel_x", dvar, 0.25);
      robot_params_["min_vel_x"] = dvar;

      pn.param("acc_lim_x", dvar, 2.5);
      robot_params_["acc_lim_x"] = dvar;
      robot_params_["acc_lim_y"] = 0.0;
      pn.param("acc_lim_theta", dvar, 3.2);
      robot_params_["acc_lim_theta"] = dvar;

      pn.param("xy_tolerance", dvar, 0.1);
      robot_params_["xy_tolerance"] = dvar;
      pn.param("yaw_tolerance", dvar, 0.05);
      robot_params_["yaw_tolerance"] = dvar;

      pn.param("stopped_rot_vel", dvar, 0.05);
      robot_params_["stopped_rot_vel"] = dvar;
      pn.param("stopped_trans_vel", dvar, 0.1);
      robot_params_["stopped_trans_vel"] = dvar;

      pn.param("vx_samples", ivar, 15);
      robot_params_["vx_samples"] = ivar;
      pn.param("vth_samples", ivar, 15);
      robot_params_["vth_samples"] = ivar;

      pn.param("inscribed_radius", dvar, 0.2);
      robot_params_["ins_radius"] = dvar;
      pn.param("circumscribed_radius", dvar, 0.2);
      robot_params_["circ_radius"] = dvar;

      pn.param("sim_time", dvar, 2.0);
      robot_params_["sim_time"] = dvar;
      pn.param("ang_sim_granularity", dvar, 0.08);
      robot_params_["ang_sim_granularity"] = dvar;
      pn.param("sim_granularity", dvar, 0.08);
      robot_params_["sim_granularity"] = dvar;
      pn.param("heading_scoring", bvar, false);
      robot_params_["heading_scoring"] = bvar;
      pn.param("heading_scoring_timestep", dvar, 2.0);
      robot_params_["heading_scoring_timestep"] = dvar;
      pn.param("simple_attractor", bvar, false);
      robot_params_["simp_attractor"] = bvar;

      pn.param("path_dist_bias", dvar, 5.0);
      robot_params_["path_dist_bias"] = dvar;
      pn.param("goal_dist_bias", dvar, 1.5);
      robot_params_["goal_dist_bias"] = dvar;
      pn.param("occdist_scale", dvar, 0.01);
      robot_params_["occdist_scale"] = dvar;
      pn.param("look_ahead_dist", dvar, 0.8);
      robot_params_["look_ahead"] = dvar;
      pn.param("oscillation_reset_dist", dvar, 0.2);
      robot_params_["oscillation_reset_dist"] = dvar;

      pn.param("prune_plan", bvar, true);
      robot_params_["prune_plan"] = bvar;

      ros::NodeHandle gn;
      odom_sub_ = gn.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&BotsAndUsPlannerROS::odomCB, this, _1));

      //create the actual planner to use
      bup_.reset(new BotsAndUsPlanner(name, costmap_ros_, robot_params_));
      initialized_ = true;
    }
    else
      ROS_WARN("Planner already running!");
  }

  void BotsAndUsPlannerROS::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    base_odom_.child_frame_id = msg->child_frame_id;
  }

  void BotsAndUsPlannerROS::getOdom(nav_msgs::Odometry& base_odom)
  {//! TODO update main planner odom here
    boost::mutex::scoped_lock lock(odom_mutex_);
    base_odom = base_odom_;
  }

  void BotsAndUsPlannerROS::getRobotVel(geometry_msgs::PoseStamped& robot_vel)
  {
    // Set current velocities from odometry
    geometry_msgs::Twist global_vel;
    {
      boost::mutex::scoped_lock lock(odom_mutex_);
      global_vel.linear.x = base_odom_.twist.twist.linear.x;
      global_vel.linear.y = base_odom_.twist.twist.linear.y;
      global_vel.angular.z = base_odom_.twist.twist.angular.z;

      robot_vel.header.frame_id = base_odom_.child_frame_id;
    }
    robot_vel.pose.position.x = global_vel.linear.x;
    robot_vel.pose.position.y = global_vel.linear.y;
    robot_vel.pose.position.z = 0;
    tf2::Quaternion q_tf;
    q_tf.setRPY(0, 0, global_vel.angular.z);
    q_tf.normalize();
    robot_vel.pose.orientation = tf2::toMsg(q_tf);
    robot_vel.header.stamp = ros::Time();
  }

  bool BotsAndUsPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
  {
    if(!initialized_)
    {
      ROS_ERROR("Planner has not been initialized!");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;

    //clear flags
    goal_reached_ = false;
    return true;
  }

  bool BotsAndUsPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner has not been initialized!");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> local_plan;
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if (!bup_->transformGlobalPlan(*tf_, global_plan_, transformed_plan))
    {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //now we'll prune the plan based on the position of the robot
    if(boost::get<bool>(bup_->fetchData("prune_plan")))
      bup_->prunePlan(transformed_plan, global_plan_);

    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = bup_->getBaseFrame();

    geometry_msgs::PoseStamped robot_vel;
    getRobotVel(robot_vel);

    //check if the transformed plan is not empty
    if(transformed_plan.empty())
      return false;

    //set the first point as goal and set "has_next" flag
    const geometry_msgs::PoseStamped& goal_point = transformed_plan.front();
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.pose.position.x;
    double goal_y = goal_point.pose.position.y;
    double goal_th = getYaw(goal_point.pose.orientation);
    //has_next_point_ = bup_->selectGoalPoint(goal_x, goal_y, goal_th);

    //!initial rotate to goal
    if(!is_initial_rotation_to_goal_completed_)
    {//perform initial base rotation to goal orientation
      geometry_msgs::PoseStamped robot_pose = bup_->getGlobalPose();
      double goal_dir = atan2((goal_y - robot_pose.pose.position.y),
                              (goal_x - robot_pose.pose.position.x));
      bup_->updatePlan(transformed_plan);
      bup_->rotateToGoal(robot_vel, goal_dir, cmd_vel, true);
      if(bup_->isGoalOrientationReached(goal_dir))
        is_initial_rotation_to_goal_completed_ = true;

      //publish an empty plan because we've reached our goal position
      publishPlan(transformed_plan, global_plan_pub_);
      publishPlan(local_plan, local_plan_pub_);
      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }

    if(bup_->isGoalPositionReached(Eigen::Vector2d(goal_x, goal_y)))
    {
      if(bup_->isGoalOrientationReached(goal_th))
      {
        /*if(has_next_point_)
        {
          is_initial_rotation_to_goal_completed_ = false;
          is_final_rotation_to_goal_completed_ = false;
        }
        else*/
        {
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
          is_initial_rotation_to_goal_completed_ = false;
          is_final_rotation_to_goal_completed_ = false;
          goal_reached_ = true;
        }
      }
      else
      {//! if robot is in goal xy position, rotate inplace to match goal orientation and stop
        // update the plan in the main planner
        bup_->updatePlan(transformed_plan);

        //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
        nav_msgs::Odometry base_odom;
        getOdom(base_odom);
        if(is_final_rotation_to_goal_completed_ && bup_->isMoving(base_odom))
        {
          if(!bup_->stopWithAccLimits(robot_vel, cmd_vel))
            return false;
        }
        else
        {// perform final base rotation to goal orientation
          is_final_rotation_to_goal_completed_ = !bup_->rotateToGoal(robot_vel, goal_th, cmd_vel);
          ROS_INFO("Performing final base-to-goal rotation");
        }
      }

      //publish an empty plan because we've reached our goal position
      publishPlan(transformed_plan, global_plan_pub_);
      publishPlan(local_plan, local_plan_pub_);
      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }
    //! if goal position not reached
    // update the plan in the main planner
    bup_->updatePlan(transformed_plan);
    //compute what trajectory to drive along
    Trajectory path = bup_->findBestPath(robot_vel, drive_cmds);
    //get drive commands
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = getYaw(drive_cmds.pose.orientation);

    //if we cannot move (no free path found)
    if (path.cost_ < 0)
    {
      local_plan.clear();
      publishPlan(transformed_plan, global_plan_pub_);
      publishPlan(local_plan, local_plan_pub_);
      return false;
    }

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i)
    {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = bup_->getGlobalFrame();
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = p_x;
      pose.pose.position.y = p_y;
      pose.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      pose.pose.orientation = tf2::toMsg(q);
      local_plan.push_back(pose);
    }

    publishPlan(transformed_plan, global_plan_pub_);
    publishPlan(local_plan, local_plan_pub_);
    return true;
  }

  void BotsAndUsPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub)
  {
    //given an empty path we won't do anything
    if(path.empty())
      return;

    //create a path message
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++)
      gui_path.poses[i] = path[i];
    pub.publish(gui_path);
  }

  bool BotsAndUsPlannerROS::isGoalReached()
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner has not been initialized!");
      return false;
    }
    return goal_reached_;
  }
};