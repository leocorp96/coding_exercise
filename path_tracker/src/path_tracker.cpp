#include <path_tracker/path_tracker.h>

namespace p_tracker
{
  PathTracker::PathTracker(const ros::NodeHandle &g_nh, const ros::NodeHandle &p_nh): g_nh_(g_nh), p_nh_(p_nh)
  {
    //fetch variables from parameter server
    p_nh_.param<std::string>("global_frame", global_frame_, "map");
    p_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    p_nh_.param<double>("xy_tolerance", xy_tolerance_, 0.1);
    p_nh_.param<double>("stuck_time", stuck_time_, 2.0);
    p_nh_.param<double>("stuck_dist", stuck_dist_, 0.1);

    //prepare subscribers
    odom_sub_ = g_nh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&PathTracker::odomSubCB, this, _1));
    path_sub_ = g_nh_.subscribe<nav_msgs::Path>("plan", 1, boost::bind(&PathTracker::pathSubCB, this, _1));
    cur_goal_sub_ = g_nh_.subscribe<geometry_msgs::Pose2D>("current_goal", 1,
                                                           boost::bind(&PathTracker::curGoalSubCB, this, _1));
    //prepare publisher
    tracker_pub_ = g_nh_.advertise<std_msgs::Float32>("path_accuracy", 1);
    status_pub_ = g_nh_.advertise<std_msgs::String>("robot_status", 1);

    //initialize variables
    tf2_ros::TransformListener tfListener(tf_buffer_);
    odom_global_tf_ = tf_buffer_.lookupTransform(global_frame_, ros::Time(),
                                         odom_frame_, ros::Time(),
                                         odom_frame_, ros::Duration(0.5));

    //prepare timer
    stuck_timer_ = g_nh_.createTimer(ros::Duration(stuck_time_),
                                     boost::bind(&PathTracker::stuckCB, this, _1), false, true);
  }

  void PathTracker::odomSubCB(const nav_msgs::OdometryConstPtr &msg)
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    odom_ = *msg;
  }

  void PathTracker::pathSubCB(const nav_msgs::PathConstPtr &msg)
  {
    boost::mutex::scoped_lock lock(path_mutex_);
    plan_ = *msg;
  }

  void PathTracker::curGoalSubCB(const geometry_msgs::Pose2DConstPtr &msg)
  {
    boost::mutex::scoped_lock lock(cur_goal_mutex_);
    cur_goal_ = *msg;
  }

  void PathTracker::stuckCB(const ros::TimerEvent &e)
  {//check if robot is stuck every 'stuck_time' seconds
    geometry_msgs::Pose2D robot_pose;
    if(getGlobalPose(robot_pose))
    {
      if(!prev_odom_.header.frame_id.empty())
      {
        std_msgs::String status_msg;
        status_msg.data = "";
        bool goal_reached;
        {// is robot at final goal point?
          boost::mutex::scoped_lock lock(path_mutex_);
          goal_reached = computeEuclidean(
                Eigen::Vector2d(robot_pose.x, robot_pose.y),
                Eigen::Vector2d(plan_.poses[plan_.poses.size() - 1].pose.position.x,
                                plan_.poses[plan_.poses.size() - 1].pose.position.y)) <= xy_tolerance_;
        }
        if(!goal_reached)
        {// check if we are moving (assuming we aren't already at final goal point)
          boost::mutex::scoped_lock lock(odom_mutex_);
          if(computeEuclidean(Eigen::Vector2d(odom_.pose.pose.position.x,
                                              odom_.pose.pose.position.y),
                              Eigen::Vector2d(prev_odom_.pose.pose.position.x,
                                              prev_odom_.pose.pose.position.y)) < stuck_dist_)
          {//robot is stuck
            status_msg.data = "STUCK";
          }
          else
            status_msg.data = "MOVING";
          //update prev_odom
          prev_odom_ = odom_;
        }
        //publish base status
        status_pub_.publish(status_msg);
      }
      else //populate prev_odom for the first time
        prev_odom_ = odom_;
    }
  }

  bool PathTracker::getStartCoordinates(geometry_msgs::Pose2D &start)
  {
    boost::mutex::scoped_lock lock1(path_mutex_);
    boost::mutex::scoped_lock lock2(cur_goal_mutex_);
    for(std::size_t i = 0; i < plan_.poses.size(); i++)
    {
      if(computeEuclidean(Eigen::Vector2d(plan_.poses[i].pose.position.x, plan_.poses[i].pose.position.y),
                          Eigen::Vector2d(cur_goal_.x, cur_goal_.y)) <= xy_tolerance_)
      {
        if(i == 0)//first point in path. Return false: we need robot to traverse more points
          return false;
        start.x = plan_.poses[i - 1].pose.position.x;
        start.y = plan_.poses[i - 1].pose.position.y;
        start.theta = getYaw(plan_.poses[i - 1].pose.orientation);
        return true;
      }
    }
    return false;
  }

  bool PathTracker::getGlobalPose(geometry_msgs::Pose2D &robot_pose)
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    geometry_msgs::PoseStamped pose, new_pose;
    pose.header = odom_.header;
    pose.pose = odom_.pose.pose;
    try
    {
      tf2::doTransform(pose, new_pose, odom_global_tf_);
      robot_pose.x = new_pose.pose.position.x;
      robot_pose.y = new_pose.pose.position.y;
      robot_pose.theta = getYaw(new_pose.pose.orientation);
    }
    catch(tf2::LookupException& ex)
    {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ConnectivityException& ex)
    {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ExtrapolationException& ex)
    {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      return false;
    }
    return true;
  }


  void PathTracker::run()
  {
    geometry_msgs::Pose2D start;
    geometry_msgs::Pose2D robot_pose;
    std_msgs::Float32 err_msg;

    while(!ros::isShuttingDown() && ros::ok())
    {
      /* if we got the start coordinates and robot pose (global coordinates),
         compute deviation from path
      */
      if(getStartCoordinates(start) && getGlobalPose(robot_pose))
      {
        boost::mutex::scoped_lock lock(cur_goal_mutex_);
        double err = atan2((cur_goal_.y - start.y), (cur_goal_.x - start.x)) -
                     atan2((robot_pose.y - start.y), (robot_pose.x - start.x));
        err_msg.data = float(err);
        tracker_pub_.publish(err_msg);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    odom_sub_.shutdown();
    path_sub_.shutdown();
    cur_goal_sub_.shutdown();
  }
};
