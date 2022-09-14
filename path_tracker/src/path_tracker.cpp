#include <path_tracker/path_tracker.h>

namespace p_tracker
{
  PathTracker::PathTracker(const ros::NodeHandle &g_nh, const ros::NodeHandle &p_nh): g_nh_(g_nh), p_nh_(p_nh)
  {

  }

  void PathTracker::odomSubscriber(const nav_msgs::Odometry &msg)
  {

  }

  void PathTracker::pathSubscriber(const geometry_msgs::PoseStamped &msg)
  {

  }

  void run()
  {

  }
};
