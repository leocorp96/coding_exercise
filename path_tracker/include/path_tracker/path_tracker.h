#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

namespace p_tracker
{
  class PathTracker
  {
  public:
    PathTracker(const ros::NodeHandle &g_nh, const ros::NodeHandle &p_nh);

    void odomSubscriber(const nav_msgs::Odometry &msg);
    void pathSubscriber(const geometry_msgs::PoseStamped &msg);

  private:
    void run();

    ros::NodeHandle g_nh_, p_nh_;
    ros::Subscriber odom_sub_, path_sub_;
    ros::Publisher path_tracker_pub_;
    boost::mutex odom_mutex_, path_mutex_;
  };
};

#endif // PATH_TRACKER_H
