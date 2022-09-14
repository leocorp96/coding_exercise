#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <Eigen/Core>
#include <thread>
#include <path_tracker/utils.h>

namespace p_tracker
{
  class PathTracker
  {
  public:
    PathTracker(const ros::NodeHandle &g_nh, const ros::NodeHandle &p_nh);
    ~PathTracker(){}
    /**
    * @brief  Gets the robots odometry
    * @param msg Robot's current odometry info
    */
    void odomSubCB(const nav_msgs::OdometryConstPtr &msg);
    /**
    * @brief  Gets the published robot plan
    * @param msg Current plan being followed by the robot
    */
    void pathSubCB(const nav_msgs::PathConstPtr &msg);
    /**
    * @brief  Gets the robots immediate goal (local goal)
    * @param msg Robot's current goal target
    */
    void curGoalSubCB(const geometry_msgs::Pose2DConstPtr &msg);
    /**
    * @brief  Checks if the robot reached its destination or stuck
    * @param e Timer event
    */
    void stuckCB(const ros::TimerEvent &e);
    /**
    * @brief Executes the path tracking operations
    */
    void run();

  private:
    /**
    * @brief  Computes the robots local start coordinate based on its current goal (local)
    * @param  start Robot start coordinates
    * @return True if start coordinates found
    */
    bool getStartCoordinates(geometry_msgs::Pose2D &start);
    /**
    * @brief  Formats the robot pose to pose2D
    * @param  robot_pose Robot pose
    * @return True if transformation succeeded
    */
    bool getGlobalPose(geometry_msgs::Pose2D &robot_pose);

    double xy_tolerance_, stuck_time_, stuck_dist_;
    ros::NodeHandle g_nh_, p_nh_;
    ros::Subscriber odom_sub_, path_sub_, cur_goal_sub_;
    ros::Publisher tracker_pub_, status_pub_;
    ros::Timer stuck_timer_;
    boost::mutex odom_mutex_, path_mutex_, cur_goal_mutex_;
    nav_msgs::Odometry odom_, prev_odom_;
    nav_msgs::Path plan_;
    geometry_msgs::Pose2D cur_goal_;
    geometry_msgs::TransformStamped odom_global_tf_;
    tf2_ros::Buffer tf_buffer_;
  };
};

#endif // PATH_TRACKER_H
