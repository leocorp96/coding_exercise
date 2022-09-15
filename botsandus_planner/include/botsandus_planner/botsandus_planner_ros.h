#ifndef BOTSANDUSPLANNER_H
#define BOTSANDUSPLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <botsandus_planner/botsandus_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>

namespace bup_local_planner
{
  class BotsAndUsPlannerROS : public nav_core::BaseLocalPlanner
  {
  public:
    BotsAndUsPlannerROS();
    BotsAndUsPlannerROS(std::string name, tf2_ros::Buffer *tf,
                     costmap_2d::Costmap2DROS *costmap_ros);
    ~BotsAndUsPlannerROS();

    void initialize(std::string name, tf2_ros::Buffer *tf,
                    costmap_2d::Costmap2DROS *costmap_ros);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);
    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
    bool isGoalReached();

    /**
    * @brief  Receives odometry data
    * @param msg An Odometry message
    */
    void odomCB(const nav_msgs::Odometry::ConstPtr &msg);
    /**
    * @brief  Copies odometry data into a new variable
    * @param msg An Odometry message variable
    */
    void getOdom(nav_msgs::Odometry& base_odom);
    /**
    * @brief  Updates passed variable with current robot velocities
    * @param robot_vel A PoseStamped message variable
    */
    void getRobotVel(geometry_msgs::PoseStamped& robot_vel);

    /**
    * @brief  Check if base is moving
    * @return  True if velocity is greater than stopped velocity, false otherwise
    */
    bool isMoving();
    /**
    * @brief  Publish a plan for visualization purposes
    * @param  path The plan to publish
    * @param  pub The published to use
    * @param  r,g,b,a The color and alpha value to use when publishing
    */
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);
    /**
    * @brief  Publish a waypoints for visualization purposes
    * @param  path The plan to publish
    * @param  pub The published to use
    * @param  r,g,b,a The color and alpha value to use when publishing
    */
    void publishWayPoints(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);

  private:
    bool initialized_, has_next_point_, allow_plan_update_,
    is_initial_rotation_to_goal_completed_, is_final_rotation_to_goal_completed_,
    goal_reached_, prune_plan_, fetch_local_goal_, use_p2p_, use_static_;
    Eigen::Vector2d goal_vec_;
    double goal_th_;
    std::size_t idx_;
    RobotParams robot_params_;
    boost::shared_ptr<BotsAndUsPlanner> bup_;
    costmap_2d::Costmap2DROS *costmap_ros_;
    tf2_ros::Buffer *tf_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    boost::mutex odom_mutex_;
    nav_msgs::Odometry base_odom_;
    ros::Subscriber odom_sub_;
    ros::Publisher global_plan_pub_, local_plan_pub_, wp_plan_pub_, cur_goal_pub_;

  };
};

#endif // BOTSANDUSPLANNER_H
