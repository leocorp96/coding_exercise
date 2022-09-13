#ifndef BOTSANDUS_PLANNER_H
#define BOTSANDUS_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <botsandus_planner/map_grid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/variant.hpp>
#include <botsandus_planner/bup_utils.h>
#include <botsandus_planner/trajectory.h>
#include <botsandus_planner/map_grid.h>
#include <botsandus_planner/footprint_helper.h>
#include <botsandus_planner/costmap_model.h>
#include <botsandus_planner/line_iterator.h>

using RobotPVariant = boost::variant<double, int, std::string, bool>;
using RobotParams = std::map<std::string, RobotPVariant>;

namespace bup_local_planner
{
  /*struct RobotPValue : boost::static_visitor<RobotPVariant>
  {
    double operator()(double var){ return var; }
    int operator()(int var){ return var; }
    std::string operator()(std::string var){ return var; }
    bool operator()(bool var){ return var; }
  };*/

  class BotsAndUsPlanner
  {
  public:
    /**
    * @brief  Constructor for the main planner
    * @param name Planner name
    * @param costmap Global Costmap
    * @param  robot_params Robot parameters:
    * max_vel_x Maximum velocity in X
    * min_vel_x Minimum velocity in X
    * max_vel_th Maximum velocity around Z-Axis
    * min_vel_th Minimum velocity around Z-Axis
    * rot_stopped_vel Stop rotational velocity
    * trans_stop_vel Stopped translational velocity
    * acc_lim_[x|y|theta] Acceleration limits
    * xy_tolerance Goal position tolerance
    * yaw_tolerance Goal orientation tolerance
    * sim_time Trajectory Generation Simulation time
    */
    BotsAndUsPlanner(std::string, costmap_2d::Costmap2DROS *, const RobotParams &);
    ~BotsAndUsPlanner() {}

    /**
    * @brief  Returns the costmap
    */
    inline costmap_2d::Costmap2D& getCostmap() const { return *costmap_;}
    /**
    * @brief  Returns the costmap frame (global frame)
    */
    inline std::string getGlobalFrame() const { return costmap_ros_->getGlobalFrameID(); }
    /**
    * @brief  Returns the base frame id of the robot
    */
    inline std::string getBaseFrame() const { return costmap_ros_->getBaseFrameID(); }
    /**
    * @brief  Updates the robot global pose
    */
    bool updateGlobalPose();
    /**
    * @brief  Returns the robot global pose
    */
    inline geometry_msgs::PoseStamped& getGlobalPose() { updateGlobalPose(); return global_pose_; }
    /**
    * @brief  Returns the data at "key" else returns 0.0
    * @param key Key for accessing the robot parameters dictionary
    */
    RobotPVariant fetchData(const std::string &);

    /**
     * @brief  Transforms the global plan of the robot from the planner frame to the frame of the costmap,
     * selects only the (first) part of the plan that is within the costmap area.
     * @param tf A reference to a transform listener
     * @param global_plan The plan to be transformed
     * @param robot_pose The pose of the robot in the global frame (same as costmap)
     * @param costmap A reference to the costmap being used so the window size for transforming can be computed
     * @param global_frame The frame to transform the plan to
     * @param transformed_plan Populated with the transformed plan
     */
    bool transformGlobalPlan(const tf2_ros::Buffer& tf,
        const std::vector<geometry_msgs::PoseStamped>& global_plan,
        std::vector<geometry_msgs::PoseStamped>& transformed_plan);
    /**
    * @brief  Trim off parts of the global plan that are far enough behind the robot
    * @param plan The plan to be pruned
    * @param global_plan The plan to be pruned in the frame of the planner
    */
    void prunePlan(std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan);

    /**
    * @brief  return true or false if goal position reached (uses xy-tolerance)
    * @param  goal_vec The goal x and y points
    * @return true or false
    */
    bool isGoalPositionReached(const Eigen::Vector2d &goal_vec);
    /**
    * @brief  return true or false if goal orientation reached (uses yaw tolerance)
    * @param  goal_th Goal orientation
    * @return true or false
    */
    bool isGoalOrientationReached(const double &goal_th);
    /**
    * @brief  return true or false if robot stopped moving
    * @param  base_odom The current odometry information for the robot
    * @return true or false
    */
    bool isMoving(const nav_msgs::Odometry& base_odom);
    /**
    * @brief Stop the robot taking into account acceleration limits
    * @param  robot_vel The velocity of the robot
    * @param  cmd_vel The velocity commands to be filled
    * @return  True if a valid trajectory was found, false otherwise
    */
    bool stopWithAccLimits(const geometry_msgs::PoseStamped& robot_vel, geometry_msgs::Twist& cmd_vel);
    /**
    * @brief  Update the plan that the controller is following
    * @param new_plan A new plan for the controller to follow
    */
    void updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan, const bool &set_goal=true);
    /**
    * @brief  Given the current position, orientation, and velocity of the robot, return a trajectory to follow
    * @param global_vel The current velocity of the robot in world space
    * @param drive_velocities Will be set to velocities to send to the robot base
    * @return The selected path or trajectory
    */
    Trajectory findBestPath(geometry_msgs::PoseStamped& global_vel,
                            geometry_msgs::PoseStamped& drive_velocities);
    /**
    * @brief  Rotates base to the goal orientation
    * @param  robot_vel The velocity of the robot
    * @param  goal_th The desired theta value for the goal
    * @param  cmd_vel The velocity commands to be filled
    * @param  is_init If use in initial rotation run, resets the path and goal map grids
    * @return  True if a valid trajectory was found, false otherwise
    */
    bool rotateToGoal(const geometry_msgs::PoseStamped& robot_vel,
                      double goal_th,
                      geometry_msgs::Twist& cmd_vel, const bool is_init=false);
    /**
    * @brief  Drives in a straight line to the goal
    * @param  robot_vel The velocity of the robot
    * @param  cmd_vel The velocity commands to be filled
    * @return  True if a valid trajectory was found, false otherwise
    */
    bool driveToGoal(const geometry_msgs::PoseStamped& robot_vel, geometry_msgs::Twist& cmd_vel);
    /**
    * @brief  Selects the next goal point to traverse
    * @param  plan The current transformed plan to follow
    * @param  goal_vec The goal x and y coordinates to go to
    * @param  goal_th The orientation of goal
    * @return  True if trajectory has next point else, False
    */
    bool selectGoalPoint(const std::vector<geometry_msgs::PoseStamped> &plan, Eigen::Vector2d &goal_vec, double &goal_th);

    /**
    * @brief  Reduces the global plan resolution
    * @param  plan The current plan to downsample
    */
    void downSamplePlan(std::vector<geometry_msgs::PoseStamped> &plan);


  private:
    /**
    * @brief  Returns the sign of the given variable
    * @param  Variable (type double) to check
    */
    inline double sign(double x) const { return x < 0.0 ? -1.0 : 1.0; }
    /**
    * @brief  Create the trajectories we wish to explore, score them, and return the best option
    * @param x The x position of the robot
    * @param y The y position of the robot
    * @param theta The orientation of the robot
    * @param vx The x velocity of the robot
    * @param vy The y velocity of the robot
    * @param vtheta The theta velocity of the robot
    * @return
    */
    Trajectory createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta);
    /**
    * @brief  Generate and score a single trajectory
    * @param x The x position of the robot
    * @param y The y position of the robot
    * @param theta The orientation of the robot
    * @param vx The x velocity of the robot
    * @param vy The y velocity of the robot
    * @param vtheta The theta velocity of the robot
    * @param vx_samp The x velocity used to seed the trajectory
    * @param vy_samp The y velocity used to seed the trajectory
    * @param vtheta_samp The theta velocity used to seed the trajectory
    * @param acc_x The x acceleration limit of the robot
    * @param acc_y The y acceleration limit of the robot
    * @param acc_theta The theta acceleration limit of the robot
    * @param impossible_cost The cost value of a cell in the local map grid that is considered impassable
    * @param traj Will be set to the generated trajectory with its associated score
    */
    void generateTrajectory(double x, double y, double theta,
                            double vx, double vy, double vtheta,
                            double vx_samp, double vy_samp, double vtheta_samp,
                            double acc_x, double acc_y, double acc_theta,
                            double impossible_cost, Trajectory& traj);

    /**
    * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
    * @param x_i The x position of the robot
    * @param y_i The y position of the robot
    * @param theta_i The orientation of the robot
    * @return
    */
    double footprintCost(double x_i, double y_i, double theta_i);
    double pointCost(int x, int y);
    /**
    * @brief Finds a clear line of sight from the robot's cell to a farthest point on the path
    * @param cell_x robot's x-cell
    * @param cell_y robot's y-cell
    * @param x The x position of the robot
    * @param y The y position of the robot
    * @param heading The orientation of the robot
    * @return
    */
    double headingDiff(uint cell_x, uint cell_y, double x, double y, double heading);

    /**
    * @brief  Compute x position based on velocity
    * @param  xi The current x position
    * @param  vx The current x velocity
    * @param  vy The current y velocity
    * @param  theta The current orientation
    * @param  dt The timestep to take
    * @return The new x position
    */
    inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt)
    { return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;}

    /**
    * @brief  Compute y position based on velocity
    * @param  yi The current y position
    * @param  vx The current x velocity
    * @param  vy The current y velocity
    * @param  theta The current orientation
    * @param  dt The timestep to take
    * @return The new y position
    */
    inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt)
    { return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;}

    /**
    * @brief  Compute orientation based on velocity
    * @param  thetai The current orientation
    * @param  vth The current theta velocity
    * @param  dt The timestep to take
    * @return The new orientation
    */
    inline double computeNewThetaPosition(double thetai, double vth, double dt)
    { return thetai + vth * dt; }

    /**
    * @brief  Compute velocity based on acceleration
    * @param vg The desired velocity, what we're accelerating up to
    * @param vi The current velocity
    * @param a_max An acceleration limit
    * @param  dt The timestep to take
    * @return The new velocity
    */
    inline double computeNewVelocity(double vg, double vi, double a_max, double dt)
    {
      if((vg - vi) >= 0)
        return std::min(vg, vi + a_max * dt);
      return std::max(vg, vi - a_max * dt);
    }
    /**
    * @brief  Generate and score a single trajectory
    * @param x The x position of the robot
    * @param y The y position of the robot
    * @param theta The orientation of the robot
    * @param vx The x velocity of the robot
    * @param vy The y velocity of the robot
    * @param vtheta The theta velocity of the robot
    * @param vx_samp The x velocity used to seed the trajectory
    * @param vy_samp The y velocity used to seed the trajectory
    * @param vtheta_samp The theta velocity used to seed the trajectory
    * @return True if the trajectory is legal, false otherwise
    */
    bool checkTrajectory(double x, double y, double theta,
                         double vx, double vy, double vtheta,
                         double vx_samp, double vy_samp, double vtheta_samp);
    /**
    * @brief  Generate and score a single trajectory
    * @param x The x position of the robot
    * @param y The y position of the robot
    * @param theta The orientation of the robot
    * @param vx The x velocity of the robot
    * @param vy The y velocity of the robot
    * @param vtheta The theta velocity of the robot
    * @param vx_samp The x velocity used to seed the trajectory
    * @param vy_samp The y velocity used to seed the trajectory
    * @param vtheta_samp The theta velocity used to seed the trajectory
    * @return The score (as double)
    */
    double scoreTrajectory(double x, double y, double theta,
                           double vx, double vy, double vtheta,
                           double vx_samp, double vy_samp, double vtheta_samp);


    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;
    geometry_msgs::PoseStamped global_pose_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<geometry_msgs::Point> robot_footprint_;

    double local_goal_x_, local_goal_y_, final_goal_x_, final_goal_y_;
    RobotParams robot_params_; /// @brief Robot parameters
    bool final_goal_position_valid_; ///< @brief True if final_goal_x_ and final_goal_y_ have valid data.  Only false if an empty path is sent.

    //! map grid
    MapGrid path_map_; ///< @brief The local map grid where we propagate path distance
    MapGrid goal_map_; ///< @brief The local map grid where we propagate goal distance
    FootprintHelper footprint_helper_;

    //! trajectory generation
    bool stuck_left, stuck_right; ///< @brief Booleans to keep the robot from oscillating during rotation
    bool rotating_left, rotating_right; ///< @brief Booleans to keep track of the direction of rotation for the robot
    double prev_x_, prev_y_; ///< @brief Used to calculate the distance the robot has traveled before reseting oscillation booleans
    Trajectory traj_one, traj_two; ///< @brief Used for scoring trajectories
    WorldModel *world_model_; ///< @brief The world model that the controller uses for collision detection
    boost::mutex configuration_mutex_;
    boost::shared_ptr<LineIterator> linecost_;
    std::size_t prev_idx_;
  };
};

#endif // BOTSANDUS_PLANNER_H
