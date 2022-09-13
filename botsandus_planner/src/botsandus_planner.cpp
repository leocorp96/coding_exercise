#include <botsandus_planner/botsandus_planner.h>

namespace bup_local_planner
{
  BotsAndUsPlanner::BotsAndUsPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros, const RobotParams &robot_params):
    costmap_ros_(NULL), costmap_(NULL), robot_params_(robot_params)
  {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    path_map_ = MapGrid(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    goal_map_ = MapGrid(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    robot_footprint_ = costmap_ros_->getRobotFootprint();
    world_model_ = new CostmapModel(*costmap_);
  }

  bool BotsAndUsPlanner::updateGlobalPose()
  {
    if (!costmap_ros_->getRobotPose(global_pose_))
      return false;
    return true;
  }

  void BotsAndUsPlanner::downSamplePlan(std::vector<geometry_msgs::PoseStamped> &plan)
  {
    if (plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      return;
    }
    std::vector<geometry_msgs::PoseStamped> res;
    double ds_dist = 0.25; //25cm
    ROS_WARN_STREAM_THROTTLE(1, "Pre-Sample Length: " << computeEuclidean(
                      Eigen::Vector2d(plan[0].pose.position.x, plan[0].pose.position.y),
                      Eigen::Vector2d(plan.back().pose.position.x, plan.back().pose.position.y))
        << " Size: " << plan.size());
    res.push_back(plan[0]); //add first point
    ROS_WARN("Downsample");
    //ROS_WARN_STREAM("First point: " << plan[0].pose.position.x << " | " << plan[0].pose.position.y);
    for(std::size_t t = 1; t < plan.size(); t++)
    {
      double prev_dist = computeEuclidean(
            Eigen::Vector2d(plan[t].pose.position.x, plan[t].pose.position.y),
            Eigen::Vector2d(res.back().pose.position.x, res.back().pose.position.y));
      //check if last point in plan
      if((t == (plan.size()-1)) && (prev_dist < ds_dist))
      {
        //if(res.size() > 1)
        res.pop_back();
        res.push_back(plan[t]);
        /*ROS_WARN_STREAM("Cur goal: " << plan[t].pose.position.x << " | " << plan[t].pose.position.y <<
                        " Robot @: " << global_pose_.pose.position.x << " | " << global_pose_.pose.position.y);*/
        break;
      }

      if(prev_dist >= ds_dist)
      {//add point if greater/equal to ds_dist
        res.push_back(plan[t]);
      }
    }
    plan = res;
    ROS_WARN_STREAM_THROTTLE(1, "Post-Sample Length: " << computeEuclidean(
                      Eigen::Vector2d(plan[0].pose.position.x, plan[0].pose.position.y),
                      Eigen::Vector2d(plan.back().pose.position.x, plan.back().pose.position.y))
        << " Size: " << plan.size());

  }

  bool BotsAndUsPlanner::transformGlobalPlan(const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan)
  {
    transformed_plan.clear();
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      return false;
    }
    if(!updateGlobalPose())
    {
      ROS_ERROR("Global pose for robot not found!");
      return false;
    }

    const geometry_msgs::PoseStamped& plan_pose = global_plan[0]; //get first point in global plan
    try
    {
      // get plan_to_global_tf from plan_frame to global_frame
      geometry_msgs::TransformStamped plan_to_global_tf = tf.lookupTransform(getGlobalFrame(), ros::Time(),
          plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));

      //let's get the pose of the robot in the frame of the plan
      geometry_msgs::PoseStamped robot_pose;
      tf.transform(global_pose_, robot_pose, plan_pose.header.frame_id);

      //we'll discard points on the plan that are outside the local costmap
      double dist_threshold = std::max(costmap_->getSizeInCellsX() * costmap_->getResolution() / 2.0,
                                       costmap_->getSizeInCellsY() * costmap_->getResolution() / 2.0);

      unsigned int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = 0;

      //we need to loop to a point on the plan that is within a certain distance of the robot
      while(i < global_plan.size()) {
        sq_dist = computeEuclidean(Eigen::Vector2d(robot_pose.pose.position.x, robot_pose.pose.position.y),
                                   Eigen::Vector2d(global_plan[i].pose.position.x, global_plan[i].pose.position.y));
        if (sq_dist <= sq_dist_threshold) {
          break;
        }
        ++i;
      }

      geometry_msgs::PoseStamped newer_pose;
      //now we'll transform until points are outside of our distance threshold
      while(i < global_plan.size() && sq_dist <= sq_dist_threshold) {
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        tf2::doTransform(pose, newer_pose, plan_to_global_tf);

        transformed_plan.push_back(newer_pose);
        sq_dist = computeEuclidean(Eigen::Vector2d(robot_pose.pose.position.x, robot_pose.pose.position.y),
                                   Eigen::Vector2d(global_plan[i].pose.position.x, global_plan[i].pose.position.y));
        ++i;
      }
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
      if (!global_plan.empty())
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", getGlobalFrame().c_str(), global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }
    return true;
  }

  void BotsAndUsPlanner::prunePlan(std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
    ROS_ASSERT(global_plan.size() >= plan.size());
    std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator global_it = global_plan.begin();
    while(it != plan.end()){
      const geometry_msgs::PoseStamped& w = *it;
      double distance_sq = 0.0;
      if(updateGlobalPose())
        distance_sq = computeEuclidean(Eigen::Vector2d(global_pose_.pose.position.x, global_pose_.pose.position.y),
                                       Eigen::Vector2d(w.pose.position.x, w.pose.position.y));
      if(distance_sq < 1){
        ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose_.pose.position.x, global_pose_.pose.position.y,
                  w.pose.position.x, w.pose.position.y);
        break;
      }
      it = plan.erase(it);
      global_it = global_plan.erase(global_it);
    }
  }

  RobotPVariant BotsAndUsPlanner::fetchData(const std::string &key)
  {
    auto it = robot_params_.find(key);
    if(it != robot_params_.end())
      return it->second;
    if(key == "sim_time")
      return 1.0;
    return 0.0;
  }

  bool BotsAndUsPlanner::isGoalPositionReached(const Eigen::Vector2d &goal_vec)
  {
    if(computeEuclidean(Eigen::Vector2d(global_pose_.pose.position.x, global_pose_.pose.position.y), goal_vec)
       <= boost::get<double>(fetchData("xy_tolerance")))
      return true;
    return false;
  }

  bool BotsAndUsPlanner::isGoalOrientationReached(const double &goal_th)
  {
    if(fabs(angles::shortest_angular_distance(getYaw(global_pose_.pose.orientation), goal_th))
       <= boost::get<double>(fetchData("yaw_tolerance")))
      return true;
    return false;
  }

  bool BotsAndUsPlanner::isMoving(const nav_msgs::Odometry &base_odom)
  {
    double rot_stop_vel = boost::get<double>(fetchData("stopped_rot_vel"));
    double trans_stop_vel = boost::get<double>(fetchData("stopped_trans_vel"));
    return fabs(base_odom.twist.twist.angular.z) <= rot_stop_vel
          && fabs(base_odom.twist.twist.linear.x) <= trans_stop_vel
          && fabs(base_odom.twist.twist.linear.y) <= trans_stop_vel;
  }

  bool BotsAndUsPlanner::stopWithAccLimits(const geometry_msgs::PoseStamped& robot_vel, geometry_msgs::Twist& cmd_vel)
  {
    updateGlobalPose();
    double acc_x = boost::get<double>(fetchData("acc_lim_x")), acc_th = boost::get<double>(fetchData("acc_lim_theta"));
    double sim_time = boost::get<double>(fetchData("sim_time"));
    double vx = sign(robot_vel.pose.position.x) * std::max(0.0, (fabs(robot_vel.pose.position.x) - acc_x * sim_time));
    double vy = 0.0;
    double vel_yaw = getYaw(robot_vel.pose.orientation);
    double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_th * sim_time));

    //we do want to check whether or not the command is valid
    double yaw = getYaw(global_pose_.pose.orientation);
    bool valid_cmd = checkTrajectory(global_pose_.pose.position.x, global_pose_.pose.position.y, yaw,
                                     robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw,
                                     vx, vy, vth);
    if(valid_cmd)
    {
      ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.angular.z = vth;
      return true;
    }
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  void BotsAndUsPlanner::updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan, const bool &set_goal)
  {
    global_plan_.resize(new_plan.size());
    for(unsigned int i = 0; i < new_plan.size(); ++i)
      global_plan_[i] = new_plan[i];

    if(global_plan_.size() > 0)
    {
      if(set_goal)
      {
        geometry_msgs::PoseStamped& final_goal_pose = global_plan_[ global_plan_.size() - 1 ];
        final_goal_x_ = final_goal_pose.pose.position.x;
        final_goal_y_ = final_goal_pose.pose.position.y;
      }
      final_goal_position_valid_ = true;
    }
    else
      final_goal_position_valid_ = false;
  }

  Trajectory BotsAndUsPlanner::createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta)
  {
    //compute feasible velocity limits in robot space
    double max_vel_x = boost::get<double>(fetchData("max_vel_x"));
    double min_vel_x, min_vel_theta, max_vel_theta;
    double acc_x = boost::get<double>(fetchData("acc_lim_x")), acc_th = boost::get<double>(fetchData("acc_lim_theta"));
    double sim_time = boost::get<double>(fetchData("sim_time"));

    if( final_goal_position_valid_ )
    {
      double final_goal_dist = hypot( final_goal_x_ - x, final_goal_y_ - y );
      max_vel_x = std::min( max_vel_x, final_goal_dist / sim_time);
    }

    //using v = u + at;
    max_vel_x = std::max(std::min(max_vel_x, vx + acc_x * sim_time), boost::get<double>(fetchData("min_vel_x")));
    min_vel_x = std::max(boost::get<double>(fetchData("min_vel_x")), vx - acc_x * sim_time);

    max_vel_theta = std::min(boost::get<double>(fetchData("max_rot_vel")), vtheta + acc_th * sim_time);
    min_vel_theta = std::max(boost::get<double>(fetchData("min_rot_vel")), vtheta - acc_th * sim_time);

    //we want to sample the velocity space regularly
    int vx_samples = boost::get<int>(fetchData("vx_samples"));
    int vth_samples = boost::get<int>(fetchData("vth_samples"));
    double dvx = (max_vel_x - min_vel_x) / (vx_samples - 1);
    double dvtheta = (max_vel_theta - min_vel_theta) / (vth_samples - 1);

    double vx_samp = min_vel_x;
    double vtheta_samp = min_vel_theta;
    double vy_samp = 0.0;

    //keep track of the best trajectory seen so far
    Trajectory* best_traj = &traj_one;
    best_traj->cost_ = -1.0;
    Trajectory* comp_traj = &traj_two;
    comp_traj->cost_ = -1.0;
    Trajectory* swap = NULL;

    //any cell with a cost greater than the size of the map is impossible
    double impossible_cost = path_map_.obstacleCosts();

    //for each x velocity sample, sample all theta velocities
    for(int i = 0; i < vx_samples; ++i)
    {// loop through all x velocities
      vtheta_samp = 0;
      //first sample the straight trajectory
      generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                         acc_x, 0.0, acc_th, impossible_cost, *comp_traj);

      //if the new trajectory is better... let's take it
      if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
        swap = best_traj;
        best_traj = comp_traj;
        comp_traj = swap;
      }

      vtheta_samp = min_vel_theta;
      for(int j = 0; j < vth_samples - 1; ++j)
      {//next sample all theta trajectories
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                           acc_x, 0.0, acc_th, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
        vtheta_samp += dvtheta;
      }
      vx_samp += dvx;
    }

    //Next we want to generate trajectories for rotating in place
    vtheta_samp = min_vel_theta;
    vx_samp = 0.0;
    vy_samp = 0.0;
    //let's try to rotate toward open space
    double heading_dist = DBL_MAX;
    for(int i = 0; i < vth_samples; ++i)
    {
      //enforce a minimum rotational velocity because the base can't handle small in-place rotations
      double vtheta_samp_limited = vtheta_samp > 0 ? std::max(vtheta_samp, min_vel_theta)
                                                   : std::min(vtheta_samp, -1.0 * min_vel_theta);

      generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp_limited,
                         acc_x, 0.0, acc_th, impossible_cost, *comp_traj);

      //if the new trajectory is better... let's take it...
      //note if we can legally rotate in place we prefer to do that rather than move with y velocity
      if(comp_traj->cost_ >= 0
         && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0 || best_traj->yv_ != 0.0)
         && (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta))
      {
        double x_r, y_r, th_r;
        double heading_lookahead = boost::get<double>(fetchData("look_ahead"));
        comp_traj->getEndpoint(x_r, y_r, th_r);
        x_r += heading_lookahead * cos(th_r);
        y_r += heading_lookahead * sin(th_r);
        unsigned int cell_x, cell_y;

        //make sure that we'll be looking at a legal cell
        if (costmap_->worldToMap(x_r, y_r, cell_x, cell_y))
        {
          double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
          if (ahead_gdist < heading_dist)
          {//if we haven't already tried rotating left since we've moved forward
            if (vtheta_samp < 0 && !stuck_left)
            {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
            else if(vtheta_samp > 0 && !stuck_right)
            {//if we haven't already tried rotating right since we've moved forward
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
          }
        }
      }
      vtheta_samp += dvtheta;
    }

    //do we have a legal trajectory
    if(best_traj->cost_ >= 0)
    {
      //avoid oscillations of in place rotation
      if(!(best_traj->xv_ > 0))
      {
        if (best_traj->thetav_ < 0)
        {
          if (rotating_right)
            stuck_right = true;
          rotating_right = true;
        }
        else if (best_traj->thetav_ > 0)
        {
          if (rotating_left)
            stuck_left = true;
          rotating_left = true;
        }

        //set the position we must move a certain distance away from
        prev_x_ = x;
        prev_y_ = y;
      }

      double dist = computeEuclidean(Eigen::Vector2d(x, y), Eigen::Vector2d(prev_x_, prev_y_));
      if(dist > boost::get<double>(fetchData("oscillation_reset_dist")))
      {
        rotating_left = false;
        rotating_right = false;
        stuck_left = false;
        stuck_right = false;
      }
      return *best_traj;
    }

    //and finally, if we can't do anything else, we want to generate trajectories that move backwards slowly
    vtheta_samp = 0.0;
    vx_samp = -min_vel_theta; //using negative of min vel as backup_vel
    vy_samp = 0.0;
    generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                       acc_x, 0.0, acc_th, impossible_cost, *comp_traj);
    //we'll allow moving backwards slowly even when the static map shows it as blocked
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;

    //if the trajectory failed because the footprint hits something, we're still going to back up
    if(best_traj->cost_ == -1.0)
      best_traj->cost_ = 1.0;
    return *best_traj;
  }

  double BotsAndUsPlanner::footprintCost(double x_i, double y_i, double theta_i)
  {
    return world_model_->footprintCost(x_i, y_i, theta_i, robot_footprint_,
                                       boost::get<double>(fetchData("ins_radius")),
                                       boost::get<double>(fetchData("circ_radius")));
  }

  double BotsAndUsPlanner::pointCost(int x, int y)
  {
    unsigned char cost = costmap_->getCost(uint(x), uint(y));
    //if the cell is in an obstacle the path is invalid
    if(cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION)
      return -1;
    return cost;
  }

  double BotsAndUsPlanner::headingDiff(uint cell_x, uint cell_y, double x, double y, double heading)
  {
    unsigned int goal_cell_x, goal_cell_y;
    // find a clear line of sight from the robot's cell to a farthest point on the path
    for (int i = global_plan_.size() - 1; i >=0; --i)
    {
      if (costmap_->worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y))
      {
        linecost_.reset(new LineIterator(cell_x, goal_cell_x, cell_y, goal_cell_y));
        if (linecost_->scoreLine(boost::bind(&BotsAndUsPlanner::pointCost, this, _1, _2)) >= 0)
        {
          double gx, gy;
          costmap_->mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
          return fabs(angles::shortest_angular_distance(heading, atan2(gy - y, gx - x)));
        }
      }
    }
    return DBL_MAX;
  }

  void BotsAndUsPlanner::generateTrajectory(
      double x, double y, double theta,
      double vx, double vy, double vtheta,
      double vx_samp, double vy_samp, double vtheta_samp,
      double acc_x, double acc_y, double acc_theta,
      double impossible_cost, Trajectory& traj)
  {
    // make sure the configuration doesn't change mid run
    boost::mutex::scoped_lock l(configuration_mutex_);

    double x_i = x, y_i = y, theta_i = theta;
    double vx_i = vx, vy_i = vy, vtheta_i = vtheta;

    //compute the magnitude of the velocities
    double vmag = hypot(vx_samp, vy_samp);

    //compute the number of steps we must take along this trajectory to be "safe"
    bool heading_scoring = boost::get<bool>(fetchData("heading_scoring"));
    double heading_scoring_timestep = boost::get<double>(fetchData("heading_scoring_timestep"));
    double sim_time = boost::get<double>(fetchData("sim_time"));
    double ang_sim_granularity = boost::get<double>(fetchData("ang_sim_granularity"));
    double sim_granularity = boost::get<double>(fetchData("sim_granularity"));

    int num_steps;
    if(!heading_scoring)
      num_steps = int(std::max((vmag * sim_time) / sim_granularity, fabs(vtheta_samp) / ang_sim_granularity) + 0.5);
    else
      num_steps = int(sim_time / sim_granularity + 0.5);

    //we at least want to take one step... even if we won't move, we want to score our current position
    if(num_steps == 0)
      num_steps = 1;

    double dt = sim_time / num_steps;
    double time = 0.0;
    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
    double heading_diff = 0.0;

    for(int i = 0; i < num_steps; ++i)
    {
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the know map
      if(!costmap_->worldToMap(x_i, y_i, cell_x, cell_y))
      {
        traj.cost_ = -1.0;
        return;
      }

      //check the point on the trajectory for legality
      double footprint_cost = footprintCost(x_i, y_i, theta_i);
      //if the footprint hits an obstacle this trajectory is invalid
      if(footprint_cost < 0)
      {
        traj.cost_ = -1.0;
        return;
      }
      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_->getCost(cell_x, cell_y)));
      //using a simple attractor to goal point
      if(boost::get<bool>(fetchData("simp_attractor")))
      {
        goal_dist = computeEuclidean(Eigen::Vector2d(x_i, y_i),
                                     Eigen::Vector2d(final_goal_x_, final_goal_x_));
      }
      else
      {
        bool update_path_and_goal_distances = true;
        // with heading scoring, we take into account heading diff, and also only score
        // path and goal distance for one point of the trajectory
        if(heading_scoring)
        {
          if (time >= heading_scoring_timestep && time < heading_scoring_timestep + dt)
            heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
          else
            update_path_and_goal_distances = false;
        }

        if(update_path_and_goal_distances)
        {
          //update path and goal distances
          path_dist = path_map_(cell_x, cell_y).target_dist;
          goal_dist = goal_map_(cell_x, cell_y).target_dist;
          //if a point on this trajectory has no clear path to goal it is invalid
          if(impossible_cost <= goal_dist || impossible_cost <= path_dist)
          {
            traj.cost_ = -2.0;
            return;
          }
        }
      }

      //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);
      //calculate velocities
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //calculate positions
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

      //increment time
      time += dt;
    }// end for i < numsteps

    double cost = -1.0;
    double path_distance_bias = boost::get<double>(fetchData("path_dist_bias"));
    double goal_distance_bias = boost::get<double>(fetchData("goal_dist_bias"));
    double occdist_scale = boost::get<double>(fetchData("occdist_scale"));
    if (!heading_scoring)
      cost = path_distance_bias * path_dist + goal_dist * goal_distance_bias + occdist_scale * occ_cost;
    else
      cost = occdist_scale * occ_cost + path_distance_bias * path_dist + 0.3 * heading_diff + goal_dist * goal_distance_bias;
    traj.cost_ = cost;
  }

  Trajectory BotsAndUsPlanner::findBestPath(geometry_msgs::PoseStamped& global_vel,
                                            geometry_msgs::PoseStamped& drive_velocities)
  {
    if(!updateGlobalPose())
    {
      ROS_ERROR("Global pose for robot not found!");
      return Trajectory(); // return empty trajectory
    }
    //populate the pos and vel variables
    Eigen::Vector3d pos(global_pose_.pose.position.x, global_pose_.pose.position.y, getYaw(global_pose_.pose.orientation));
    Eigen::Vector3d vel(global_vel.pose.position.x, global_vel.pose.position.y, getYaw(global_vel.pose.orientation));
    //reset the map for new operations
    path_map_.resetPathDist();
    goal_map_.resetPathDist();

    //temporarily remove obstacles that are within the footprint of the robot
    std::vector<botsandus_planner::Position2DInt> footprint_list = footprint_helper_.getFootprintCells(
          pos, robot_footprint_, *costmap_, true);

    //mark cells within the initial footprint of the robot
    for (unsigned int i = 0; i < footprint_list.size(); ++i) {
      path_map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
    }

    //make sure that we update our path based on the global plan and compute costs
    path_map_.setTargetCells(*costmap_, global_plan_);
    goal_map_.setLocalGoal(*costmap_, global_plan_);
    ROS_DEBUG("Path/Goal distance computed");

    //rollout trajectories and find the minimum cost one
    Trajectory best = createTrajectories(pos[0], pos[1], pos[2],
                                         vel[0], vel[1], vel[2]);
    ROS_DEBUG("Trajectories created");
    if(best.cost_ < 0)
    {
      drive_velocities.pose.position.x = 0;
      drive_velocities.pose.position.y = 0;
      drive_velocities.pose.position.z = 0;
      drive_velocities.pose.orientation.w = 1;
      drive_velocities.pose.orientation.x = 0;
      drive_velocities.pose.orientation.y = 0;
      drive_velocities.pose.orientation.z = 0;
    }
    else
    {
      drive_velocities.pose.position.x = best.xv_;
      drive_velocities.pose.position.y = best.yv_;
      drive_velocities.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, best.thetav_);
      drive_velocities.pose.orientation = tf2::toMsg(q);
    }
    return best;
  }

  bool BotsAndUsPlanner::checkTrajectory(double x, double y, double theta,
                                         double vx, double vy, double vtheta,
                                         double vx_samp, double vy_samp, double vtheta_samp)
  {
    Trajectory t;
    double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0)
      return true;
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, cost);
    return false;
  }

  double BotsAndUsPlanner::scoreTrajectory(double x, double y, double theta,
                                           double vx, double vy, double vtheta,
                                           double vx_samp, double vy_samp, double vtheta_samp)
  {
    Trajectory t;
    double impossible_cost = path_map_.obstacleCosts();
    double acc_x = boost::get<double>(fetchData("acc_lim_x")), acc_th = boost::get<double>(fetchData("acc_lim_theta"));
    generateTrajectory(x, y, theta,
                       vx, vy, vtheta,
                       vx_samp, vy_samp, vtheta_samp,
                       acc_x, 0.0, acc_th,
                       impossible_cost, t);
    // return the cost
    return double( t.cost_ );
  }

  bool BotsAndUsPlanner::rotateToGoal(const geometry_msgs::PoseStamped& robot_vel, double goal_th,
                                      geometry_msgs::Twist& cmd_vel, const bool is_init)
  {
    updateGlobalPose();
    if(is_init)
    {//prepare map grid for cost computations
      path_map_.resetPathDist();
      goal_map_.resetPathDist();
      //make sure that we update our path based on the global plan and compute costs
      path_map_.setTargetCells(*costmap_, global_plan_);
      goal_map_.setLocalGoal(*costmap_, global_plan_);
    }

    double yaw = getYaw(global_pose_.pose.orientation);
    double vel_yaw = getYaw(robot_vel.pose.orientation);
    ROS_WARN_STREAM("Goal Yaw: " << goal_th << " Robot Yaw: " << yaw);
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

    //compute feasible velocity limits in robot space
    double min_vel_th = boost::get<double>(fetchData("min_rot_vel"));
    double max_vel_th = boost::get<double>(fetchData("max_rot_vel"));
    double acc_th = boost::get<double>(fetchData("acc_lim_theta"));
    double sim_time = boost::get<double>(fetchData("sim_time"));

    //NB: using min_vel_th as min_in_place_th
    double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th,
            std::max(min_vel_th, ang_diff)) : std::max(min_vel_th,
            std::min(-1.0 * min_vel_th, ang_diff));
    double max_acc_vel = fabs(vel_yaw) + acc_th * sim_time;
    double min_acc_vel = fabs(vel_yaw) - acc_th * sim_time;
    v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);
    //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt(2 * acc_th * fabs(ang_diff));
    v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));
    // Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
    v_theta_samp = v_theta_samp > 0.0
        ? std::min( max_vel_th, std::max( min_vel_th, v_theta_samp ))
        : std::max( min_vel_th, std::min( -1.0 * min_vel_th, v_theta_samp ));
    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = checkTrajectory(global_pose_.pose.position.x, global_pose_.pose.position.y, yaw,
                                     robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw,
                                     0.0, 0.0, v_theta_samp);
    ROS_WARN("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);
    if(valid_cmd)
    {
      cmd_vel.angular.z = sign(v_theta_samp) * std::max(fabs(v_theta_samp), fabs(boost::get<double>(fetchData("min_in_place_vel_theta"))));
      return true;
    }

    cmd_vel.angular.z = 0.0;
    return false;
  }

  bool BotsAndUsPlanner::driveToGoal(const geometry_msgs::PoseStamped& robot_vel, geometry_msgs::Twist& cmd_vel)
  {
    updateGlobalPose();
    cmd_vel.angular.z = 0.0;
    cmd_vel.linear.y = 0.0;
    double yaw = getYaw(global_pose_.pose.orientation);

    //compute feasible velocity limits in robot space
    double min_vel_x = boost::get<double>(fetchData("min_vel_x"));
    double max_vel_x = boost::get<double>(fetchData("max_vel_x"));
    double acc_x = boost::get<double>(fetchData("acc_lim_x"));
    double sim_time = boost::get<double>(fetchData("sim_time"));

    //using v = u + at;
    max_vel_x = std::max(std::min(max_vel_x, robot_vel.pose.position.x + acc_x * sim_time), boost::get<double>(fetchData("min_vel_x")));
    min_vel_x = std::max(boost::get<double>(fetchData("min_vel_x")), robot_vel.pose.position.x - acc_x * sim_time);

    int vx_samples = boost::get<int>(fetchData("vx_samples"));
    double dvx = (max_vel_x - min_vel_x) / (vx_samples - 1);

    double vx_samp = min_vel_x;
    double vy_samp = 0.0;


    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = checkTrajectory(global_pose_.pose.position.x, global_pose_.pose.position.y, yaw,
                                     robot_vel.pose.position.x, robot_vel.pose.position.y, 0.0,
                                     vx_samp, 0.0, 0.0);
    if(valid_cmd)
    {
      cmd_vel.linear.x = vx_samp;
      return true;
    }

    cmd_vel.linear.x = 0.0;
    return false;
  }

  bool BotsAndUsPlanner::selectGoalPoint(std::vector<geometry_msgs::PoseStamped> &plan,
                                         Eigen::Vector2d &goal_vec, double &goal_th)
  {
    if(!updateGlobalPose())
    {
      ROS_ERROR("Unable to fetch robot pose from costmap!");
      return false;
    }
    if(plan.empty())
      return false;
    std::size_t idx;
    for(idx = 0; idx < plan.size(); idx++)
    {
      goal_vec[0] = plan[idx].pose.position.x;
      goal_vec[1] = plan[idx].pose.position.y;
      goal_th = getYaw(plan[idx].pose.orientation);
      if(!checkVisited(goal_vec) || visited_pts_.empty())
      {
        //break if chosen point not reached
        if(!isGoalPositionReached(goal_vec))
        {
          visited_pts_.push_back(goal_vec);
          //update local goal
          final_goal_x_ = goal_vec.x();
          final_goal_y_ = goal_vec.y();
          //check if this is the last point in plan
          if(idx == (plan.size() - 1))
          {
            //ROS_WARN("Last point!!");
            return false;
          }
          break;
        }
        else
          visited_pts_.push_back(goal_vec);
      }
    }

    ROS_WARN_STREAM("Goal @: " << goal_vec.x() << " | " << goal_vec.y() <<
                    " Robot @: " << global_pose_.pose.position.x << " | " << global_pose_.pose.position.y);
    return (plan.size() == 1) ? false : true;
  }

  void BotsAndUsPlanner::reset()
  {
    visited_pts_.clear();
  }

  bool BotsAndUsPlanner::checkVisited(const Eigen::Vector2d &pts)
  {
    double xy_tolerance = boost::get<double>(fetchData("xy_tolerance"));
    for(std::size_t t = 0; t < visited_pts_.size(); t++)
    {
      if(computeEuclidean(pts, visited_pts_[t]) <= xy_tolerance)
        return true;
    }
    return false;
  }

};
