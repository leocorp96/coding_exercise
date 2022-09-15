#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tf2_ros/transform_listener.h>
#include <botsandus_planner/botsandus_planner_ros.h>

class BotsAndUsPlannerTest: public ::testing::Test
{
protected:
  void SetUp() override
  {
    robot_params_["max_rot_vel"] = 0.8;
    robot_params_["min_rot_vel"] = 0.35;
    robot_params_["min_in_place_vel_theta"] = 0.2;
    robot_params_["max_vel_x"] = 1.0;
    robot_params_["min_vel_x"] = 0.25;
    robot_params_["acc_lim_x"] = 2.5;
    robot_params_["acc_lim_y"] = 0.0;
    robot_params_["acc_lim_theta"] = 3.2;
    robot_params_["xy_tolerance"] = 0.1;
    robot_params_["yaw_tolerance"] = 0.05;
    robot_params_["stopped_rot_vel"] = 0.05;
    robot_params_["stopped_trans_vel"] = 0.1;
    robot_params_["vx_samples"] = 15;
    robot_params_["vth_samples"] = 15;
    robot_params_["ins_radius"] = 0.2;
    robot_params_["circ_radius"] = 0.2;
    robot_params_["sim_time"] = 2.0;
    robot_params_["ang_sim_granularity"] = 0.08;
    robot_params_["sim_granularity"] = 0.08;
    robot_params_["heading_scoring"] = false;
    robot_params_["heading_scoring_timestep"] = 2.0;
    robot_params_["simp_attractor"] = false;

    robot_params_["path_dist_bias"] = 5.0;
    robot_params_["goal_dist_bias"] = 1.5;
    robot_params_["occdist_scale"] = 0.01;
    robot_params_["look_ahead"] = 0.8;
    robot_params_["oscillation_reset_dist"] = 0.2;

    robot_params_["prune_plan"] = true;

    robot_params_["dsample_dist"] = 0.25;
    robot_params_["use_p2p"] = true;
    robot_params_["use_static"] = true;

    bup_.reset(new bup_local_planner::BotsAndUsPlanner(std::string("BotsAndUsPlannerROS"), nullptr, robot_params_));
  }

  RobotParams robot_params_;
  boost::shared_ptr<bup_local_planner::BotsAndUsPlanner> bup_;
};

TEST_F(BotsAndUsPlannerTest, parameterTest)
{//test parameter accessing
  ASSERT_EQ(0.8, boost::get<double>(bup_->fetchData("max_rot_vel")));
  ASSERT_EQ(0.35, boost::get<double>(bup_->fetchData("min_rot_vel")));
  ASSERT_EQ(0.2, boost::get<double>(bup_->fetchData("min_in_place_vel_theta")));
  ASSERT_EQ(1.0, boost::get<double>(bup_->fetchData("max_vel_x")));
  ASSERT_EQ(0.25, boost::get<double>(bup_->fetchData("min_vel_x")));
  ASSERT_EQ(2.5, boost::get<double>(bup_->fetchData("acc_lim_x")));
  ASSERT_EQ(0.0, boost::get<double>(bup_->fetchData("acc_lim_y")));
  ASSERT_EQ(3.2, boost::get<double>(bup_->fetchData("acc_lim_theta")));
  ASSERT_EQ(0.1, boost::get<double>(bup_->fetchData("xy_tolerance")));
  ASSERT_EQ(0.05, boost::get<double>(bup_->fetchData("yaw_tolerance")));
  ASSERT_EQ(0.05, boost::get<double>(bup_->fetchData("stopped_rot_vel")));
  ASSERT_EQ(0.1, boost::get<double>(bup_->fetchData("stopped_trans_vel")));
  ASSERT_EQ(15, boost::get<int>(bup_->fetchData("vx_samples")));
  ASSERT_EQ(15, boost::get<int>(bup_->fetchData("vth_samples")));
  ASSERT_EQ(0.2, boost::get<double>(bup_->fetchData("ins_radius")));
  ASSERT_EQ(0.2, boost::get<double>(bup_->fetchData("circ_radius")));
  ASSERT_EQ(2.0, boost::get<double>(bup_->fetchData("sim_time")));
  ASSERT_EQ(0.08, boost::get<double>(bup_->fetchData("ang_sim_granularity")));
  ASSERT_EQ(0.08, boost::get<double>(bup_->fetchData("sim_granularity")));
  ASSERT_EQ(false, boost::get<bool>(bup_->fetchData("heading_scoring")));
  ASSERT_EQ(2.0, boost::get<double>(bup_->fetchData("heading_scoring_timestep")));
  ASSERT_EQ(false, boost::get<bool>(bup_->fetchData("simp_attractor")));
  ASSERT_EQ(5.0, boost::get<double>(bup_->fetchData("path_dist_bias")));
  ASSERT_EQ(1.5, boost::get<double>(bup_->fetchData("goal_dist_bias")));
  ASSERT_EQ(0.01, boost::get<double>(bup_->fetchData("occdist_scale")));
  ASSERT_EQ(0.8, boost::get<double>(bup_->fetchData("look_ahead")));
  ASSERT_EQ(0.2, boost::get<double>(bup_->fetchData("oscillation_reset_dist")));
  ASSERT_EQ(true, boost::get<bool>(bup_->fetchData("prune_plan")));
  ASSERT_EQ(0.25, boost::get<double>(bup_->fetchData("dsample_dist")));
  ASSERT_EQ(true, boost::get<bool>(bup_->fetchData("use_p2p")));
  ASSERT_EQ(true, boost::get<bool>(bup_->fetchData("use_static")));
}

TEST_F(BotsAndUsPlannerTest, movementTest)
{
  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = -0.01;
  odom.pose.pose.position.y = -0.52;
  odom.pose.pose.orientation.x = 0.00;
  odom.pose.pose.orientation.y = 0.00;
  odom.pose.pose.orientation.z = -0.16;
  odom.pose.pose.orientation.w = -0.99;
  odom.twist.twist.linear.x = 0.0799;
  odom.twist.twist.angular.z = -0.0006;
  //test
  ASSERT_TRUE(bup_->isMoving(odom)) << "Moving test failed!";
}

TEST_F(BotsAndUsPlannerTest, downsampleTest)
{
  //create a random path
  std::vector<geometry_msgs::PoseStamped> plan;
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::uniform_int_distribution<> distr_01(-100, 100);
  std::uniform_int_distribution<> distr_010(-100, 100);
  for(std::size_t i = 0; i < 100; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = (distr_010(gen)/ 10.0);
    pose.pose.position.y = (distr_010(gen)/ 10.0);
    pose.pose.orientation.w = (distr_01(gen)/ 100.0);
    pose.pose.orientation.x = (distr_01(gen)/ 100.0);
    pose.pose.orientation.y = (distr_01(gen)/ 100.0);
    pose.pose.orientation.z = (distr_01(gen)/ 100.0);
    plan.push_back(pose);
  }

  //test
  ASSERT_EQ(100, plan.size()) << "Plan size mismatch!";

  double pre_sample_size = plan.size();
  bup_->downSamplePlan(plan);
  double post_sample_size = plan.size();
  ASSERT_GE(pre_sample_size, post_sample_size) << "De-sampling process failed!";
}

TEST_F(BotsAndUsPlannerTest, goalSelectTest)
{
  std::vector<geometry_msgs::PoseStamped> plan;
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::uniform_int_distribution<> distr_01(-100, 100);
  std::uniform_int_distribution<> distr_010(-100, 100);
  for(std::size_t i = 0; i < 100; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = (distr_010(gen)/ 10.0);
    pose.pose.position.y = (distr_010(gen)/ 10.0);
    pose.pose.orientation.w = (distr_01(gen)/ 100.0);
    pose.pose.orientation.x = (distr_01(gen)/ 100.0);
    pose.pose.orientation.y = (distr_01(gen)/ 100.0);
    pose.pose.orientation.z = (distr_01(gen)/ 100.0);
    plan.push_back(pose);
  }
  //test
  ASSERT_TRUE(!plan.empty()) << "Plan is empty!";

  for(std::size_t t = 1; t < plan.size(); t++)
  {
    //robot pose
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.pose.position.x = plan[t - 1].pose.position.x;
    robot_pose.pose.position.y = plan[t - 1].pose.position.y;
    bup_->_setGlobalPose(robot_pose);

    //fetch goal pts
    Eigen::Vector2d goal_vec;
    double goal_th;
    std::size_t idx;
    if(t < (plan.size()-2))
    {
      if(t == (plan.size() - 1))
      {
        EXPECT_FALSE(bup_->selectGoalPoint(plan, goal_vec, goal_th, idx)) <<
                 "Goal Point selection failed @1 index [" << t << "] ";
      }
      else
      {
        EXPECT_TRUE(bup_->selectGoalPoint(plan, goal_vec, goal_th, idx)) <<
                 "Goal Point selection failed @2 index [" << t << "] ";
      }
    }
    else bup_->selectGoalPoint(plan, goal_vec, goal_th, idx);

    ASSERT_EQ(goal_vec.x(), plan[idx].pose.position.x) << "Goal -x mismatch!";
    ASSERT_EQ(goal_vec.y(), plan[idx].pose.position.y) << "Goal -y mismatch!";
    ASSERT_EQ(goal_th, bup_local_planner::getYaw(plan[idx].pose.orientation)) << "Goal theta mismatch!";
  }
  bup_->reset(); //clear the visited points container: that will be tested next
}

TEST_F(BotsAndUsPlannerTest, checkVisitedTest)
{
  std::vector<geometry_msgs::PoseStamped> plan;
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::uniform_int_distribution<> distr_01(-100, 100);
  std::uniform_int_distribution<> distr_010(-100, 100);
  for(std::size_t i = 0; i < 100; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = (distr_010(gen)/ 10.0);
    pose.pose.position.y = (distr_010(gen)/ 10.0);
    pose.pose.orientation.w = (distr_01(gen)/ 100.0);
    pose.pose.orientation.x = (distr_01(gen)/ 100.0);
    pose.pose.orientation.y = (distr_01(gen)/ 100.0);
    pose.pose.orientation.z = (distr_01(gen)/ 100.0);
    plan.push_back(pose);
  }

  //test
  ASSERT_TRUE(!plan.empty()) << "Plan is empty!";

  for(std::size_t t = 1; t < (plan.size()-1); t++)
  {
    //robot pose
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.pose.position.x = plan[t - 1].pose.position.x;
    robot_pose.pose.position.y = plan[t - 1].pose.position.y;
    bup_->_setGlobalPose(robot_pose);

    //fetch goal pts
    Eigen::Vector2d goal_vec;
    double goal_th;
    std::size_t idx;
    bup_->selectGoalPoint(plan, goal_vec, goal_th, idx);
    std::vector<Eigen::Vector2d> visited_pts = bup_->_visitedPoints();
    ASSERT_EQ((t + 1), visited_pts.size()) << "Visited count mismatch!";
  }

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_botsandusplanner");
  return RUN_ALL_TESTS();;
}
