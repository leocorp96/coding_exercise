#include <ros/ros.h>
#include <gtest/gtest.h>
#include <path_tracker/path_tracker.h>

class PathTrackerTest: public ::testing::Test
{
protected:
  void SetUp() override
  {
    ptracker_.reset(new p_tracker::PathTracker(g_nh, p_nh));
    geometry_msgs::Pose2D goal;
    goal.x = 0.38; goal.y = -0.52;
    goal.theta = 0.03;
    nav_msgs::Odometry odom, prev_odom;
    odom.pose.pose.position.x = -0.01; odom.pose.pose.position.y = -0.52;
    odom.pose.pose.orientation.x = 0.00;
    odom.pose.pose.orientation.y = 0.00;
    odom.pose.pose.orientation.z = -0.16;
    odom.pose.pose.orientation.w = -0.99;

    prev_odom.pose.pose.position.x = -0.07; prev_odom.pose.pose.position.y = -0.54;
    prev_odom.pose.pose.orientation.x = 0.00;
    prev_odom.pose.pose.orientation.y = 0.00;
    prev_odom.pose.pose.orientation.z = -0.16;
    prev_odom.pose.pose.orientation.w = -0.99;
    nav_msgs::Path plan;
    {
      geometry_msgs::PoseStamped pt;
      pt.pose.position.x = -0.66;
      pt.pose.position.y = -0.33;
      pt.pose.orientation.x = 0.00;
      pt.pose.orientation.y = 0.00;
      pt.pose.orientation.z = 0.00;
      pt.pose.orientation.w = 1.0;

      plan.poses.push_back(pt);
      pt.pose.position.x = -0.43;
      pt.pose.position.y = -0.47;
      plan.poses.push_back(pt);
      pt.pose.position.x = -0.17;
      pt.pose.position.y = -0.53;
      plan.poses.push_back(pt);
      pt.pose.position.x = 0.10;
      pt.pose.position.y = -0.53;
      plan.poses.push_back(pt);
      pt.pose.position.x = 0.38;
      pt.pose.position.y = -0.52;
      plan.poses.push_back(pt);
      pt.pose.position.x = 0.65;
      pt.pose.position.y = -0.54;
      plan.poses.push_back(pt);
    }

    ptracker_->_testConfig(goal, odom, prev_odom, plan, 0.08, 2.0, 0.05);
  }

  boost::shared_ptr<p_tracker::PathTracker> ptracker_;
  geometry_msgs::Pose2D start_, robot_pose_;
  ros::NodeHandle g_nh, p_nh;
};

TEST_F(PathTrackerTest, startCoordinateTest)
{
  ptracker_->_getStartCoordinates(start_);
  //test start content
  ASSERT_EQ(0.10, start_.x) << "Start Coordinate-X failed!";
  ASSERT_EQ(-0.53, start_.y) << "Start Coordinate-Y failed!";
  geometry_msgs::Quaternion qt;
  qt.x = 0.00;
  qt.y = 0.00;
  qt.z = 0.00;
  qt.w = 1.0;
  EXPECT_EQ(p_tracker::getYaw(qt), start_.theta) << "Start Coordinate-Yaw failed!";
}

TEST_F(PathTrackerTest, robotPoseTest)
{
  ptracker_->_getGlobalPose(robot_pose_);
  // test here
  EXPECT_EQ(-0.01, robot_pose_.x) << "Robot pose Coordinate-X failed!";
  EXPECT_EQ(-0.52, robot_pose_.y) << "Robot pose Coordinate-Y failed!";
  geometry_msgs::Quaternion qt;
  qt.x = 0.00;
  qt.y = 0.00;
  qt.z = -0.16;
  qt.w = -0.99;
  EXPECT_EQ(p_tracker::getYaw(qt), robot_pose_.theta) << "Robot pose Coordinate-Yaw failed!";
}

TEST_F(PathTrackerTest, pathErrorTest)
{
  ptracker_->_getStartCoordinates(start_);
  ptracker_->_getGlobalPose(robot_pose_);
  double res = ptracker_->_computePathError(robot_pose_, start_);
  //test here => -3.01523
  double value = std::ceil(res * 100.0) / 100.0;
  ASSERT_EQ(-3.01, value) << "Robot Deviation from path check failed!";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_pathtracker");
  return RUN_ALL_TESTS();;
}
