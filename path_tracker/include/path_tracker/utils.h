#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include <string>
#include <cmath>
#include <angles/angles.h>
#include <Eigen/Core>

namespace p_tracker
{
  static double computeEuclidean(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
  {
    return hypot(p1.x() - p2.x(), p1.y() - p2.y());
  }

  static double getYaw(const geometry_msgs::Quaternion &q_msg)
  {
    tf2::Quaternion q_tf;
    try
    {
      tf2::convert(q_msg, q_tf);
    } catch (tf2::TransformException &tfex)
    {
      std::cout << "Exception in getYaw: " << tfex.what();
      tf2::fromMsg(q_msg, q_tf);
    }
    tf2::Matrix3x3 m(q_tf);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }
};


#endif // UTILS_H
