#include <path_tracker/path_tracker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_tracking_node", ros::init_options::AnonymousName);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  ros::NodeHandle global_nh(""), private_nh("~");

  p_tracker::PathTracker ptracker_obj(global_nh, private_nh);
  ptracker_obj.run();
  return 0;
}
