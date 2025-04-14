#include "ros/console.h"
#include <exploration_manager/explorer.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "explorer_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explorer::Explorer explorer(nh, private_nh);
  ros::spin();
  return 0;
}
