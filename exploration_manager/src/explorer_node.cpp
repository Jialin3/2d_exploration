#include "ros/console.h"
#include <exploration_manager/explorer.h>
#include <memory>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "explorer_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  auto tf = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tf_listener(*tf);

  auto costmap_ = std::make_shared<navit_costmap_2d::Costmap2DROS>(
      "planner_common_costmap", *tf);

  auto frontier_search_ = std::make_shared<explorer::ExploreFrontierSearch>(
      nh, private_nh, costmap_);

  auto explorer_ = std::make_shared<explorer::Explorer>(
      nh, private_nh, tf, costmap_, frontier_search_);
  ros::spin();
  return 0;
}
