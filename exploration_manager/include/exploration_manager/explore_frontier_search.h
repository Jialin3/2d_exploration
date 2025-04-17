#ifndef EXPLORATION_MANAGER_EXPLORE_FRONTIER_SEARCH_H_
#define EXPLORATION_MANAGER_EXPLORE_FRONTIER_SEARCH_H_

#include <costmap_2d/costmap_2d.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <memory>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include "navit_costmap_2d/costmap_2d.h"
#include <navit_collision_checker/collision_checker.h>
#include <navit_collision_checker/footprint_collision_checker.h>
#include <navit_collision_checker/line_iterator.h>
#include <navit_costmap_2d/costmap_2d_ros.h>

namespace explorer {
struct Frontier {
  std::uint32_t size;
  double min_distance;
  double cost;
  geometry_msgs::Point initial;
  geometry_msgs::Point centroid;
  geometry_msgs::Point middle;
  geometry_msgs::Pose nearest_free; // 新增：最近自由点
  std::vector<geometry_msgs::Point> points;
};

struct FootprintConfig {
  std::string footprint_str =
      "[0.12,0.2],[0.22,0.09],[0.22,-0.09],[0.12,-0.2],[-0.12,-0.2],[-0.22,-0."
      "09],[-0.22,0.09],[-0.12,0.2]";
  navit_collision_checker::Footprint footprint;
  double angle_sample_coefficient = 0.1;
  double sample_d_t = 0.1;
  double sample_max_t = 1;
  double sample_default_t = 0.5;
  double sample_min_vel = 0.02;
  double sample_min_s = 0.2;
  int sample_step = 3;
  float robot_radius = 0.19;
};

class ExploreFrontierSearch {
public:
  ExploreFrontierSearch(
      ros::NodeHandle &nh, ros::NodeHandle &private_nh,
      std::shared_ptr<navit_costmap_2d::Costmap2DROS> &costmap_ros);

  std::vector<Frontier> searchFrom(geometry_msgs::Point position);

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  bool buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                        std::vector<bool> &frontier_flag, Frontier &output);

  bool findNearestFreePoint(const geometry_msgs::Point &point,
                            geometry_msgs::Pose &result);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool> &frontier_flag);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier &frontier);

  std::shared_ptr<navit_costmap_2d::Costmap2DROS> costmap_ros_;

  double footprintCostAtPose(double x, double y, double theta,
                             const navit_collision_checker::Footprint footprint,
                             const std::vector<unsigned char> &ignore_costs);
  double footprintCost(const navit_collision_checker::Footprint footprint,
                       const std::vector<unsigned char> &ignore_costs);
  double lineCost(int x0, int x1, int y0, int y1,
                  const std::vector<unsigned char> &ignore_costs) const;
  double pointCost(int x, int y) const;

  FootprintConfig footprint_config_;

  void StringToFootprint(const std::string &str,
                         navit_collision_checker::Footprint *footprint);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::shared_ptr<navit_costmap_2d::Costmap2D> costmap_;
  unsigned char *map_;
  unsigned int size_x_, size_y_;
  double potential_scale_, gain_scale_;
  double min_frontier_size_;
};

} // namespace explorer

#endif // EXPLORATION_MANAGER_EXPLORE_FRONTIER_SEARCH_H_