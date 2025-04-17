#include "navit_costmap_2d/cost_values.h"
#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <exploration_manager/explore_costmap_tools.h>
#include <exploration_manager/explore_frontier_search.h>

namespace explorer {
using navit_costmap_2d::FREE_SPACE;
using navit_costmap_2d::LETHAL_OBSTACLE;
using navit_costmap_2d::NO_INFORMATION;
ExploreFrontierSearch::ExploreFrontierSearch(
    ros::NodeHandle &nh, ros::NodeHandle &private_nh,
    std::shared_ptr<navit_costmap_2d::Costmap2DROS> &costmap_ros)
    : costmap_ros_(costmap_ros), nh_(nh), nh_private_(private_nh) {
  private_nh.param("potential_scale", potential_scale_, 1.0);
  private_nh.param("gain_scale", gain_scale_, 1.0);
  private_nh.param("min_frontier_size", min_frontier_size_, 1.0);
  ROS_WARN("....footprint_str: %s", footprint_config_.footprint_str.c_str());
  StringToFootprint(footprint_config_.footprint_str,
                    &footprint_config_.footprint);
}

void ExploreFrontierSearch::StringToFootprint(
    const std::string &str, navit_collision_checker::Footprint *footprint) {
  std::vector<std::string> point_strs;
  footprint->clear();
  boost::algorithm::split(point_strs, str, boost::algorithm::is_any_of("][,"));
  for (size_t i = 1; i < point_strs.size() - 1; i += 4) {
    geometry_msgs::Point pt;
    pt.x = std::stod(point_strs[i]);
    pt.y = std::stod(point_strs[i + 1]);
    footprint->push_back(pt);
  }
}

std::vector<Frontier>
ExploreFrontierSearch::searchFrom(geometry_msgs::Point position) {
  std::vector<Frontier> frontier_list;
  costmap_ = costmap_ros_->getMapClone();

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search 副本带锁

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    ROS_WARN("Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier;
        if (new_frontier.size * costmap_->getResolution() >=
                min_frontier_size_ &&
            buildNewFrontier(nbr, pos, frontier_flag, new_frontier)) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto &frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier &f1, const Frontier &f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

bool ExploreFrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                             unsigned int reference,
                                             std::vector<bool> &frontier_flag,
                                             Frontier &output) {
  // initialize frontier structure
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;

  // 新增：计算最近自由点
  bool have_free_point =
      findNearestFreePoint(output.centroid, output.nearest_free);

  if (!have_free_point) {
    ROS_WARN("No free point found for frontier");
  }
  return have_free_point;
}

// 新增辅助函数：寻找最近自由点
bool ExploreFrontierSearch::findNearestFreePoint(
    const geometry_msgs::Point &point, geometry_msgs::Pose &result) {
  // 初始化
  bool have_free_pose = false;
  geometry_msgs::Pose tmp;
  unsigned int mx, my;
  if (!costmap_->worldToMap(point.x, point.y, mx, my)) {
    ROS_WARN("Point outside costmap");
    result.position = point;
    result.orientation.w = 1.0;
    return false;
  }

  // 配置参数
  const double max_search_radius = 3.0; // 米
  const int max_radius_cells =
      static_cast<int>(max_search_radius / costmap_->getResolution());

  // BFS搜索
  std::queue<std::pair<int, int>> queue;
  std::vector<std::vector<bool>> visited(
      costmap_->getSizeInCellsX(),
      std::vector<bool>(costmap_->getSizeInCellsY(), false));

  queue.emplace(mx, my);
  visited[mx][my] = true;

  geometry_msgs::Point best_point = point;
  double min_dist = std::numeric_limits<double>::max();

  while (!queue.empty()) {
    auto [cx, cy] = queue.front();
    queue.pop();

    // 检查当前点
    double wx, wy;
    costmap_->mapToWorld(cx, cy, wx, wy);
    double dist = hypot(wx - point.x, wy - point.y);

    // 计算朝向角（yaw），指向边界的质心
    double dx = point.x - wx;
    double dy = point.y - wy;
    double yaw = atan2(dy, dx);

    std::vector<unsigned char> ignore_costs;
    auto cost = footprintCostAtPose(wx, wy, yaw, footprint_config_.footprint,
                                    ignore_costs);

    std::vector<unsigned char> line_ignore_costs(1, 255);
    auto line_cost = lineCost(cx, mx, cy, my, line_ignore_costs);
    // 如果找到更近的自由点
    if (cost < 240 && dist < min_dist && (line_cost < 250)) {
      min_dist = dist;
      best_point.x = wx;
      best_point.y = wy;
      result.position = best_point;
      have_free_pose = true;

      // 计算朝向角（yaw），指向边界的质心
      double dx = point.x - wx;
      double dy = point.y - wy;
      double yaw = atan2(dy, dx);
      result.orientation =
          tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));

      // 如果足够近则提前终止
      if (min_dist < costmap_->getResolution())
        break;
    }

    // 扩展搜索
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0)
          continue;

        int nx = cx + dx;
        int ny = cy + dy;

        if (nx >= 0 && nx < costmap_->getSizeInCellsX() && ny >= 0 &&
            ny < costmap_->getSizeInCellsY() && !visited[nx][ny] &&
            dist + hypot(dx, dy) * costmap_->getResolution() <=
                max_search_radius) {

          visited[nx][ny] = true;
          queue.emplace(nx, ny);
        }
      }
    }
  }
  return have_free_pose;
}

double ExploreFrontierSearch::footprintCostAtPose(
    double x, double y, double theta,
    const navit_collision_checker::Footprint footprint,
    const std::vector<unsigned char> &ignore_costs) {
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  navit_collision_checker::Footprint oriented_footprint;
  for (unsigned int i = 0; i < footprint.size(); ++i) {
    geometry_msgs::Point new_pt;
    new_pt.x = x + (footprint[i].x * cos_th - footprint[i].y * sin_th);
    new_pt.y = y + (footprint[i].x * sin_th + footprint[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
    // ROS_ERROR("new_pt:x=%f y=%f",new_pt.x,new_pt.y);
  }

  return footprintCost(oriented_footprint, ignore_costs);
}

double ExploreFrontierSearch::footprintCost(
    const navit_collision_checker::Footprint footprint,
    const std::vector<unsigned char> &ignore_costs) {
  // now we really have to lay down the footprint in the costmap_ grid
  unsigned int x0, x1, y0, y1;
  double footprint_cost = 0.0;
  // we need to rasterize each line in the footprint
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the first point
    if (!costmap_->worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
      return static_cast<double>(LETHAL_OBSTACLE);
    }
    // get the cell coord of the second point
    if (!costmap_->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      return static_cast<double>(LETHAL_OBSTACLE);
    }

    footprint_cost =
        std::max(lineCost(x0, x1, y0, y1, ignore_costs), footprint_cost);
  }
  //  ROS_ERROR("footprint_cost=%f--------1",footprint_cost);
  // we also need to connect the first point in the footprint to the last point
  // get the cell coord of the last point
  if (!costmap_->worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
    return static_cast<double>(LETHAL_OBSTACLE);
  }

  // get the cell coord of the first point
  if (!costmap_->worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
    return static_cast<double>(LETHAL_OBSTACLE);
  }

  footprint_cost =
      std::max(lineCost(x0, x1, y0, y1, ignore_costs), footprint_cost);
  //  ROS_ERROR("footprint_cost=%f--------2",footprint_cost);
  // if all line costs are legal... then we can return that the footprint is
  // legal
  return footprint_cost;
}

double ExploreFrontierSearch::lineCost(
    int x0, int x1, int y0, int y1,
    const std::vector<unsigned char> &ignore_costs) const {
  double line_cost = -2.0;
  double point_cost = -1.0;

  for (navit_collision_checker::LineIterator line(x0, y0, x1, y1);
       line.isValid(); line.advance()) {
    point_cost = pointCost(line.getX(), line.getY()); // Score the current point
    if (!ignore_costs.empty()) {
      for (int i = 0; i < ignore_costs.size(); i++) {
        if (fabs(point_cost - ignore_costs[i]) < 1e-9) {
          point_cost = -1;
          break;
        }
      }
    }

    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }
  // ROS_ERROR("line_cost: %lf", line_cost);
  return line_cost;
}

double ExploreFrontierSearch::pointCost(int x, int y) const {
  return costmap_->getCost(x, y);
}

bool ExploreFrontierSearch::isNewFrontierCell(
    unsigned int idx, const std::vector<bool> &frontier_flag) {
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double ExploreFrontierSearch::frontierCost(const Frontier &frontier) {
  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
} // namespace explorer
