#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <exploration_manager/explore_costmap_tools.h>
#include <exploration_manager/explore_frontier_search.h>

namespace explorer {
using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
ExploreFrontierSearch::ExploreFrontierSearch(costmap_2d::Costmap2D *costmap,
                                             double potential_scale,
                                             double gain_scale,
                                             double min_frontier_size)
    : costmap_(costmap), potential_scale_(potential_scale),
      gain_scale_(gain_scale), min_frontier_size_(min_frontier_size) {}


std::vector<Frontier> ExploreFrontierSearch::searchFrom(geometry_msgs::Point position)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

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
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
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
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

Frontier ExploreFrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
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
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
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
  output.nearest_free = findNearestFreePoint(output.centroid);
        
  // 验证自由点有效性
  unsigned int fx, fy;
  if(!costmap_->worldToMap(output.nearest_free.x, output.nearest_free.y, fx, fy) ||
  costmap_->getCost(fx, fy) != costmap_2d::FREE_SPACE) {
      ROS_WARN("Failed to find valid free point, using middle point");
      output.nearest_free = output.middle;
  }
  return output;
}

// 新增辅助函数：寻找最近自由点
geometry_msgs::Point ExploreFrontierSearch::findNearestFreePoint(const geometry_msgs::Point& point) {
  unsigned int mx, my;
  if(!costmap_->worldToMap(point.x, point.y, mx, my)) {
      ROS_ERROR("Worldtomap error??????");
      return point; // 如果转换失败返回原始点
  }

  // 搜索半径（栅格单位）
  // const int max_radius = std::min(30, static_cast<int>(3.0/nav_costmap_->getResolution()));
  const int max_radius = std::max(30, static_cast<int>(2.0/costmap_->getResolution()));
  geometry_msgs::Point result = point;
  double min_dist = std::numeric_limits<double>::max();

  // 螺旋式搜索
  for(int r=1; r<=max_radius; ++r) {
      for(int dy=-r; dy<=r; ++dy) {
          for(int dx=-r; dx<=r; ++dx) {
              // if(abs(dx)!=r && abs(dy)!=r) continue; // 只检查外环
              
              unsigned int nx = mx + dx;
              unsigned int ny = my + dy;
              
              if(nx < costmap_->getSizeInCellsX() && ny < costmap_->getSizeInCellsY()) {
                  if(costmap_->getCost(nx, ny) == costmap_2d::FREE_SPACE) {
                      double wx, wy;
                      costmap_->mapToWorld(nx, ny, wx, wy);
                      double dist = hypot(wx-point.x, wy-point.y);
                      if(dist < min_dist) {
                          min_dist = dist;
                          result.x = wx;
                          result.y = wy;
                      }
                  }
              }
          }
      }
      // if(min_dist < std::numeric_limits<double>::max()) break; // 找到即退出
  }

  return result;
}

bool ExploreFrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double ExploreFrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());

  // // 原始成本计算
  // double distance_term = potential_scale_ * frontier.min_distance * costmap_->getResolution();
  // double size_term = gain_scale_ * frontier.size * costmap_->getResolution();
  // double raw_cost = distance_term - size_term;

  // // Sigmoid归一化到 (0,1)
  // double normalized_cost = 1.0 / (1.0 + exp(-raw_cost));
  
  // return normalized_cost;
}
} // namespace explorer
