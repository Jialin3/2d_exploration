#ifndef EXPLORATION_MANAGER_EXPLORE_COSTMAP_TOOLS_H_
#define EXPLORATION_MANAGER_EXPLORE_COSTMAP_TOOLS_H_
// #include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <navit_costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>

namespace explorer {

std::vector<unsigned int>
nhood4(unsigned int idx,
       const std::shared_ptr<navit_costmap_2d::Costmap2D> &costmap) {
  // get 4-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out;

  unsigned int size_x_ = costmap->getSizeInCellsX(),
               size_y_ = costmap->getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    ROS_WARN("Evaluating nhood for offmap point");
    return out;
  }

  if (idx % size_x_ > 0) {
    out.push_back(idx - 1);
  }
  if (idx % size_x_ < size_x_ - 1) {
    out.push_back(idx + 1);
  }
  if (idx >= size_x_) {
    out.push_back(idx - size_x_);
  }
  if (idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + size_x_);
  }
  return out;
}

std::vector<unsigned int>
nhood8(unsigned int idx,
       const std::shared_ptr<navit_costmap_2d::Costmap2D> &costmap) {
  // get 8-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out = nhood4(idx, costmap);

  unsigned int size_x_ = costmap->getSizeInCellsX(),
               size_y_ = costmap->getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    return out;
  }

  if (idx % size_x_ > 0 && idx >= size_x_) {
    out.push_back(idx - 1 - size_x_);
  }
  if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx - 1 + size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
    out.push_back(idx + 1 - size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + 1 + size_x_);
  }

  return out;
}

bool nearestCell(unsigned int &result, unsigned int start, unsigned char val,
                 const std::shared_ptr<navit_costmap_2d::Costmap2D> &costmap) {
  const unsigned char *map = costmap->getCharMap();
  const unsigned int size_x = costmap->getSizeInCellsX(),
                     size_y = costmap->getSizeInCellsY();

  // ROS_DEBUG("[nearestCell] Start searching for value %d from index %u (map
  // size: %ux%u)", val, start, size_x, size_y);

  if (start >= size_x * size_y) {
    ROS_ERROR(
        "[nearestCell] Start index %u is out of bounds (map size: %u cells)",
        start, size_x * size_y);
    return false;
  }

  // Initialize breadth-first search
  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x * size_y, false);

  // Push initial cell
  bfs.push(start);
  visited_flag[start] = true;
  // ROS_DEBUG("[nearestCell] Starting BFS from index %u", start);

  // Search for neighbouring cell matching value
  unsigned int iterations = 0;
  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // ROS_DEBUG("[nearestCell] Checking index %u (value: %d)", idx, map[idx]);

    // Return if cell of correct value is found
    if (map[idx] == val) {
      result = idx;
      // ROS_INFO("[nearestCell] Found target value %d at index %u (iterations:
      // %u)", val, idx, iterations);
      return true;
    }

    for (unsigned nbr : nhood8(idx, costmap)) {
      if (!visited_flag[nbr]) {
        bfs.push(nbr);
        visited_flag[nbr] = true;
        // ROS_DEBUG("[nearestCell] Adding neighbor index %u to queue", nbr);
      }
    }

    iterations++;
    if (iterations % 1000 == 0) {
      ROS_WARN("[nearestCell] Search running long: %u iterations", iterations);
    }
  }

  ROS_WARN("[nearestCell] Target value %d not found after %u iterations", val,
           iterations);
  return false;
}

} // namespace explorer

#endif // EXPLORATION_MANAGER_EXPLORE_COSTMAP_TOOLS_H_