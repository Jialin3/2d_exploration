#include "exploration_manager/explorer.h"
#include "ros/this_node.h"

namespace explorer {
Explorer::Explorer(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    : nh_(nh), nh_private_(private_nh), tf_listener_(ros::Duration(5)),
      costmap_client_(nh_, nh_private_, &tf_listener_), prev_distance_(0),
      last_markers_count_(0) {
  double timeout;
  double min_frontier_size;
  nh_.param("planner_frequency", planner_frequency_, 1.0);
  nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  nh_.param("visualize", visualize_, false);
  nh_.param("potential_scale", potential_scale_, 1e-3);
  nh_.param("orientation_scale", orientation_scale_, 0.0);
  nh_.param("gain_scale", gain_scale_, 1.0);
  nh_.param("min_frontier_size", min_frontier_size, 0.5);

  search_ =
      ExploreFrontierSearch(costmap_client_.getCostmap(), potential_scale_,
                            gain_scale_, min_frontier_size);

  // 添加：记录初始位置
  auto initial_pose = costmap_client_.getRobotPose();
  origin_ = initial_pose.position;
  has_origin_ = true;
  ROS_INFO("Origin position recorded: (%.2f, %.2f)", origin_.x, origin_.y);

  if (visualize_) {
    marker_array_publisher_ =
        nh_private_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ac_ = std::make_shared<Client>(kActionServerA2BMove, true);
  ROS_INFO("Waiting to connect to action_task_a2b_move server");
  ac_->waitForServer();
  ROS_INFO("Connected to action_task_a2b_move server");

  // ROS_INFO("Waiting to connect to move_base server");
  // move_base_client_.waitForServer();
  // ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      nh_private_.createTimer(ros::Duration(1. / planner_frequency_),
                              [this](const ros::TimerEvent &) { makePlan(); });
}

Explorer::~Explorer() { stop(); }

void Explorer::visualizeFrontiers(const std::vector<Frontier> &frontiers) {
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;
  std_msgs::ColorRGBA purple;
  purple.r = 1.0;
  purple.g = 0.0;
  purple.b = 1.0;
  purple.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker> &markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto &frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.nearest_free)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;

    // 最近自由点（紫色）
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id++);
    m.pose.position = frontier.nearest_free;
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
    m.points = {};
    m.color = purple;
    markers.push_back(m);
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explorer::makePlan() {
  // 如果当前有正在执行的目标且未超时，则跳过重新规划
  if (current_goal_active_ &&
      (ros::Time::now() - current_goal_start_time_ < ros::Duration(10.0))) {
    ROS_DEBUG_THROTTLE(5, "Current goal still active, skipping replan");
    return;
  }

  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    stop();
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier = std::find_if_not(
      frontiers.begin(), frontiers.end(),
      [this](const Frontier &f) { return goalOnBlacklist(f.nearest_free); });
  // if (current_goal_.cost * 0.8 < frontier->cost && current_goal_ != nullptr)
  // {
  //   ROS_WARN("Current goal cost is too high, skipping replan");
  //   return;
  // }

  if (frontier == frontiers.end()) {
    stop();
    return;
  }
  // geometry_msgs::Point target_position = frontier->centroid;
  geometry_msgs::Point target_position = frontier->nearest_free;

  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (ros::Time::now() - last_progress_ > progress_timeout_) {
    frontier_blacklist_.push_back(target_position);
    ROS_DEBUG("Adding current goal to black list");
    makePlan();
    return;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return;
  }

  // 如果新目标与当前目标相同，跳过
  if (current_goal_active_ &&
      frontier->nearest_free == current_goal_.nearest_free) {
    return;
  }

  // 发送新目标
  sendNewGoal(target_position);

  // 更新当前目标状态
  current_goal_ = *frontier;
  current_goal_active_ = true;
  current_goal_start_time_ = ros::Time::now();
}

void Explorer::sendNewGoal(const geometry_msgs::Point &target_position) {
  alg_public_msgs::TaskA2BMoveGoal goal;
  goal.goal_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.goal_pose.header.stamp = ros::Time::now();
  goal.goal_pose.pose.position = target_position;
  goal.goal_pose.pose.orientation.w = 1.;
  goal.arrival_mode = "precise";
  goal.arrival_radius = 0;
  goal.avoiding_distance_level = 2;
  goal.max_velocity = 1.0;
  goal.recovery_timeout = 60;

  ac_->sendGoal(goal, [this, target_position](
                          const actionlib::SimpleClientGoalState &status,
                          const auto &result) {
    reachedGoal(status, result, target_position);
  });
}

bool Explorer::goalOnBlacklist(const geometry_msgs::Point &goal) {
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D *costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto &frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explorer::reachedGoal(
    const actionlib::SimpleClientGoalState &status,
    const alg_public_msgs::TaskA2BMoveResultConstPtr &result,
    const geometry_msgs::Point &frontier_goal) {
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  current_goal_active_ = false;

  // 处理任务失败逻辑
  if (status == actionlib::SimpleClientGoalState::ABORTED ||
      status == actionlib::SimpleClientGoalState::REJECTED ||
      status == actionlib::SimpleClientGoalState::PREEMPTED) {
    // 1. 将失败的目标加入黑名单
    frontier_blacklist_.push_back(frontier_goal);
    ROS_WARN("Task failed (status: %s), adding goal to blacklist.",
             status.toString().c_str());
  }
  makePlan();
}

void Explorer::start() { exploring_timer_.start(); }

void Explorer::stop() {
  ac_->cancelAllGoals();
  exploring_timer_.stop();
  current_goal_active_ = false;

  // 2. 清除所有可视化标记
  if (visualize_) {
    clearMarkers();
  }

  // 3. 如果记录了原点，则导航回原点
  if (has_origin_) {
    ROS_INFO("Returning to origin (%.2f, %.2f)", origin_.x, origin_.y);

    alg_public_msgs::TaskA2BMoveGoal goal;
    goal.goal_pose.header.frame_id = costmap_client_.getGlobalFrameID();
    goal.goal_pose.header.stamp = ros::Time::now();
    goal.goal_pose.pose.position = origin_;
    goal.goal_pose.pose.orientation.w = 1.0;
    goal.arrival_mode = "precise";
    goal.arrival_radius = 0.5; // 允许0.5米的到达误差
    goal.avoiding_distance_level = 2;
    goal.max_velocity = 1.0;
    goal.recovery_timeout = 60;

    ac_->sendGoal(goal);
    ROS_INFO("Return-to-origin command sent");
  } else {
    ROS_WARN("No origin position recorded. Cannot return to origin.");
  }

  ROS_INFO("Exploration stopped.");
}

void Explorer::clearMarkers() {
  visualization_msgs::MarkerArray markers_msg;
  visualization_msgs::Marker marker;

  marker.header.frame_id = costmap_client_.getGlobalFrameID();
  marker.header.stamp = ros::Time::now();
  marker.ns = "frontiers";
  marker.action = visualization_msgs::Marker::DELETEALL;

  markers_msg.markers.push_back(marker);
  marker_array_publisher_.publish(markers_msg);

  ROS_DEBUG("Cleared all visualization markers");
}

} // namespace explorer
