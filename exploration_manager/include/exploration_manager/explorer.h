#ifndef EXPLORATION_MANAGER_EXPLORER_H
#define EXPLORATION_MANAGER_EXPLORER_H

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <exploration_manager/explore_costmap_client.h>
#include <exploration_manager/explore_frontier_search.h>

#include <alg_public_msgs/TaskA2BMoveAction.h>

namespace explorer {
class Explorer {
public:
  enum struct RunModeType {
    kSim = 0, // Run in simulation.
    kReal = 1 // Run with real robot.
  };

  Explorer(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  ~Explorer();

  void start();
  void stop();

private:
  bool loadParams();
  RunModeType run_mode_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::string kNodeName = "test_a2b_move";
  const char *kActionServerA2BMove = "/action_task_a2b_move";

  using Client =
      actionlib::SimpleActionClient<alg_public_msgs::TaskA2BMoveAction>;
  using ClientPtr = std::shared_ptr<Client>;

  geometry_msgs::Point origin_; // 原点
  bool has_origin_ = false;     // 原点是否已设置

  void makePlan();

  void visualizeFrontiers(const std::vector<Frontier> &frontiers);

  void clearMarkers();

  void reachedGoal(const actionlib::SimpleClientGoalState &status,
                   const alg_public_msgs::TaskA2BMoveResultConstPtr &result,
                   const geometry_msgs::Point &frontier_goal);

  bool goalOnBlacklist(const geometry_msgs::Point &goal);

//   ros::NodeHandle private_nh_;
//   ros::NodeHandle relative_nh_;
  ros::Publisher marker_array_publisher_;
  tf::TransformListener tf_listener_;

  ExploreCostmapClient costmap_client_;
  std::shared_ptr<
      actionlib::SimpleActionClient<alg_public_msgs::TaskA2BMoveAction>>
      ac_;
  ExploreFrontierSearch search_;
  ros::Timer exploring_timer_;
  ros::Timer oneshot_;

  std::vector<geometry_msgs::Point> frontier_blacklist_;
  geometry_msgs::Point prev_goal_;
  double prev_distance_;
  ros::Time last_progress_;
  size_t last_markers_count_;

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  ros::Duration progress_timeout_;
  bool visualize_;

  bool current_goal_active_ = false;
  Frontier current_goal_;
  ros::Time current_goal_start_time_;
  void sendNewGoal(const geometry_msgs::Point &target_position);
};
} // namespace explorer

#endif // EXPLORATION_MANAGER_EXPLORER_H