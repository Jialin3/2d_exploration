#ifndef EXPLORATION_MANAGER_EXPLORE_COSTMAP_CLIENT_H_
#define EXPLORATION_MANAGER_EXPLORE_COSTMAP_CLIENT_H_

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace explorer {
class ExploreCostmapClient {
public:
    ExploreCostmapClient(ros::NodeHandle& nh, ros::NodeHandle& private_nh, const tf::TransformListener* tf_listener);
    // ~ExploreCostmapClient();
    geometry_msgs::Pose getRobotPose() const;
    costmap_2d::Costmap2D* getCostmap()
    {
        return &costmap_;
    }
    
    const costmap_2d::Costmap2D* getCostmap() const
    {
        return &costmap_;
    }

    const std::string& getGlobalFrameID() const
    {
        return global_frame_;
    }

    const std::string& getBaseFrameID() const
    {
        return robot_base_frame_;
    }

    void updateCostmap(const map_msgs::OccupancyGridUpdate& msg);
    bool getFrontier(geometry_msgs::Pose& frontier_pose);

protected:
    void updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void updatePartialMap(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);

    const tf::TransformListener* const tf_;
    costmap_2d::Costmap2D costmap_;

    std::string global_frame_;      ///< @brief The global frame for the costmap
    std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
    double transform_tolerance_;    ///< timeout before transform errors

private:
    ros::Subscriber costmap_sub_;
    ros::Subscriber costmap_updates_sub_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
};
}

#endif // EXPLORATION_MANAGER_EXPLORE_COSTMAP_CLIENT_H_