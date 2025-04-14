#include "explorer/explorer.h"
#include "ros/this_node.h"

namespace explorer {
Explorer::Explorer(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), nh_private_(private_nh) {
    loadParams();
}

bool Explorer::loadParams() {
    // 加载参数
    const std::string &ns = ros::this_node::getName();
    std::string parse_str;
    ros::param::get(ns + "/run_mode", parse_str);
    if (!parse_str.compare("kReal")) {
        run_mode_ = RunModeType::kReal;
    } else if (!parse_str.compare("kSim")) {
        run_mode_ = RunModeType::kSim;
    } else {
        ROS_ERROR("Run mode is not set properly, please check the parameter");
        return false;
    }

    if (run_mode_ == RunModeType::kSim) {
        ;
    }
    return true;

}



}



