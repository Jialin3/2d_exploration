#ifndef EXPLORER_H
#define EXPLORER_H

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

namespace explorer {
class Explorer {
public:


    enum struct RunModeType {
        kSim = 0,  // Run in simulation.
        kReal= 1     // Run with real robot.
    };

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    Explorer(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    // ~Explorer();

private:
    bool loadParams();
    RunModeType run_mode_;
};
}

#endif // EXPLORER_H