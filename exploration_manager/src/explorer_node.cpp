#include <exploration_manager/explorer.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "explorer_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    explorer::Explorer explorer(nh, private_nh);
    return 0;
}
