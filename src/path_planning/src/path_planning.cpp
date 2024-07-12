#include "path_planning/path_planning.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle nh("~");
    // std::cout << "Namespace" << nh.getNamespace() << std::endl;
    PathPlanning path_planning(nh);
    path_planning.init();

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void PathPlanning::init() {
    path_optimization.init();
    ROS_INFO("PathPlanning init success!");
}