#ifndef _PATH_PLANNING_H_
#define _PATH_PLANNING_H_

#include <iostream>
#include <ros/ros.h>

#include "path_planning/path_optimization.h"

class PathPlanning
{
    private:
        ros::NodeHandle nh_;
        // the optimized trajectory
        Eigen::MatrixXd pts;
    public:
        PathPlanning() {};
        ~PathPlanning() {};
        PathPlanning(ros::NodeHandle& nh) : nh_(nh), path_optimization(nh) {};

        void init();

        PathOptimization path_optimization;
};

#endif