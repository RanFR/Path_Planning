#ifndef _PATH_OPTIMIZATION_H_
#define _PATH_OPTIMIZATION_H_

#include <iostream>
#include <string>

#include <Eigen/Eigen>
#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>

#include "prrt/prrt.h"
#include "path_planning/Trajectory.h"

class PathOptimization {
    private:
        ros::NodeHandle nh_;
        ros::Timer path_optimization_timer_;
        // the optimized trajectory
        Eigen::MatrixXd traj_;
        Eigen::MatrixXd matrix_path_;
        std::vector<Eigen::Vector3d> vector_path_;
        ros::Publisher traj_pub_, vis_traj_pub_, traj_pose_pub_, traj_multi_pub_;

        // publish the pose
        Eigen::Vector3d traj_pose_;
        Eigen::Vector3d odom_pose_;
        Eigen::Vector3d pub_pose_ = Eigen::Vector3d::Zero();

        // path optimization steps
        int step_ = 5;
        
        // message
        path_planning::Trajectory traj_msg_;
        geometry_msgs::PoseStamped traj_pose_msg_;
        trajectory_msgs::MultiDOFJointTrajectory traj_multi_msg_;

        std::string traj_pose_topic_, traj_multi_topic_;

        // limitation of the optimization
        double limit_d_, limit_vel_, limit_acc_, limit_jerk_;
    public:
        PathOptimization() {};
        ~PathOptimization() {};
        PathOptimization(ros::NodeHandle& nh) : nh_(nh), prrt(nh) {};

        // path optimization
        Eigen::VectorXd time_allocation(const std::vector<Eigen::Vector3d>& path);
        Eigen::MatrixXd CoordGeneration(const std::vector<Eigen::Vector3d>& Path, Eigen::VectorXd &Time,
                                        const Eigen::Vector3d& start_vel, const Eigen::Vector3d& end_vel,
                                        const Eigen::Vector3d& start_acc, const Eigen::Vector3d& end_acc);
        Eigen::VectorXd OsqpEigenSolve(const Eigen::MatrixXd &Path, const Eigen::VectorXd &Time,
                                       const double &start_vel, const double &end_vel,
                                       const double &start_acc, const double &end_acc,
                                       bool &flag);

        // init the main function of the path optimization
        void init();
        void exec_path_optimization_callback(const ros::TimerEvent& event);

        // publish the message of path optimization
        void pub_traj_pose_msg(const Eigen::Vector3d& pose);
        void pub_traj_msg(const Eigen::MatrixXd& path, const Eigen::VectorXd& time);
        void pub_traj_multi_msg(const Eigen::MatrixXd& traj, const Eigen::VectorXd& time);

        // visualization
        void vis_traj(const Eigen::MatrixXd& traj);

        void get_nodehandle_namespace() {std::cout << "Namespace: " << nh_.getNamespace() << std::endl;};

        // RRT class
        PRRT prrt;
};

#endif