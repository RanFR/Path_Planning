#ifndef _PRRT_H_
#define _PRRT_H_

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <fstream>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <ros/package.h>

#include "convert_pc2/convert_pc2.h"
#include "prrt/rrt_path.h"

struct Vector3dLessThan {
    bool operator() (const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const {
        if (v1(0) != v2(0)) {
            return v1(0) < v2(0);
        }
        if (v1(1) != v2(1)) {
            return v1(1) < v2(1);
        }
        return v1(2) < v2(2);
    }
};

// set the local grid map size
class PRRT {
    private:
        ros::NodeHandle nh_;
        // get pts according to the PRRT algorithms
        Eigen::MatrixXd pts;
        // the solved path from start to end
        std::vector<Eigen::Vector3d> path_;
        std::vector<Eigen::Vector3d> reduced_path_;
        std::vector<Eigen::Vector3d> targets_;
        double step = 0.5;
        // target index (global index)
        int target_idx = 0;
        Eigen::Vector3d start_node, end_node;
        // odom pose of the uav
        Eigen::Vector3d odom_pose_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        double epsilon = 0.3;
        int max_iter;

        ros::Subscriber current_node_sub_;
        ros::Timer prrt_timer_;
        ros::Publisher current_pt_pub_, rrt_path_pub_, vis_rrt_path_pub_, vis_reduced_rrt_path_pub_;
        prrt::rrt_path rrt_path_msg_;
    public:
        PRRT() {};
        ~PRRT() {};
        PRRT(ros::NodeHandle& nh) : nh_(nh), convert_pc2(nh_) {};

        // get random number
        double generate_random(double m, double n, double d);

        bool hit_obstacle(const std::vector<int>& obss_buffer, const Eigen::Vector3d& node_new, const Eigen::Vector3d& node_near);
        Eigen::Vector3d step_move(const Eigen::Vector3d &node_near, const Eigen::Vector3d &node_rand);
        Eigen::Vector3d get_nearest_node(const std::vector<Eigen::Vector3d> &nodes, const Eigen::Vector3d &node_rand);
        void find_path(const Eigen::Vector3d& node, const std::map<Eigen::Vector3d, Eigen::Vector3d, Vector3dLessThan> &parent);
        // get the current point as the start node
        void get_current_pt();

        void init();
        void run(const ros::TimerEvent& event);

        // reduced the number of points about the rrt path
        void reduce_rrt_points(const std::vector<Eigen::Vector3d>& path);

        // visualization rrt path
        void vis_rrt_path(const std::vector<Eigen::Vector3d>& path);
        void vis_reduced_rrt_path(const std::vector<Eigen::Vector3d>& path);

        void init_targets();

        // FSM
        void change_node();

        // functions related to getting the members of the class
        std::vector<Eigen::Vector3d> get_path() {return path_;};
        std::vector<Eigen::Vector3d> get_reduced_path() {return reduced_path_;};

        // get NodeHandle Namespace
        void get_nodehandle_namespace() {std::cout << "Namespace: " << nh_.getNamespace() << std::endl;};

        ConvertPC2 convert_pc2;
};

#endif