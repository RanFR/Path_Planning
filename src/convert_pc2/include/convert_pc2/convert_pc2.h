#ifndef _CONVERT_PC2_H_
#define _CONVERT_PC2_H_

#include <vector>
#include <string>
#include <cmath>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

class ConvertPC2
{
    private:
        // save the point cloud as a matrix
        std::vector<Eigen::Vector3d> pts;
        ros::NodeHandle nh_;
        ros::Subscriber convert_pc2_sub_, odom_sub_;
        ros::Publisher vis_expansion_obstacles_pub_;
        std::string pc2_topic_, odom_topic_;
        std::vector<int> obss_buffer_;
        std::vector<Eigen::Vector3d> expansion_obstacles_buffer_;
        Eigen::Vector3d local_update_range_;
        Eigen::Vector3d odom_pose_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d odom_vel_ = Eigen::Vector3d::Zero();
        double expansion_range_;
    public:
        ConvertPC2() {};
        ~ConvertPC2() {};
        ConvertPC2(ros::NodeHandle& nh) : nh_(nh) {};
        void init();
        // convert the point cloud to XYZ matrix
        void convert_pc2_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void convert_pc_callback(const sensor_msgs::PointCloud::ConstPtr& msg);
        void pcl_to_matrix(const pcl::PointCloud<pcl::PointXYZ> &cloud);
        void pcl_to_vector(const pcl::PointCloud<pcl::PointXYZ> &cloud);
        std::vector<Eigen::Vector3d> get_pts() {return pts;};
        int pose_to_index(const Eigen::Vector3d& pose);

        // return class members
        std::vector<int> get_obss_buffer() {return obss_buffer_;};
        Eigen::Vector3d get_local_update_range() {return local_update_range_;};
        Eigen::Vector3d get_odom_pose() {return odom_pose_;};
        Eigen::Vector3d get_odom_velocity() {return odom_vel_;};

        // the function related to the vector
        void reset_obss_buffer();
        void init_obss_buffer();

        // get odom topic
        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

        // judge whether the point is in the local grid map
        bool in_local_grid_map(const Eigen::Vector3d& pt);

        // test functions
        void get_nodehandle_namespace() {std::cout << "Namespace: " << nh_.getNamespace() << std::endl;};
        void vis_expansion_obstacles(const std::vector<Eigen::Vector3d>& ex_obss);
        
};

#endif