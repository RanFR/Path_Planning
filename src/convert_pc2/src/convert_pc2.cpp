#include "convert_pc2/convert_pc2.h"

void ConvertPC2::convert_pc2_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // convert the point cloud to a matrix
    // pcl_to_matrix(cloud);
    pcl_to_vector(cloud);

    // visualization of the expansion obstacles
    vis_expansion_obstacles(expansion_obstacles_buffer_);
}

void ConvertPC2::convert_pc_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    // convert the point cloud to a matrix
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (geometry_msgs::Point32 pt : msg->points) {
        pcl::PointXYZ tmp;
        tmp.x = pt.x;
        tmp.y = pt.y;
        tmp.z = pt.z;
        cloud.points.push_back(tmp);
    }

    pcl_to_vector(cloud);

    // visualization of the expansion obstacles
    vis_expansion_obstacles(expansion_obstacles_buffer_);
}

void ConvertPC2::pcl_to_matrix(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    // convert the point cloud to a matrix
    pts.clear();
    for (pcl::PointXYZ pt : cloud.points) {
        Eigen::Vector3d tmp;
        tmp << pt.x, pt.y, pt.z;
        pts.push_back(tmp);
    }
}

void ConvertPC2::pcl_to_vector(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    // convert the point cloud to a vector
    reset_obss_buffer();
    Eigen::Vector3d obs, obs_local;
    // loop every cloud point
    for (pcl::PointXYZ pt : cloud.points) {
        obs << pt.x, pt.y, pt.z;
        obs_local = obs - odom_pose_;
        // calu the point in the local update range
        if (std::abs(obs_local(0)) < local_update_range_(0) && std::abs(obs_local(1)) < local_update_range_(1) && std::abs(obs_local(2)) < local_update_range_(2)) {
            int index = pose_to_index(obs_local);
            obss_buffer_[index] = 1;
            // expansion the obstacle
            for (double val=-expansion_range_; val<=expansion_range_; val+=0.05) {
                for (double val1=-expansion_range_; val1<=expansion_range_; val1+=0.05) {
                    for (double val2=-expansion_range_; val2<=expansion_range_; val2+=0.05) {
                        Eigen::Vector3d tmp = obs_local + Eigen::Vector3d(val, val1, val2);
                        if (std::abs(tmp(0)) < local_update_range_(0) && std::abs(tmp(1)) < local_update_range_(1) && std::abs(tmp(2)) < local_update_range_(2)) {
                            int idx = pose_to_index(tmp);
                            obss_buffer_[idx] = 1;

                            // test function: save the all expansion obstacles
                            expansion_obstacles_buffer_.push_back(tmp);
                        }
                    }
                }
            }
        }
    }
    // std::cout << expansion_obstacles_buffer_.size() << std::endl;
}

int ConvertPC2::pose_to_index(const Eigen::Vector3d& pose) {
    // convert the pose to the index
    Eigen::Vector3d tmp = pose + local_update_range_;
    // make sure double to int is correct
    int idx_x = tmp(0) * 10.0 + 1e-3;
    int idx_y = tmp(1) * 10.0 + 1e-3;
    int idx_z = tmp(2) * 10.0 + 1e-3;
    int idx = 110*30*idx_x + 30*idx_y + idx_z;
    return idx;
}

void ConvertPC2::reset_obss_buffer() {
    // reset the obss buffer, fill every element with 0
    std::fill(obss_buffer_.begin(), obss_buffer_.end(), 0);

    // clear the expansion obstacles buffer
    expansion_obstacles_buffer_.clear();
}

void ConvertPC2::init_obss_buffer()
{
    // init the obss buffer
    int size_x = 2.0*local_update_range_(0)*10.0 + 0.001;
    int size_y = 2.0*local_update_range_(1)*10.0 + 0.001;
    int size_z = 2.0*local_update_range_(2)*10.0 + 0.001;
    obss_buffer_.resize(size_x*size_y*size_z);
    reset_obss_buffer();
}

void ConvertPC2::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // get the odom position
    odom_pose_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    // get the odom velocity
    odom_vel_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
}

void ConvertPC2::vis_expansion_obstacles(const std::vector<Eigen::Vector3d>& ex_obss)
{
    visualization_msgs::Marker points;
    int id = 0;
    points.header.frame_id    = "camera_init";
    points.header.stamp       = ros::Time::now();
    points.ns                 = "wp_path";
    points.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.pose.orientation.x = 0.0;
    points.pose.orientation.y = 0.0;
    points.pose.orientation.z = 0.0;

    points.id    = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 255.0 / 255.0;
    points.color.g = 192.0 / 255.0;
    points.color.b = 203.0 / 255.0;

    for (Eigen::Vector3d obs : ex_obss)
    {
        geometry_msgs::Point p;
        p.x = obs(0) + odom_pose_(0);
        p.y = obs(1) + odom_pose_(1);
        p.z = obs(2) + odom_pose_(2);

        points.points.push_back(p);
    }

    vis_expansion_obstacles_pub_.publish(points);
}

bool ConvertPC2::in_local_grid_map(const Eigen::Vector3d& pt) {
    // judge whether the point is in the grid map
    Eigen::Vector3d pt_local = pt - odom_pose_;
    if (std::abs(pt_local(0)) < local_update_range_(0) && std::abs(pt_local(1)) < local_update_range_(1) && std::abs(pt_local(2)) < local_update_range_(2)) {
        return true;
    }
    return false;
}

void ConvertPC2::init() {
    // topic
    nh_.param("pc2_topic", pc2_topic_, std::string("/cloud_registered"));
    nh_.param("plan_env/odom_topic", odom_topic_, std::string("/Odometry"));
    // plan_env parameters
    nh_.param("grid_map/local_update_range_x", local_update_range_(0), -1.0);
    nh_.param("grid_map/local_update_range_y", local_update_range_(1), -1.0);
    nh_.param("grid_map/local_update_range_z", local_update_range_(2), -1.0);
    // expansion parameter of plan_env
    nh_.param("grid_map/expansion_range", expansion_range_, -1.0);

    // init obss buffer
    init_obss_buffer();

    // convert pointcloud2 to XYZ vector
    convert_pc2_sub_ = nh_.subscribe(pc2_topic_, 1, &ConvertPC2::convert_pc_callback, this);

    // get odom message
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &ConvertPC2::odom_callback, this);

    // visulization of the expansion obstacles
    vis_expansion_obstacles_pub_ = nh_.advertise<visualization_msgs::Marker>("convert_pc2/vis_expansion_obstacles", 1);
}