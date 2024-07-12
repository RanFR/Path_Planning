#include "prrt/prrt.h"

double PRRT::generate_random(double m, double n, double d) {
    std::random_device rd;
    std::mt19937 gen(rd());
    if (m > n)
    {
        std::uniform_real_distribution<> dis(n-d, m+d);
        return dis(gen);
    }
    std::uniform_real_distribution<> dis(m-d, n+d);
    return dis(gen);
}

bool PRRT::hit_obstacle(const std::vector<int>& obss_buffer, const Eigen::Vector3d& node_new, const Eigen::Vector3d& node_near) {
    // loop every obstacle
    Eigen::Vector3d v = node_new - node_near;
    Eigen::Vector3d node_center;
    double distance;
    // std::cout << obss_buffer[convert_pc2.pose_to_index(node_near)] << std::endl;
    for (double j=0; j<11.0; ++j) {
        node_center = node_near + v * j / 10.0;
        int index = convert_pc2.pose_to_index(node_center);
        // if the index is out of the range of the obss_buffer, then continue and consider it as not hitting the obstacles
        if (index < 0 || index >= obss_buffer.size())
        {
            continue;
        }
        if (obss_buffer[index] == 1) {
            return true;
        }
    }
    return false;
}

Eigen::Vector3d PRRT::step_move(const Eigen::Vector3d &node_near, const Eigen::Vector3d &node_rand) {
    Eigen::Vector3d v = node_rand - node_near;
    double norm = v.norm();
    if (norm > step) {
        return node_near + v / norm * step;
    }
    return node_rand;
}

Eigen::Vector3d PRRT::get_nearest_node(const std::vector<Eigen::Vector3d> &nodes, const Eigen::Vector3d &node_rand) {
    double min_distance = 1e6;
    Eigen::Vector3d node_near;
    // loop every node to find the nearest node
    for (Eigen::Vector3d node : nodes) {
        double distance = (node - node_rand).norm();
        if (distance < min_distance) {
            min_distance = distance;
            node_near = node;
        }
    }
    return node_near;
}

void PRRT::find_path(const Eigen::Vector3d& node, const std::map<Eigen::Vector3d, Eigen::Vector3d, Vector3dLessThan>& parent) {
    // init the path
    path_.clear();
    Eigen::Vector3d current_node = node;
    // std::cout << "current node: " << current_node.transpose() << std::endl;

    // find the path(start searching from the end node to the start node)
    for (int iter=0; iter<parent.size(); ++iter)
    {
        path_.push_back(current_node);
        if ((current_node-start_node).norm() < step)
        {
            break;
        }
        current_node = parent.at(current_node);
    }

    // add the start node into the path vector
    path_.push_back(start_node);
    // reverse the path vector, because the origin path vector is from the end node to the start node
    std::reverse(path_.begin(), path_.end());
    // std::cout << "rrt path:" << std::endl;
    // for (Eigen::Vector3d pt : path_)
    // {
    //     std::cout << pt.transpose() << std::endl;
    // }

    // reduce the number of points in the rrt path
    reduced_path_.clear();
    reduce_rrt_points(path_);

    // // publish the path
    // rrt_path_msg_.header.stamp = ros::Time::now();
    // rrt_path_msg_.header.frame_id = "camera_init";
    // rrt_path_msg_.path.clear();
    // for (Eigen::Vector3d pt : path_)
    // {
    //     geometry_msgs::Point p;
    //     p.x = pt(0);
    //     p.y = pt(1);
    //     p.z = pt(2);
    //     rrt_path_msg_.path.push_back(p);
    // }
    // rrt_path_pub_.publish(rrt_path_msg_);

    // publish the rrt path visualization
    vis_rrt_path(path_);
    vis_reduced_rrt_path(reduced_path_);
}

void PRRT::reduce_rrt_points(const std::vector<Eigen::Vector3d>& path)
{
    // adjacent nodes not hit the obstacles by default
    int i = 0, j = 2;
    reduced_path_.push_back(path.front());
    while (j < path.size())
    {
        Eigen::Vector3d start = path[i], end = path[j];
        // between start and end point, if there is no obstacle, then plus j
        if (!hit_obstacle(convert_pc2.get_obss_buffer(), end-odom_pose_, start-odom_pose_))
        {
            ++j;
        }
        else
        {
            reduced_path_.push_back(path[j-1]);
            i = j - 1;
            j = i + 2;
        }
    }
    reduced_path_.push_back(path.back());
}

void PRRT::run(const ros::TimerEvent& event)
{
    // get the current point as the start node
    get_current_pt();
    // FSM
    change_node();

    // main function of RRT
    std::vector<Eigen::Vector3d> nodes;
    nodes.push_back(start_node);
    std::map<Eigen::Vector3d, Eigen::Vector3d, Vector3dLessThan> parent;
    bool flag = false;

    // odom pose to match the local grid map judgement
    // optimize the code lately
    odom_pose_ = convert_pc2.get_odom_pose();

    // if the distance between the start node and the end node is less than the step
    if ((start_node - end_node).norm() < step) {
        parent[end_node] = start_node;
        find_path(end_node, parent);
        flag = true;
    }
    else {
        for (int iter=0; iter<max_iter; ++iter) {
            // std::cout << "iter: " << iter << std::endl;
            Eigen::Vector3d node_rand;
            if (generate_random(0.0, 1.0, 0.0) < epsilon) {
                node_rand = end_node;
            } else {
                node_rand(0) = generate_random(start_node(0), end_node(0), 2.0);
                node_rand(1) = generate_random(start_node(1), end_node(1), 2.0);
                node_rand(2) = generate_random(start_node(2), end_node(2), 1.0);
            }

            // get the nearest node
            Eigen::Vector3d node_near = get_nearest_node(nodes, node_rand);

            // get the new node
            Eigen::Vector3d node_new = step_move(node_near, node_rand);
            
            // if the node_new is in the local grid map, the judge whether it hit the obstacles, otherwise, continue
            if (convert_pc2.in_local_grid_map(node_new-odom_pose_))
            {
                // if the node_new is in the local grid map, then judge whether it hit the obstacles
                // -end_node to make sure that the point is in the local grid map
                if (!(hit_obstacle(convert_pc2.get_obss_buffer(), node_new-odom_pose_, node_near-odom_pose_))) {
                    nodes.push_back(node_new);
                    parent[node_new] = node_near;
                }
                else {
                    continue;
                }
            }
            else
            {
                // if the node_new is not in the local grid map, then add the node_new into the nodes
                nodes.push_back(node_new);
                parent[node_new] = node_near;
            }

            // if the node_new is close to the end_node, then break
            double distance = (node_new - end_node).norm();
            if (distance < step) {
                // make sure node_new is not equals to end_node
                if (distance > 1e-3)
                {
                    parent[end_node] = node_new;
                    find_path(end_node, parent);
                }
                else
                {
                    // std::cout << "Enter." << std::endl;
                    find_path(node_new, parent);
                }
                flag = true;
                break;
            }
        }
    }

    // add the path between the end node and the node closest to the end node to make sure the path exists
    if (!flag) {
        Eigen::Vector3d node_near = get_nearest_node(nodes, end_node);
        parent[end_node] = node_near;
        find_path(end_node, parent);
        ROS_WARN("WARNING! The path was not successfully searched, intervention measures have been initiated.");
    }
}

void PRRT::change_node()
{
    // change the start node according to the end node
    if ((odom_pose_-end_node).norm() < 0.1)
    {   
        if (target_idx < targets_.size())
        {
            end_node = targets_[++target_idx];
        }
        else
        {
            // out of range, set the end node to the last target
            end_node = targets_.back();
        }
    }
    // std::cout << end_node.transpose() << std::endl;
}

void PRRT::get_current_pt()
{
    // get the current point
    start_node = convert_pc2.get_odom_pose();

    // publish the current point
    geometry_msgs::PointStamped pt;
    pt.header.stamp = ros::Time::now();
    pt.header.frame_id = "world";
    pt.point.x = start_node(0);
    pt.point.y = start_node(1);
    pt.point.z = start_node(2);
    current_pt_pub_.publish(pt);
}

void PRRT::vis_rrt_path(const std::vector<Eigen::Vector3d>& path) {
    visualization_msgs::Marker points;
    int id = 0;
    points.header.frame_id    = "world";
    points.header.stamp       = ros::Time::now();
    points.ns                 = "wp_path";
    points.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.pose.orientation.x = 0.0;
    points.pose.orientation.y = 0.0;
    points.pose.orientation.z = 0.0;

    points.id = id;

    points.type = visualization_msgs::Marker::SPHERE_LIST;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 1.0;
    points.color.g = 1.0;
    points.color.b = 0.0;

    for (Eigen::Vector3d pt : path)
    {
        geometry_msgs::Point p;
        p.x = pt(0);
        p.y = pt(1);
        p.z = pt(2);

        points.points.push_back(p);
    }

    vis_rrt_path_pub_.publish(points);
}

void PRRT::vis_reduced_rrt_path(const std::vector<Eigen::Vector3d>& path) {
    visualization_msgs::Marker points;
    int id = 0;
    points.header.frame_id    = "world";
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
    points.color.r = 1.0;
    points.color.g = 0.0;
    points.color.b = 1.0;

    for (Eigen::Vector3d pt : path)
    {
        geometry_msgs::Point p;
        p.x = pt(0);
        p.y = pt(1);
        p.z = pt(2);

        points.points.push_back(p);
    }

    vis_reduced_rrt_path_pub_.publish(points);
}

void PRRT::init_targets()
{
    std::fstream file;
    std::string prrt_path = ros::package::getPath("prrt");
    file.open(prrt_path + "/data/targets.txt", std::ios::in);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open the targets file!");
        return;
    }
    Eigen::Vector3d target;
    while (!file.eof())
    {
        file >> target(0) >> target(1) >> target(2);
        targets_.push_back(target);
    }
}

void PRRT::init() {
    // init targets
    init_targets();

    // set the (0, 0, 0) as the end node
    start_node << 0.0, 0.0, 1.0;
    end_node = targets_[0];

    // init convert_pc2
    convert_pc2.init();
    ROS_INFO("ConvertPC2 init success!");

    // rrt sample max iter
    nh_.param<int>("rrt/max_iter", max_iter, 1000);

    // execute the prrt algorithm on a regular basis
    prrt_timer_ = nh_.createTimer(ros::Duration(0.1), &PRRT::run, this);

    current_pt_pub_ = nh_.advertise<geometry_msgs::PointStamped>("prrt/current_node", 1);

    // advertise the rrt path visualization
    vis_rrt_path_pub_ = nh_.advertise<visualization_msgs::Marker>("prrt/vis_rrt_path", 1);
    vis_reduced_rrt_path_pub_ = nh_.advertise<visualization_msgs::Marker>("prrt/vis_reduced_rrt_path", 1);
}