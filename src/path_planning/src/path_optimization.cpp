#include "path_planning/path_optimization.h"

Eigen::VectorXd PathOptimization::time_allocation(const std::vector<Eigen::Vector3d>& path) {
    Eigen::VectorXd time(path.size() - 1);
    int idx = path.size() - 1;
    double tmp_x, tmp_y, tmp_z;
    tmp_x = std::abs(path[0](0) - path[1](0));
    tmp_y = std::abs(path[0](1) - path[1](1));
    tmp_z = std::abs(path[0](2) - path[1](2));
    time(0) = 2.0 * std::max(tmp_x, std::max(tmp_y, tmp_z));
    for (int i=1; i<idx-1; ++i) {
        tmp_x = std::abs(path[i](0) - path[i+1](0));
        tmp_y = std::abs(path[i](1) - path[i+1](1));
        tmp_z = std::abs(path[i](2) - path[i+1](2));
        time(i) = std::max(tmp_x, std::max(tmp_y, tmp_z));
    }
    tmp_x = std::abs(path[idx-1](0) - path[idx](0));
    tmp_y = std::abs(path[idx-1](1) - path[idx](1));
    tmp_z = std::abs(path[idx-1](2) - path[idx](2));
    time(idx-1) = 2.0 * std::max(tmp_x, std::max(tmp_y, tmp_z));
    return time;
}

Eigen::MatrixXd PathOptimization::CoordGeneration(const std::vector<Eigen::Vector3d>& path, Eigen::VectorXd &Time,
                                                  const Eigen::Vector3d& start_vel, const Eigen::Vector3d& end_vel,
                                                  const Eigen::Vector3d& start_acc, const Eigen::Vector3d& end_acc)
{
    matrix_path_.resize(path.size(), 3);
    for (int i=0; i<path.size(); ++i) {
        matrix_path_.row(i) = path[i].transpose();
    }
    bool flag_x, flag_y, flag_z;
    flag_x = flag_y = flag_z = false;
    Eigen::VectorXd coord_x, coord_y, coord_z;
    Eigen::VectorXd time = Time;
    int failed_time = 0;
    while (!(flag_x && flag_y && flag_z)) {
        coord_x = OsqpEigenSolve(matrix_path_.col(0), time, start_vel[0], end_vel[0], start_acc[0], end_acc[0], flag_x);
        coord_y = OsqpEigenSolve(matrix_path_.col(1), time, start_vel[1], end_vel[1], start_acc[1], end_acc[1], flag_y);
        coord_z = OsqpEigenSolve(matrix_path_.col(2), time, start_vel[2], end_vel[2], start_acc[2], end_acc[2], flag_z);
        if (!(flag_x && flag_y && flag_z))
        {
            ++failed_time;
            // ROS_INFO("Failed to solve the problem, increase the time allocation and try again.");
            time *= 2.0;
        }
        if (failed_time > 3) {
            ROS_WARN("Failed to solve the problem, please check the parameters.");
            break;
        }
    }
    // get the optimal time
    Time = time;
    // Path points
    Eigen::MatrixXd coord(coord_x.size(), 3);
    coord << coord_x, coord_y, coord_z;
    
    return coord;
}

Eigen::VectorXd PathOptimization::OsqpEigenSolve(const Eigen::MatrixXd &Path, const Eigen::VectorXd &Time, 
                                                 const double& start_vel, const double& end_vel,
                                                 const double& start_acc, const double& end_acc,
                                                 bool &flag)
{
    flag = true;
    int m = Time.size();
    // int steps = m * 10.0 + 1e-3;
    int steps = 100;
    int len = 3 * m * (steps+1);
    Eigen::MatrixXd Aul = Eigen::MatrixXd::Identity(len-3, len);

    // first segment
    double dt = Time(0) / double(steps);
    for (int i=0; i<steps; ++i) {
        Aul(3*i, 3*i+1) = dt; 
        Aul(3*i+1, 3*i+2) = dt;
        Aul(3*i, 3*i+3) = -1.0;
        Aul(3*i+1, 3*i+4) = -1.0;
        Aul(3*i+2, 3*i+5) = -1.0;
    }
    // other segments
    for (int seg=1; seg<m; ++seg) {
        dt = Time(seg) / double(steps);
        int idx = 3 * (steps+1) * seg - 3;
        Aul(idx, idx+3) = -1.0;
        Aul(idx+1, idx+4) = -1.0;
        Aul(idx+2, idx+5) = -1.0;
        for (int i=0; i<steps; ++i) {
            Aul(idx+3+3*i, idx+3+3*i+1) = dt;
            Aul(idx+3+3*i+1, idx+3+3*i+2) = dt;
            Aul(idx+3+3*i, idx+3+3*i+3) = -1.0;
            Aul(idx+3+3*i+1, idx+3+3*i+4) = -1.0;
            Aul(idx+3+3*i+2, idx+3+3*i+5) = -1.0;
        }
    }

    // the up left part of matrix Ac
    Eigen::MatrixXd Aur = Eigen::MatrixXd::Zero(len-3, m*steps);
    // first segment
    dt = Time(0) / double(steps);
    for (int i=0; i<steps; ++i) {
        Aur(3*i+2, i) = dt;
    }
    // other segments
    for (int seg=1; seg<m; ++seg) {
        dt = Time(seg) / double(steps);
        // calculate the first index of every segment
        int idx = 3*steps + 3*(steps+1)*(seg-1);
        int idy = seg * steps;
        for (int i=0; i<steps; ++i) {
            Aur(idx+3*i+5, idy+i) = dt;
        }
    }

    // state and control input
    len = 4*m*steps + 3*m;
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(len, len);
    Eigen::MatrixXd Ac(Aul.rows()+len, len);
    Ac << Aul, Aur, Ad;
    Eigen::SparseMatrix<double> Acs = Ac.sparseView();

    // compute l and u
    Eigen::VectorXd lu0 = Eigen::VectorXd::Zero(3*(m*steps+m-1));
    Eigen::VectorXd ldx_state = Eigen::VectorXd::Ones(3*m*(steps+1)) * -limit_acc_;
    // every segment
    for (int i=0; i<m; ++i) {
        // every step
        int idx = 3 * i * (steps+1);
        ldx_state(idx) = Path(i, 0);
        ldx_state[idx+1] = -limit_vel_;
        for (int j=1; j<steps; ++j) {
            ldx_state(idx+3*j) = Path(i, 0) + double(j)/double(steps)*(Path(i+1, 0)-Path(i, 0)) - limit_d_;
            ldx_state(idx+3*j+1) = -limit_vel_;
        }
        ldx_state(idx+3*steps) = Path(i+1, 0);
        ldx_state[idx+3*steps+1] = -limit_vel_;
    }
    // first segment and last segment
    ldx_state[1] = start_vel;
    ldx_state[2] = start_acc;
    ldx_state[3*m*(steps+1)-1] = end_acc;
    ldx_state[3*m*(steps+1)-2] = end_vel;
    Eigen::VectorXd ldx_control = Eigen::VectorXd::Ones(m*steps) * -limit_jerk_;
    Eigen::VectorXd lux_state = Eigen::VectorXd::Ones(3*m*(steps+1)) * limit_acc_;
    // every segment
    for (int i=0; i<m; ++i) {
        // every step
        int idx = 3 * i * (steps+1);
        lux_state(idx) = Path(i, 0);
        lux_state[idx+1] = limit_vel_;
        for (int j=1; j<steps; ++j) {
            lux_state(idx+3*j) = Path(i, 0) + double(j)/double(steps)*(Path(i+1, 0)-Path(i, 0)) + limit_d_;
            lux_state(idx+3*j+1) = limit_vel_;
        }
        lux_state(idx+3*steps) = Path(i+1, 0);
        lux_state[idx+3*steps+1] = limit_vel_;
    }
    // first segment and last segment
    lux_state[1] = start_vel;
    lux_state[2] = start_acc;
    lux_state[3*m*(steps+1)-1] = end_acc;
    lux_state[3*m*(steps+1)-2] = end_vel;
    // control upper vector
    Eigen::VectorXd lux_control = Eigen::VectorXd::Ones(m*steps) * limit_jerk_;
    len = lu0.rows()+ldx_state.rows()+ldx_control.rows();
    Eigen::VectorXd lx(len), ux(len);
    lx << lu0, ldx_state, ldx_control;
    ux << lu0, lux_state, lux_control;
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(4*m*steps+3*m, 4*m*steps+3*m);
    P.block(3*m*(steps+1), 3*m*(steps+1), m*steps, m*steps) = Eigen::MatrixXd::Identity(m*steps, m*steps);
    Eigen::SparseMatrix<double> Ps = P.sparseView();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(4*m*steps+3*m);

    // OsqpEigen solver
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(4*m*steps+3*m);
    solver.data()->setNumberOfConstraints(7*m*steps+6*m-3);
    if (!solver.data()->setHessianMatrix(Ps))
    {
        ROS_WARN("Failed to set hessian matrix.");
    }
    if (!solver.data()->setGradient(q))
    {
        ROS_WARN("Failed to set gradient.");
    }
    if (!solver.data()->setLinearConstraintsMatrix(Acs))
    {
        ROS_WARN("Failed to set linear constraints matrix.");
    }
    if (!solver.data()->setLowerBound(lx))
    {
        ROS_WARN("Failed to set lower bound.");
    }
    if (!solver.data()->setUpperBound(ux))
    {
        ROS_WARN("Failed to set upper bound.");
    }

    if (!solver.initSolver())
    {
        ROS_WARN("Failed to initialize the solver.");
    }
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
        ROS_WARN("Failed to solve the problem.");
    }
    if (solver.getStatus() != OsqpEigen::Status::Solved)
    {
        // ROS_WARN("The solver did not find an optimal solution.");
        flag = false;
    }

    Eigen::VectorXd solution = solver.getSolution();
    len = m * (steps+1);
    Eigen::VectorXd coord(len);
    for (int i=0; i<len; ++i) {
        coord[i] = solution(3*i);
    }

    return coord;
}

void PathOptimization::vis_traj(const Eigen::MatrixXd& traj) {
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "camera_init";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;
    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < traj.rows(); i++){
      geometry_msgs::Point p;
      p.x = traj(i, 0);
      p.y = traj(i, 1); 
      p.z = traj(i, 2); 

      points.points.push_back(p);

      if( i < (traj.rows() - 1) )
      {
          geometry_msgs::Point p_line;
          p_line = p;
          line_list.points.push_back(p_line);
          p_line.x = traj(i+1, 0);
          p_line.y = traj(i+1, 1); 
          p_line.z = traj(i+1, 2);
          line_list.points.push_back(p_line);
      }
    }

    vis_traj_pub_.publish(points);
    vis_traj_pub_.publish(line_list);
}

void PathOptimization::exec_path_optimization_callback(const ros::TimerEvent& event) {
    // get the current position
    odom_pose_ = prrt.convert_pc2.get_odom_pose();
    // clear the vector path
    vector_path_.clear();
    // get the reduced rrt path
    vector_path_ = prrt.get_reduced_path();
    // get the start velocity and other information
    // Eigen::Vector3d start_vel = prrt.convert_pc2.get_odom_velocity();
    Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d end_acc = Eigen::Vector3d::Zero();

    // test function
    // std::cout << prrt.convert_pc2.get_odom_velocity().transpose() << std::endl;
    // std::cout << "vector_path_ last: " << vector_path_[vector_path_.size()-1].transpose() << std::endl;
    // std::cout << "vector path size: " << vector_path_.size() << std::endl;
    // std::cout << "Vector path:" << std::endl;
    // for (Eigen::Vector3d pt : vector_path_) {
    //     std::cout << pt.transpose() << std::endl;
    // }

    // make sure the path is existed, otherwise wait until the path is existed
    if (vector_path_.size() > 1) {
        // time allocation
        Eigen::VectorXd time = time_allocation(vector_path_);
        // // get the optimized trajectory
        traj_ = CoordGeneration(vector_path_, time, start_vel, end_vel, start_acc, end_acc);
        // std::cout << "traj_ start, row: " << traj_.rows() << std::endl << traj_.row(traj_.rows()-1) << std::endl;
        // publish the traj msg
        traj_pub_.publish(traj_msg_);

        // // first pub (at the original pose)
        // if (pub_pose_.norm() < 0.1) {
        //     pub_pose_ = traj_.row(5);
        //     pub_traj_pose_msg(pub_pose_);
        // }
        // // if the odom pose reach the last pub pose, then publish the next pose
        // if ((odom_pose_-pub_pose_).norm() < 0.1)
        // {
        //     // row 0 is the start node(odom node), so choose the row 1
        //     // std::cout << pub_pose_.transpose() << std::endl;
        //     std::cout << "Before: " << pub_pose_.transpose() << std::endl;
        //     pub_pose_ = traj_.row(5);
        //     std::cout << "After: " << pub_pose_.transpose() << std::endl;
        //     pub_traj_pose_msg(pub_pose_);
        // }
        // std::cout << time.transpose() << std::endl;
        pub_traj_multi_msg(traj_, time);

        // publish the trajectory

        // visualization
        vis_traj(traj_);
    }
    else {
        ROS_WARN("The path is not existed, please wait until the path is existed.");
    }
}

void PathOptimization::pub_traj_multi_msg(const Eigen::MatrixXd& traj, const Eigen::VectorXd& time) {
    // std::cout << "traj start, length: " << traj.rows() << std::endl << traj << std::endl;
    int traj_size = traj.rows();
    Eigen::VectorXd t(traj_size);
    for (int i=0; i<traj_size; ++i) {
        t[i] = time[0] / double(traj_size) * (i + 1.0);
    }
    traj_multi_msg_.header.stamp = ros::Time::now();
    traj_multi_msg_.header.frame_id = "world";
    traj_multi_msg_.points.clear();
    for (int i=0; i<traj.rows(); ++i) {
        geometry_msgs::Transform transform;
        transform.translation.x = traj(i, 0);
        transform.translation.y = traj(i, 1);
        // transform.translation.z = traj(i, 2);
        transform.translation.z = 1.0;
        trajectory_msgs::MultiDOFJointTrajectoryPoint point;
        point.transforms.push_back(transform);
        // point.time_from_start = ros::Duration(t[i-1]);

        traj_multi_msg_.points.push_back(point);
        traj_multi_msg_.joint_names.push_back(std::string("Point" + std::to_string(i)));
    }
    // traj_multi_pub_.publish(traj_multi_msg_);
}

void PathOptimization::pub_traj_pose_msg(const Eigen::Vector3d& pose)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = pose(0);
    pose_msg.pose.position.y = pose(1);
    pose_msg.pose.position.z = pose(2);
    traj_pose_pub_.publish(pose_msg);
}

void PathOptimization::pub_traj_msg(const Eigen::MatrixXd& path, const Eigen::VectorXd& time) {
    int path_len = path.rows();    
    traj_msg_.header.stamp = ros::Time::now();
    traj_msg_.header.frame_id = "world";
    
    Eigen::VectorXd time_each_point(path_len);
    // time
    double time_sum = time.sum();
    for (int i=0; i<path_len; ++i) {
        time_each_point(i) = time_sum / double(path_len) * double(i);
        traj_msg_.time.push_back(time_each_point(i));
        traj_msg_.yaw.push_back(0.0);
    }
    geometry_msgs::Point p;
    for (int i=0; i<path_len; ++i) {
        p.x = path(i, 0);
        p.y = path(i, 1);
        p.z = path(i, 2);
        traj_msg_.pos.push_back(p);
    }

    // _traj_pub.publish(traj_msg_);
}

void PathOptimization::init() {
    // get_nodehandle_namespace();
    // init prrt class
    prrt.init();
    ROS_INFO("PRRT init success!");

    // TODO: add the limitation of vel, acc, jerk
    nh_.param("path_optimization/limit_d", limit_d_, 0.1);
    nh_.param("path_optimization/limit_vel", limit_vel_, 1.0);
    nh_.param("path_optimization/limit_acc", limit_acc_, 1.0);
    nh_.param("path_optimization/limit_jerk", limit_jerk_, 1.0);

    // publish topic
    nh_.param<std::string>("path_optimization/traj_pose_topic", traj_pose_topic_, std::string("path_optimization/traj_pose"));
    nh_.param<std::string>("path_optimization/traj_multi_topic", traj_multi_topic_, "path_optimization/traj_multi_msg");

    // exec path optimization
    path_optimization_timer_ = nh_.createTimer(ros::Duration(0.05), &PathOptimization::exec_path_optimization_callback, this);

    // publish the msg of the single point of the trajectory
    traj_pub_ = nh_.advertise<path_planning::Trajectory>("path_optimization/traj_msg", 1);
    // publish the msg of the trajectory
    traj_multi_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(traj_multi_topic_, 1);

    traj_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(traj_pose_topic_, 1);

    // visualization
    vis_traj_pub_ = nh_.advertise<visualization_msgs::Marker>("path_optimization/vis_traj", 1);
}