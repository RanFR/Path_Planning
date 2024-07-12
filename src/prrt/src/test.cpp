#include <vector>
#include <map>
#include <Eigen/Dense>

std::vector<Eigen::MatrixXd> find_path(Eigen::MatrixXd node, std::map<std::vector<double>, Eigen::MatrixXd> &parent) {
    std::vector<Eigen::MatrixXd> path;
    Eigen::MatrixXd start_node; // 需要定义start_node的值

    while (!(node == start_node).all()) { // 使用Eigen库的all()函数来检查两个矩阵是否相等
        path.push_back(node);
        node = parent[std::vector<double>(node.data(), node.data() + node.size())];
    }

    std::reverse(path.begin(), path.end()); // 反转路径，使其从根节点开始

    return path;
}