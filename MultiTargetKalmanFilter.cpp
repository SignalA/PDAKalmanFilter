#include "MultiTargetKalmanFilter.h"

// 构造函数，初始化每个目标的卡尔曼滤波器
MultiTargetKalmanFilter::MultiTargetKalmanFilter(int num_targets, int state_dim, int measurement_dim) 
    : num_targets_(num_targets)
{
    // 为每个目标初始化一个卡尔曼滤波器
    for(int i=0; i<num_targets_; i++) {
        filters_.push_back(KalmanFilterCustom(state_dim, measurement_dim));
    }
}

// 预测函数，每个滤波器执行其预测步骤
void MultiTargetKalmanFilter::predict(const std::vector<Eigen::VectorXd>& initial_measurements, 
             const Eigen::MatrixXd& transition_matrix,
             const Eigen::MatrixXd& process_noise) {
    for(int i=0; i<num_targets_; i++) {
        filters_[i].predict(
            initial_measurements.size() > i ? initial_measurements[i] : Eigen::VectorXd(),
            transition_matrix, 
            process_noise
        );
    }
}

// 更新函数，计算匈牙利算法的成本矩阵，并用分配的测量更新每个滤波器
void MultiTargetKalmanFilter::update(const std::vector<Eigen::VectorXd>& measurements, 
            const Eigen::MatrixXd& measurement_matrix,
            const Eigen::MatrixXd& measurement_noise) {
    std::vector<std::vector<double>> cost_matrix(num_targets_, std::vector<double>(num_targets_));
    // 计算匈牙利算法的成本矩阵
    for (int i = 0; i < num_targets_; ++i) {
        for (int j = 0; j < num_targets_; ++j) {
            cost_matrix[i][j] = (filters_[i].getState() - measurements[j]).norm();
        }
    }

    // 运行匈牙利算法找到最优分配
    HungarianAlgorithm ha;
    std::vector<int> assignment;
    ha.Solve(cost_matrix, assignment);

    // 使用分配的测量更新每个滤波器
    for (int i = 0; i < num_targets_; ++i) {
        filters_[i].update(measurements[assignment[i]], measurement_matrix, measurement_noise, 1.0);
    }
}

// 获取所有目标的当前状态
std::vector<Eigen::VectorXd> MultiTargetKalmanFilter::getStates() const {
    std::vector<Eigen::VectorXd> states;
    for(const auto& filter : filters_) {
        states.push_back(filter.getState());
    }
    return states;
}
