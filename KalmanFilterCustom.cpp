#include "KalmanFilterCustom.h"

// 默认构造函数，初始化状态和测量维度为2，设置转移矩阵和过程噪声为单位矩阵，标记为第一次更新
KalmanFilterCustom::KalmanFilterCustom() :
    state_dim_(2), measurement_dim_(2),
    transition_matrix_(Eigen::MatrixXd::Identity(state_dim_, state_dim_)),
    process_noise_(Eigen::MatrixXd::Identity(state_dim_, state_dim_)),
    is_first_update_(true)
{
    state_ = Eigen::VectorXd::Zero(state_dim_);  // 初始化状态向量为0
    covariance_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);  // 初始化协方差为单位矩阵
}

// 构造函数，接受状态和测量维度作为参数，设置转移矩阵和过程噪声为单位矩阵，标记为第一次更新
KalmanFilterCustom::KalmanFilterCustom(int state_dim, int measurement_dim) :
    state_dim_(state_dim), measurement_dim_(measurement_dim),
    transition_matrix_(Eigen::MatrixXd::Identity(state_dim, state_dim)),
    process_noise_(Eigen::MatrixXd::Identity(state_dim, state_dim)),
    is_first_update_(true)
{
    state_ = Eigen::VectorXd::Zero(state_dim);  // 初始化状态向量为0
    covariance_ = Eigen::MatrixXd::Identity(state_dim, state_dim);  // 初始化协方差为单位矩阵
}

// 预测函数，用于进行卡尔曼滤波的预测步骤
void KalmanFilterCustom::predict(const Eigen::VectorXd& initial_measurement,
    const Eigen::MatrixXd& transition_matrix,
    const Eigen::MatrixXd& process_noise) {
    Eigen::MatrixXd transition_matrix_local = transition_matrix;
    Eigen::MatrixXd process_noise_local = process_noise;

    // 若未提供过程噪声和转移矩阵，使用类内部定义的变量
    if (transition_matrix_local.size() == 0) {
        transition_matrix_local = transition_matrix_;
    }
    if (process_noise_local.size() == 0) {
        process_noise_local = process_noise_;
    }

    // 若是第一次更新，将状态设置为初始测量值，否则使用状态转移矩阵进行预测
    if (is_first_update_ && initial_measurement.size() != 0) {
        state_ = initial_measurement;
        is_first_update_ = false;
    }
    else {
        state_ = transition_matrix_local * state_;  // 预测状态方程
    }

    // 预测协方差方程
    covariance_ = transition_matrix_local * covariance_ * transition_matrix_local.transpose() + process_noise_local;
}

// 更新函数，用于进行卡尔曼滤波的更新步骤
void KalmanFilterCustom::update(const Eigen::VectorXd& measurement,
    const Eigen::MatrixXd& measurement_matrix,
    const Eigen::MatrixXd& measurement_noise,
    double probability) {
    // 若是第一次更新，将状态设置为测量值，否则执行更新步骤
    if (is_first_update_) {
        state_ = measurement;
        is_first_update_ = false;
    }
    else {
        // 计算卡尔曼增益
        Eigen::MatrixXd S = measurement_matrix * covariance_ * measurement_matrix.transpose() + measurement_noise;
        Eigen::MatrixXd K = covariance_ * measurement_matrix.transpose() * S.inverse();
        
        // 更新状态方程
        state_ = state_ + K * (measurement - measurement_matrix * state_);
        
        // 更新协方差方程
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
        covariance_ = (I - K * measurement_matrix) * covariance_;
    }
}

// 获取当前的状态向量
Eigen::VectorXd KalmanFilterCustom::getState() const {
    return state_;
}

// 获取当前的协方差矩阵
Eigen::MatrixXd KalmanFilterCustom::getCovariance() const {
    return covariance_;
}

// 返回是否为第一次更新的标记
bool KalmanFilterCustom::is_first_update() const {
    return is_first_update_;
}
