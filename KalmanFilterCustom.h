#pragma once
#include <Eigen/Dense>
#include <vector>

// 定义一个自定义的卡尔曼滤波器类
class KalmanFilterCustom {
public:
    // 默认构造函数，设置状态和测量的维度为2
    KalmanFilterCustom();

    // 带参数的构造函数，接收状态和测量的维度作为参数
    KalmanFilterCustom(int state_dim, int measurement_dim);

    // 预测函数，执行卡尔曼滤波的预测步骤
    // 可以传入初始测量值、转移矩阵和过程噪声，如果没有传入，则使用默认值（空向量或空矩阵）
    void predict(const Eigen::VectorXd& initial_measurement = Eigen::VectorXd(), 
             const Eigen::MatrixXd& transition_matrix = Eigen::MatrixXd(),
             const Eigen::MatrixXd& process_noise = Eigen::MatrixXd());

    // 更新函数，执行卡尔曼滤波的更新步骤
    // 接收测量值、测量矩阵、测量噪声和概率作为参数
    void update(const Eigen::VectorXd& measurement, 
                const Eigen::MatrixXd& measurement_matrix,
                const Eigen::MatrixXd& measurement_noise,
                double probability);

    // 获取当前状态的函数
    Eigen::VectorXd getState() const;

    // 获取当前协方差的函数
    Eigen::MatrixXd getCovariance() const;

    // 获取是否为第一次更新的标记的函数
    bool is_first_update() const;

private:
    // 状态的维度
    int state_dim_;

    // 测量的维度
    int measurement_dim_;

    // 当前状态
    Eigen::VectorXd state_;

    // 当前协方差
    Eigen::MatrixXd covariance_;

    // 转移矩阵
    Eigen::MatrixXd transition_matrix_;

    // 过程噪声矩阵
    Eigen::MatrixXd process_noise_;

    // 是否为第一次更新的标记
    bool is_first_update_;
};
