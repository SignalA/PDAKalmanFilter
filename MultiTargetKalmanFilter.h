#ifndef MULTI_TARGET_KALMAN_FILTER_H  // 判断是否已经包含了头文件，防止重复包含
#define MULTI_TARGET_KALMAN_FILTER_H  // 如果没有定义，则定义它

// 包含所需的头文件
#include "KalmanFilterCustom.h"
#include "HungarianAlgorithm.h"
#include "DataReader.h"

#include <vector>

// 定义一个用于多目标卡尔曼滤波的类
class MultiTargetKalmanFilter {
public:
    // 带参数的构造函数，接收目标数、状态维度和测量维度作为参数
    MultiTargetKalmanFilter(int num_targets, int state_dim, int measurement_dim);

    // 预测函数，执行卡尔曼滤波的预测步骤
    // 可以传入初始测量值、转移矩阵和过程噪声，如果没有传入，则使用默认值（空向量或空矩阵）
    void predict(const std::vector<Eigen::VectorXd>& initial_measurements = std::vector<Eigen::VectorXd>(), 
                 const Eigen::MatrixXd& transition_matrix = Eigen::MatrixXd(),
                 const Eigen::MatrixXd& process_noise = Eigen::MatrixXd());

    // 更新函数，执行卡尔曼滤波的更新步骤
    // 接收测量值、测量矩阵和测量噪声作为参数
    void update(const std::vector<Eigen::VectorXd>& measurements, 
                const Eigen::MatrixXd& measurement_matrix,
                const Eigen::MatrixXd& measurement_noise);

    // 获取所有目标的当前状态的函数
    std::vector<Eigen::VectorXd> getStates() const;

    // 获取所有目标的当前协方差的函数
    std::vector<Eigen::MatrixXd> getCovariances() const;

private:
    // 目标的数量
    int num_targets_;

    // 为每个目标存储一个卡尔曼滤波器的向量
    std::vector<KalmanFilterCustom> filters_;
};

#endif  // 结束 #ifndef 语句
