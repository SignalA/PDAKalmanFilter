#include <Eigen/Dense>  // 引入Eigen库，用于处理矩阵和向量
#include <fstream>  // 引入fstream库，用于文件读写
#include <vector>  // 引入vector库，用于处理动态数组
#include <sstream>  // 引入sstream库，用于字符串流的处理
#include <iostream>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <map>  // 引入map库，用于存储每个目标的卡尔曼滤波器
#include <cmath>

#include <limits>

#include "MultiTargetKalmanFilter.h"
//"C:\Program Files\mingw64\bin\g++.exe" -fdiagnostics-color=always -g D:\work\PDAKalmanFilter\mainc.cpp -I D:\pkg\eigen-3.4.0 -I D:\work\PDAKalmanFilter -o D:\work\PDAKalmanFilter\mainc.exe


int main() {
    // 创建DataReader对象
    DataReader reader("target_actual_positions.txt", "output.txt");
    int n = 5;  // 这里我们只处理前4个目标
    // 读取数据
    auto data = reader.readData(n);

    // 定义滤波器和预测器的维度
    int state_dim = 2;  // 状态向量的维度为2，对应位置的x和y坐标
    int measurement_dim = 2;  // 测量向量的维度也为2，对应位置的x和y坐标

    // 创建一个MultiTargetKalmanFilter对象
    MultiTargetKalmanFilter mt_filter(n, state_dim, measurement_dim);

    // 创建一个容器用于存储预测的位置
    std::vector<std::vector<Eigen::VectorXd>> predicted_positions_all;

    // 对于每个时间段的每个目标，进行滤波和预测
    for (auto& group : data) {
        std::vector<Eigen::VectorXd> initial_measurements;
        for (auto& target : group) {
            initial_measurements.push_back(target.position);
        }

        // 预测所有目标的位置
        mt_filter.predict(initial_measurements);

        // 存储预测的位置
        auto predicted_positions = mt_filter.getStates();
        predicted_positions_all.push_back(predicted_positions);

        // 根据每个目标的测量值来更新滤波器的状态
        std::vector<Eigen::VectorXd> measurements;
        for (auto& target : group) {
            measurements.push_back(target.position);
        }

        Eigen::MatrixXd measurement_matrix = Eigen::MatrixXd::Identity(state_dim, state_dim);
        Eigen::MatrixXd measurement_noise = Eigen::MatrixXd::Identity(state_dim, state_dim);

        mt_filter.update(measurements, measurement_matrix, measurement_noise);

        // 存储滤波后的位置
        auto filtered_positions = mt_filter.getStates();
        for (size_t i = 0; i < group.size(); ++i) {
            group[i].filtered_position = filtered_positions[i];
        }
    }

    // 将预测的位置按照滤波值之后的顺序写入到data中
    for (size_t i = 0; i < data.size(); ++i) {
        auto& group = data[i];
        auto& predicted_positions = predicted_positions_all[i];
        for (size_t j = 0; j < group.size(); ++j) {
            group[j].predicted_position = predicted_positions[j];
        }
    }

    // 将处理后的数据写入到输出文件中
    reader.writeData(data);

    return 0;
}




