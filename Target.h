#ifndef TARGET_H
#define TARGET_H
#include <Eigen/Dense>

// 定义一个结构体Target，用于存储一条目标的数据
struct Target {
    int year, month, day, hour, minute, second;  // 年、月、日、小时、分钟、秒
    int id;  // 目标的ID
    Eigen::Vector2d position;  // 目标的实际位置，这是一个二维向量，使用Eigen库的Vector2d类型
    Eigen::Vector2d filtered_position;  // 目标的滤波位置
    Eigen::Vector2d predicted_position;  // 目标的预测位置
};
#endif  // 结束 #ifndef 语句