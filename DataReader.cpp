// 引入DataReader.h头文件
#include "DataReader.h"

// 引入C++标准库的文件流、字符串流、输入输出流头文件
#include <fstream>
#include <sstream>
#include <iostream>

// 使用构造函数初始化DataReader类
DataReader::DataReader(const std::string& input_filename, const std::string& output_filename)
    // 初始化列表中，直接将构造函数的参数赋值给类的成员变量
    : input_filename_(input_filename), output_filename_(output_filename) {}

// 读取数据的函数，n表示每个时间点最多读取的目标数量
std::vector<std::vector<Target>> DataReader::readData(int n) {
    // data用于存储读取到的所有目标数据
    std::vector<std::vector<Target>> data;

    // 使用ifstream打开输入文件
    std::ifstream file(input_filename_);
    // 检查文件是否成功打开，如果未成功打开，打印错误信息并返回空的data
    if (!file) {
        std::cerr << "Unable to open file: " << input_filename_ << std::endl;
        return data;
    }

    // currentGroup用于临时存储每个时间点的目标数据
    std::vector<Target> currentGroup;
    std::string line;
    // 使用getline函数读取文件中的每一行
    while (std::getline(file, line)) {
        // 如果读取到的行为空，表示当前时间点的数据已经读取完毕
        if (line.empty()) {
            // 如果currentGroup中有数据，将其添加到data中，并清空currentGroup
            if (!currentGroup.empty()) {
                data.push_back(currentGroup);
                currentGroup.clear();
            }
            // 继续读取下一行
            continue;
        }

        // 如果当前时间点的目标数量小于n，添加新的目标到currentGroup
        if (currentGroup.size() < n) {
            std::istringstream ss(line);
            Target target;
            int x, y;
            // 使用字符串流从line中读取数据，赋值给target和x、y
            ss >> target.year >> target.month >> target.day >> target.hour
               >> target.minute >> target.second >> target.id >> x >> y;
            // 将x和y作为目标位置
            target.position = Eigen::Vector2d(x, y);
            // 将目标添加到当前时间点的目标数据中
            currentGroup.push_back(target);
        }
    }
    // 如果文件已读取完毕，但currentGroup中还有数据，将其添加到data中
    if (!currentGroup.empty()) {
        data.push_back(currentGroup);
    }

    // 返回读取到的所有目标数据
    return data;
}

// 写入数据的函数
void DataReader::writeData(const std::vector<std::vector<Target>>& data) {
    // 使用ofstream打开输出文件
    std::ofstream file(output_filename_);
    // 检查文件是否成功打开，如果未成功打开，打印错误信息并直接返回
    if (!file) {
        std::cerr << "Unable to open file: " << output_filename_ << std::endl;
        return;
    }

    // 遍历所有数据，每个group代表一个时刻的所有目标数据
    for (const auto& group : data) {
        // 遍历每个时刻的所有目标数据，每个target代表一条目标数据
        for (const auto& target : group) {
            // 将目标数据写入到文件中，数据之间用空格分隔，每条数据占一行
            file << target.year << ' ' << target.month << ' ' << target.day << ' ' << target.hour
                 << ' ' << target.minute << ' ' << target.second << ' ' << target.id << ' '
                 << target.position[0] << ' ' << target.position[1] << ' '
                 << target.filtered_position[0] << ' ' << target.filtered_position[1] << ' '
                 << target.predicted_position[0] << ' ' << target.predicted_position[1] << '\n';
        }
        // 每个时刻的数据之后，写入一个空行，作为时刻之间的分隔
        file << '\n';
    }
}
