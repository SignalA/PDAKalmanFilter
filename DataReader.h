#ifndef DATA_READER_H

#define DATA_READER_H

#include <vector>
#include <string>
#include "Target.h"


// 定义一个类DataReader，用于读取和写入目标数据
class DataReader {
public:
    // 构造函数，接受两个参数，分别是输入文件名和输出文件名
    DataReader(const std::string& input_filename, const std::string& output_filename);

    // 添加了一个参数，用于指定每个时刻的目标数量
    std::vector<std::vector<Target>> readData(int n);

    // writeData方法，将处理后的数据写入到输出文件中
    void writeData(const std::vector<std::vector<Target>>& data);

private:
    std::string input_filename_;  // 输入文件名
    std::string output_filename_;  // 输出文件名
};
#endif  // 结束 #ifndef 语句