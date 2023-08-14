#include "HungarianAlgorithm.h"

// 定义匈牙利算法的求解函数，参数是成本矩阵和分配结果
void HungarianAlgorithm::Solve(const std::vector<std::vector<double>>& costMatrix, std::vector<int>& assignment) {
    // 获取成本矩阵的行数和列数
    int n = costMatrix.size(), m = costMatrix[0].size();

    // 定义匈牙利算法中用到的一些变量
    std::vector<int> u(n+1), v(m+1), p(m+1), way(m+1);

    // 主循环，对于每一行执行算法
    for (int i=1; i<=n; ++i) {
        // 初始列选择为0
        p[0] = i;
        int j0 = 0;

        // minv保存了最小元素值，初始化为最大值，used标记该列是否已被处理
        std::vector<double> minv(m+1, std::numeric_limits<double>::max());
        std::vector<char> used(m+1, false);

        // 这个循环在所有列上执行处理，直到找到一个完美匹配
        do {
            used[j0] = true;
            int i0 = p[j0], j1;
            double delta = std::numeric_limits<double>::max();

            // 遍历所有列，找到最小的未处理元素
            for (int j=1; j<=m; ++j)
                if (!used[j]) {
                    double cur = costMatrix[i0-1][j-1]-u[i0]-v[j];
                    if (cur<minv[j])
                        minv[j] = cur, way[j] = j0;
                    if (minv[j]<delta)
                        delta = minv[j], j1 = j;
                }

            // 更新u和v，准备下一次迭代
            for (int j=0; j<=m; ++j)
                if (used[j])
                    u[p[j]] += delta, v[j] -= delta;
                else
                    minv[j] -= delta;
            j0 = j1;
        } while (p[j0] != 0);

        // 反向回溯找到匹配路径，并更新p值
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    // 分配结果保存在assignment中，每个工作的分配情况都存储在其中
    assignment.resize(n);
    for (int j=1; j<=m; ++j)
        if (p[j] > 0)
            assignment[p[j] - 1] = j - 1;
}
