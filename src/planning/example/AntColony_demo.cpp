#include "AntColony.h"


#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>
#include <fstream>
#include <iomanip>

using namespace std;

// 地图大小
const int MM = 20;
// 起点和终点（展平后的索引）
const int S = 0;          // (0,0)
const int E = MM * MM - 1; // (19,19)

// 蚁群算法参数
const int K = 100;        // 迭代次数
const int M = 50;         // 蚂蚁数量
const double Alpha = 1.0; // 信息素重要程度
const double Beta = 7.0;  // 启发式信息重要程度
const double Rho = 0.3;   // 信息素蒸发系数
const double Q = 1.0;     // 信息素增加强度

// 网格地图（1表示障碍物）
vector<vector<int>> G = {
    {0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0},
    {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0},
    {1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0}
};

// 随机数生成器
random_device rd;
mt19937 gen(rd());
uniform_real_distribution<double> dist(0.0, 1.0);

// 将二维坐标转换为一维索引
int xy2index(int x, int y) {
    return y * MM + x;
}

// 将一维索引转换为二维坐标
pair<int, int> index2xy(int index) {
    int y = index / MM;
    int x = index % MM;
    return {x, y};
}

// 计算两点之间的欧氏距离
double distance(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

// 构建邻接矩阵D
vector<vector<double>> buildAdjacencyMatrix() {
    vector<vector<double>> D(MM * MM, vector<double>(MM * MM, 0.0));
    for (int i = 0; i < MM; ++i) {
        for (int j = 0; j < MM; ++j) {
            if (G[i][j] == 1) continue; // 障碍物跳过
            int current = xy2index(j, i);
            // 检查4连通或8连通邻居
            for (int di = -1; di <= 1; ++di) {
                for (int dj = -1; dj <= 1; ++dj) {
                    if (di == 0 && dj == 0) continue; // 跳过自身
                    int ni = i + di;
                    int nj = j + dj;
                    if (ni >= 0 && ni < MM && nj >= 0 && nj < MM && G[ni][nj] == 0) {
                        int neighbor = xy2index(nj, ni);
                        double dist_val = (abs(di) + abs(dj) == 1) ? 1.0 : sqrt(2.0);
                        D[current][neighbor] = dist_val;
                    }
                }
            }
        }
    }
    return D;
}

// 计算启发式信息Eta（到终点的距离倒数）
vector<double> computeHeuristic() {
    vector<double> Eta(MM * MM, 0.0);
    auto [ex, ey] = index2xy(E);
    for (int i = 0; i < MM; ++i) {
        for (int j = 0; j < MM; ++j) {
            if (G[i][j] == 1) continue; // 障碍物跳过
            int index = xy2index(j, i);
            auto [x, y] = index2xy(index);
            if (index != E) {
                Eta[index] = 1.0 / distance(x, y, ex, ey);
            } else {
                Eta[index] = 100.0; // 终点设为较大值
            }
        }
    }
    return Eta;
}

// 转轮赌法选择下一个节点
int rouletteWheelSelection(const vector<double>& prob) {
    double r = dist(gen);
    double sum = 0.0;
    for (size_t i = 0; i < prob.size(); ++i) {
        sum += prob[i];
        if (r <= sum) {
            return static_cast<int>(i);
        }
    }
    return static_cast<int>(prob.size()) - 1;
}

// 蚁群算法主函数
void antColonyOptimization() {
    // 初始化邻接矩阵和启发式信息
    vector<vector<double>> D = buildAdjacencyMatrix();
    vector<double> Eta = computeHeuristic();
    int N = MM * MM;

    // 初始化信息素矩阵
    vector<vector<double>> Tau(N, vector<double>(N, 8.0));

    // 记录最优路径
    vector<int> bestPath;
    double bestLength = numeric_limits<double>::max();
    vector<double> minPL(K, numeric_limits<double>::max());

    // 迭代K次
    for (int k = 0; k < K; ++k) {
        vector<vector<int>> ROUTES(M);
        vector<double> PL(M, 0.0);

        // 每只蚂蚁独立搜索
        for (int m = 0; m < M; ++m) {
            vector<int> Path = {S};
            vector<int> TABU(N, 1);
            TABU[S] = 0;
            int W = S;
            double PLkm = 0.0;

            while (W != E) {
                vector<double> PP;
                vector<int> LJD; // 可选节点列表

                // 找到所有未访问的邻居
                for (int j = 0; j < N; ++j) {
                    if (D[W][j] > 0 && TABU[j] == 1) {
                        LJD.push_back(j);
                    }
                }

                // 如果没有可选节点，退出
                if (LJD.empty()) break;

                // 计算转移概率
                for (int j : LJD) {
                    PP.push_back(pow(Tau[W][j], Alpha) * pow(Eta[j], Beta));
                }

                // 归一化概率
                double sumPP = accumulate(PP.begin(), PP.end(), 0.0);
                for (auto& p : PP) {
                    p /= sumPP;
                }

                // 转轮赌选择下一个节点
                int next = LJD[rouletteWheelSelection(PP)];
                Path.push_back(next);
                PLkm += D[W][next];
                W = next;
                TABU[W] = 0;
            }

            // 记录路径和长度
            ROUTES[m] = Path;
            if (Path.back() == E) {
                PL[m] = PLkm;
                if (PLkm < bestLength) {
                    bestLength = PLkm;
                    bestPath = Path;
                }
            } else {
                PL[m] = numeric_limits<double>::max();
            }
        }

        // 更新信息素
        vector<vector<double>> Delta_Tau(N, vector<double>(N, 0.0));
        for (int m = 0; m < M; ++m) {
            if (PL[m] != numeric_limits<double>::max()) {
                for (size_t s = 0; s < ROUTES[m].size() - 1; ++s) {
                    int x = ROUTES[m][s];
                    int y = ROUTES[m][s + 1];
                    Delta_Tau[x][y] += Q / PL[m];
                    Delta_Tau[y][x] += Q / PL[m];
                }
            }
        }

        // 信息素挥发和增强
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                Tau[i][j] = (1 - Rho) * Tau[i][j] + Delta_Tau[i][j];
            }
        }

        // 记录当前代的最优路径长度
        double minPLk = *min_element(PL.begin(), PL.end());
        minPL[k] = minPLk;
    }

    // 输出结果
    cout << "Best path length: " << bestLength << endl;
    cout << "Best path: ";
    for (int node : bestPath) {
        auto [x, y] = index2xy(node);
        cout << "(" << x << "," << y << ") ";
    }
    cout << endl;

    // 绘制收敛曲线（保存到文件）
    ofstream outFile("convergence.csv");
    for (int i = 0; i < K; ++i) {
        outFile << i + 1 << "," << minPL[i] << endl;
    }
    outFile.close();
}

int main() {
    antColonyOptimization();
    return 0;
}