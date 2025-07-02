// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <random>
// #include <algorithm>
// #include <limits>
// #include <iomanip>
// #include <queue>
// #include <unordered_set>
// #include <fstream>
// #include <sstream>
// #include <chrono>

// using namespace std;
// using namespace std::chrono;

// // 节点类，表示地图上的一个点
// class Node {
// public:
//     int x, y;
//     double gScore, fScore;
//     Node* parent;

//     Node(int x, int y) : x(x), y(y), gScore(numeric_limits<double>::max()), 
//                          fScore(numeric_limits<double>::max()), parent(nullptr) {}

//     bool operator==(const Node& other) const {
//         return x == other.x && y == other.y;
//     }
// };

// // 节点哈希函数，用于unordered_set
// namespace std {
//     template <>
//     struct hash<Node> {
//         size_t operator()(const Node& node) const {
//             return hash<int>()(node.x) ^ hash<int>()(node.y);
//         }
//     };
// }

// // 栅格地图类
// class GridMap {
// private:
//     int width, height;
//     vector<vector<int>> grid; // 0表示可通行，1表示障碍物

// public:
//     GridMap(int w, int h) : width(w), height(h) {
//         grid.resize(height, vector<int>(width, 0));
//     }

//     int getWidth() const { return width; }
//     int getHeight() const { return height; }

//     bool isTraversable(int x, int y) const {
//         return x >= 0 && x < width && y >= 0 && y < height && grid[y][x] == 0;
//     }

//     void setObstacle(int x, int y, bool obstacle = true) {
//         if (x >= 0 && x < width && y >= 0 && y < height) {
//             grid[y][x] = obstacle ? 1 : 0;
//         }
//     }

//     // 获取相邻节点（8邻域）
//     vector<Node> getNeighbors(const Node& node) const {
//         vector<Node> neighbors;
//         for (int dx = -1; dx <= 1; ++dx) {
//             for (int dy = -1; dy <= 1; ++dy) {
//                 if (dx == 0 && dy == 0) continue;
//                 int nx = node.x + dx;
//                 int ny = node.y + dy;
//                 if (isTraversable(nx, ny)) {
//                     neighbors.emplace_back(nx, ny);
//                 }
//             }
//         }
//         return neighbors;
//     }

//     // 计算两个节点之间的欧几里得距离
//     double calculateDistance(const Node& a, const Node& b) const {
//         return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
//     }

//     // 打印地图（简化显示）
//     void print() const {
//         // 只打印地图的一部分，因为200x300太大
//         cout << "Map (partial view, 50x20 section):" << endl;
//         for (int y = 50; y < 70; ++y) {
//             for (int x = 50; x < 100; ++x) {
//                 if (x == 50 && y == 50) cout << "S ";
//                 else if (x == width-51 && y == height-51) cout << "E ";
//                 else cout << (grid[y][x] ? "## " : ". ");
//             }
//             cout << endl;
//         }
//         cout << "..." << endl;
//     }

//     // 从文件加载地图
//     static GridMap loadFromFile(const string& filename) {
//         ifstream file(filename);
//         if (!file.is_open()) {
//             cerr << "无法打开文件: " << filename << endl;
//             exit(1);
//         }
        
//         vector<vector<int>> grid;
//         string line;
        
//         while (getline(file, line)) {
//             vector<int> row;
//             for (char c : line) {
//                 if (c == '0' || c == '1') {
//                     row.push_back(c - '0');
//                 }
//             }
//             if (!row.empty()) {
//                 grid.push_back(row);
//             }
//         }
        
//         if (grid.empty()) {
//             cerr << "地图文件为空或格式错误" << endl;
//             exit(1);
//         }
        
//         GridMap map(grid[0].size(), grid.size());
//         for (int y = 0; y < grid.size(); ++y) {
//             for (int x = 0; x < grid[y].size(); ++x) {
//                 if (grid[y][x] == 1) {
//                     map.setObstacle(x, y);
//                 }
//             }
//         }
        
//         return map;
//     }
    
//     // 保存地图到文件
//     void saveToFile(const string& filename) {
//         ofstream file(filename);
//         if (!file.is_open()) {
//             cerr << "无法创建文件: " << filename << endl;
//             return;
//         }
        
//         for (int y = 0; y < height; ++y) {
//             for (int x = 0; x < width; ++x) {
//                 file << (grid[y][x] ? '1' : '0');
//             }
//             file << '\n';
//         }
//     }
// };

// // 蚁群算法类
// class GridAntColonyOptimization {
// private:
//     GridMap& map;
//     int numAnts;
//     int maxIterations;
//     double alpha;
//     double beta;
//     double evaporationRate;
//     double Q;
//     bool use8Neighborhood;
    
//     vector<vector<double>> pheromones;
//     Node startNode;
//     Node endNode;
//     vector<vector<Node>> graph;

// public:
//     GridAntColonyOptimization(GridMap& gridMap, int ants, int iterations, 
//                             double a, double b, double rho, double q, bool use8N)
//         : map(gridMap), numAnts(ants), maxIterations(iterations), 
//           alpha(a), beta(b), evaporationRate(rho), Q(q), 
//           use8Neighborhood(use8N), startNode(0, 0), endNode(0, 0) {
//         initializePheromones();
//     }

//     void setStartEnd(int startX, int startY, int endX, int endY) {
//         startNode = Node(startX, startY);
//         endNode = Node(endX, endY);
//     }

//     void initializePheromones() {
//         pheromones.resize(map.getHeight(), vector<double>(map.getWidth(), 0.1));
//     }

//     void buildGraph() {
//         graph.clear();
//         graph.resize(map.getHeight(), vector<Node>(map.getWidth(), Node(0, 0)));
        
//         for (int y = 0; y < map.getHeight(); ++y) {
//             for (int x = 0; x < map.getWidth(); ++x) {
//                 graph[y][x] = Node(x, y);
//             }
//         }
//     }

//     // 选择下一个节点（概率选择）
//     Node selectNextNode(const Node& current, const vector<Node>& neighbors) {
//         vector<double> probabilities;
//         double sum = 0.0;
        
//         // 计算每个邻居的选择概率
//         for (const auto& neighbor : neighbors) {
//             double pheromone = pheromones[neighbor.y][neighbor.x];
//             double distance = map.calculateDistance(current, neighbor);
//             double probability = pow(pheromone, alpha) * pow(1.0 / distance, beta);
//             probabilities.push_back(probability);
//             sum += probability;
//         }
        
//         // 归一化概率
//         for (auto& prob : probabilities) {
//             prob /= sum;
//         }
        
//         // 轮盘赌选择
//         double r = (double)rand() / RAND_MAX;
//         double cumulativeProb = 0.0;
//         for (size_t i = 0; i < neighbors.size(); ++i) {
//             cumulativeProb += probabilities[i];
//             if (r <= cumulativeProb) {
//                 return neighbors[i];
//             }
//         }
        
//         return neighbors.back(); // 防止浮点误差
//     }

//     // 蚂蚁构建路径
//     vector<Node> constructAntPath() {
//         vector<Node> path;
//         unordered_set<Node> visited;
        
//         Node current = startNode;
//         path.push_back(current);
//         visited.insert(current);
        
//         while (!(current == endNode)) {
//             vector<Node> neighbors = map.getNeighbors(current);
//             vector<Node> validNeighbors;
            
//             // 过滤掉已访问的节点
//             for (auto& neighbor : neighbors) {
//                 if (visited.find(neighbor) == visited.end()) {
//                     validNeighbors.push_back(neighbor);
//                 }
//             }
            
//             if (validNeighbors.empty()) {
//                 break; // 无路可走
//             }
            
//             current = selectNextNode(current, validNeighbors);
//             path.push_back(current);
//             visited.insert(current);
            
//             // 防止无限循环
//             if (path.size() > 2 * (map.getWidth() + map.getHeight())) {
//                 break;
//             }
//         }
        
//         return path;
//     }

//     // 更新信息素
//     void updatePheromones(const vector<vector<Node>>& allPaths) {
//         // 信息素挥发
//         for (int y = 0; y < map.getHeight(); ++y) {
//             for (int x = 0; x < map.getWidth(); ++x) {
//                 pheromones[y][x] *= (1.0 - evaporationRate);
//             }
//         }
        
//         // 信息素增强
//         for (const auto& path : allPaths) {
//             double pathLength = 0.0;
//             for (size_t i = 1; i < path.size(); ++i) {
//                 pathLength += map.calculateDistance(path[i-1], path[i]);
//             }
            
//             double pheromoneDelta = Q / pathLength;
//             for (const auto& node : path) {
//                 pheromones[node.y][node.x] += pheromoneDelta;
//             }
//         }
//     }

//     // 运行蚁群算法
//     vector<Node> run() {
//         vector<Node> bestPath;
//         double bestPathLength = numeric_limits<double>::max();
        
//         auto start = high_resolution_clock::now();
        
//         for (int iter = 0; iter < maxIterations; ++iter) {
//             vector<vector<Node>> allPaths;
            
//             // 每只蚂蚁构建路径
//             for (int i = 0; i < numAnts; ++i) {
//                 vector<Node> path = constructAntPath();
//                 if (!path.empty() && path.back() == endNode) {
//                     allPaths.push_back(path);
                    
//                     // 计算路径长度
//                     double length = 0.0;
//                     for (size_t j = 1; j < path.size(); ++j) {
//                         length += map.calculateDistance(path[j-1], path[j]);
//                     }
                    
//                     // 更新最佳路径
//                     if (length < bestPathLength) {
//                         bestPathLength = length;
//                         bestPath = path;
//                     }
//                 }
//             }
            
//             // 更新信息素
//             updatePheromones(allPaths);
            
//             // 打印进度
//             if (iter % 10 == 0) {
//                 auto now = high_resolution_clock::now();
//                 auto duration = duration_cast<seconds>(now - start).count();
//                 cout << "Iteration " << iter << ", Best Path Length: " << bestPathLength 
//                      << ", Time: " << duration << "s" << endl;
//             }
//         }
        
//         return bestPath;
//     }

//     // 打印路径
//     void printPath(const vector<Node>& path) const {
//         if (path.empty()) {
//             cout << "No path found!" << endl;
//             return;
//         }
        
//         cout << "Path length: " << path.size() << " nodes" << endl;
        
//         // 由于地图太大，只打印部分路径
//         cout << "Partial path (first 50 nodes):" << endl;
//         for (size_t i = 0; i < min((size_t)50, path.size()); ++i) {
//             cout << "(" << path[i].x << ", " << path[i].y << ")";
//             if (i != path.size() - 1) cout << " -> ";
//         }
//         cout << endl;
//     }
// };

// // 添加随机障碍物生成
// void addRandomObstacles(GridMap& map, double obstacleRatio) {
//     random_device rd;
//     mt19937 gen(rd());
//     uniform_real_distribution<> dis(0.0, 1.0);
    
//     for (int y = 0; y < map.getHeight(); ++y) {
//         for (int x = 0; x < map.getWidth(); ++x) {
//             // 确保边界保持可通行
//             if (x > 0 && x < map.getWidth()-1 && y > 0 && y < map.getHeight()-1) {
//                 if (dis(gen) < obstacleRatio) {
//                     map.setObstacle(x, y);
//                 }
//             }
//         }
//     }
// }

// // 添加迷宫式障碍物
// void addMazeObstacles(GridMap& map) {
//     int w = map.getWidth();
//     int h = map.getHeight();
    
//     // 创建垂直和水平墙壁
//     for (int x = 20; x < w-20; x += 3) {
//         for (int y = 15; y < h-15; ++y) {
//             if (y % 5 != 0) {
//                 map.setObstacle(x, y);
//             }
//         }
//     }
    
//     for (int y = 20; y < h-20; y += 3) {
//         for (int x = 15; x < w-15; ++x) {
//             if (x % 5 != 0) {
//                 map.setObstacle(x, y);
//             }
//         }
//     }
    
//     // 添加一些随机缺口
//     random_device rd;
//     mt19937 gen(rd());
//     uniform_int_distribution<> xdis(20, w-21);
//     uniform_int_distribution<> ydis(20, h-21);
    
//     for (int i = 0; i < 2*(w+h); ++i) {
//         int x = xdis(gen);
//         int y = ydis(gen);
//         if (x % 3 == 0 || y % 3 == 0) {
//             map.setObstacle(x, y, 0); // 清除障碍物
//         }
//     }
// }

// // 添加区域障碍物（大型不可通行区域）
// void addRegionObstacles(GridMap& map) {
//     random_device rd;
//     mt19937 gen(rd());
//     uniform_int_distribution<> wdist(5, 15);
//     uniform_int_distribution<> hdist(5, 15);
//     uniform_int_distribution<> xdist(30, map.getWidth()-60);
//     uniform_int_distribution<> ydist(30, map.getHeight()-60);
    
//     // 添加5个随机矩形障碍物区域
//     for (int i = 0; i < 5; ++i) {
//         int x = xdist(gen);
//         int y = ydist(gen);
//         int w = wdist(gen);
//         int h = hdist(gen);
        
//         for (int dy = 0; dy < h; ++dy) {
//             for (int dx = 0; dx < w; ++dx) {
//                 int nx = x + dx;
//                 int ny = y + dy;
//                 if (nx >= 0 && nx < map.getWidth() && ny >= 0 && ny < map.getHeight()) {
//                     map.setObstacle(nx, ny);
//                 }
//             }
//         }
//     }
// }

// int main() {
//     // 创建200x300的超大栅格地图
//     const int WIDTH = 300;
//     const int HEIGHT = 200;
//     GridMap map(WIDTH, HEIGHT);
    
//     // 添加结构化障碍物 - 迷宫式布局
//     addMazeObstacles(map);
    
//     // 添加大型区域障碍物
//     addRegionObstacles(map);
    
//     // 添加随机障碍物（障碍物占比15%）
//     addRandomObstacles(map, 0.15);
    
//     // 确保起点和终点是可通行的
//     Node start(10, 10);
//     Node end(WIDTH-11, HEIGHT-11);
    
//     // 清除起点和终点周围的障碍物
//     for (int y = start.y-5; y <= start.y+5; ++y) {
//         for (int x = start.x-5; x <= start.x+5; ++x) {
//             map.setObstacle(x, y, 0);
//         }
//     }
//     for (int y = end.y-5; y <= end.y+5; ++y) {
//         for (int x = end.x-5; x <= end.x+5; ++x) {
//             map.setObstacle(x, y, 0);
//         }
//     }
    
//     // 保存地图到文件（可选）
//     // map.saveToFile("large_complex_map.txt");
    
//     cout << "Large Grid Map with Complex Obstacles (300x200):" << endl;
//     map.print();
//     cout << endl;
    
//     // 创建ACO实例
//     // 参数调整以适应更大地图：
//     // - 增加蚂蚁数量(50)
//     // - 增加迭代次数(200)
//     // - 增加信息素挥发率(0.4)以避免过早收敛
//     // - 增加Q值(20.0)以增强信息素更新
//     GridAntColonyOptimization aco(map, 50, 200, 1.0, 2.0, 0.4, 20.0, true);
    
//     // 设置起点和终点
//     aco.setStartEnd(start.x, start.y, end.x, end.y);
    
//     // 构建图结构
//     aco.buildGraph();
    
//     // 运行算法
//     auto start = high_resolution_clock::now();
//     vector<Node> bestPath = aco.run();
//     auto end = high_resolution_clock::now();
//     auto duration = duration_cast<seconds>(end - start).count();
    
//     // 打印结果
//     cout << "\nAlgorithm completed in " << duration << " seconds" << endl;
//     cout << "\nBest path found:" << endl;
//     aco.printPath(bestPath);
    
//     return 0;
// }