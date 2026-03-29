#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <queue>

const int COLS = 14;
const int ROWS = 17;
std::vector<std::pair<int, int>> obstacle = {
    {3,6},{4,6},{5,6},{6,6},{6,7},{6,8},{6,9},{6,10},{6,11},{6,12},
    {5,12},{4,12},{3,12},{9,3},{9,4},{9,5},{9,6},{10,3},{10,4},{10,5},
    {10,6},{1,17},{2,17},{3,17},{4,17},{5,17},{6,17},{7,17},{8,17},
    {9,17},{10,17},{11,17},{12,17},{13,17},{14,17},{14,1},{14,2},{14,3},
    {14,4},{14,5},{14,6},{14,7},{14,8},{14,9},{14,10},{14,11},{14,12},
    {14,13},{14,14},{14,15},{14,16},{14,17},
};
//bresenham直线算法
std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1) {
    std::vector<std::pair<int, int>> points;
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;
    while (true) {
        points.push_back({ x, y });
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx) { err += dx; y += sy; }
    }
    return points;
}

bool isValid(int x, int y, const std::vector<std::vector<int>>& map) {
    return x >= 0 && x < COLS && y >= 0 && y < ROWS && map[y][x] == 0;
}
//找强制邻居
bool hasForcedNeighbor(int x, int y, int dx, int dy, const std::vector<std::vector<int>>& map) {
    if (dx != 0 && dy == 0) { // 水平
        if (!isValid(x, y - 1, map) && isValid(x + dx, y - 1, map)) return true;
        if (!isValid(x, y + 1, map) && isValid(x + dx, y + 1, map)) return true;
    }
    else if (dx == 0 && dy != 0) { //垂直
        if (!isValid(x - 1, y, map) && isValid(x - 1, y + dy, map)) return true;
        if (!isValid(x + 1, y, map) && isValid(x + 1, y + dy, map)) return true;
    }
    else { // 对角线
        if (hasForcedNeighbor(x, y, dx, 0, map)) return true;
        if (hasForcedNeighbor(x, y, 0, dy, map)) return true;
    }
    return false;
}
//找跳点
std::pair<int, int> jump(int x, int y, int dx, int dy,
    const std::vector<std::vector<int>>& map,
    int goalX, int goalY) {
    int nx = x + dx;
    int ny = y + dy;
    if (!isValid(nx, ny, map)) return { -1, -1 };
    if (nx == goalX && ny == goalY) return { nx, ny };
    if (hasForcedNeighbor(nx, ny, dx, dy, map)) return { nx, ny };
    if (dx != 0 && dy != 0) {
        auto horz = jump(nx, ny, dx, 0, map, goalX, goalY);
        auto vert = jump(nx, ny, 0, dy, map, goalX, goalY);
        if (horz.first != -1 || vert.first != -1) return { nx, ny };
    }
    return jump(nx, ny, dx, dy, map, goalX, goalY);
}

struct Node {
    int x, y;
    double g;
    double fx;
    Node(int x_, int y_, double g_) : x(x_), y(y_), g(g_), fx(g_) {}
    //比较规则
    bool operator>(const Node& other) const {
        return fx > other.fx;
    }
};

double gValue[ROWS][COLS];
//父节点
std::pair<int, int> parent[ROWS][COLS];

int main() {
    std::vector<std::vector<int>> map(ROWS, std::vector<int>(COLS, 0));
    for (const auto& obs : obstacle) {
        int x = obs.first;
        int y = obs.second;
        if (x >= 0 && x < COLS && y >= 0 && y < ROWS) {
            map[y][x] = 1;
        }
    }
    std::pair<int, int> start = { 4, 9 };
    std::pair<int, int> goal = { 12, 4 };
    //初始化父节点和代价
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLS; ++j) {
            gValue[i][j] = 99;
            parent[i][j] = { -1, -1 };
        }

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    gValue[start.second][start.first] = 0.0;
    pq.emplace(start.first, start.second, 0.0);
    bool found = false;
    //八个方向
    const int dirs[8][2] = {
        {1,0}, {-1,0}, {0,1}, {0,-1},
        {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };

    while (!pq.empty()) {
        Node cur = pq.top();
        pq.pop();

        int x = cur.x, y = cur.y;
        double curG = cur.g;
        if (curG > gValue[y][x]) continue;
        if (x == goal.first && y == goal.second) {
            found = true;
            break;
        }

        for (auto& d : dirs) {
            int dx = d[0], dy = d[1];
            auto jp = jump(x, y, dx, dy, map, goal.first, goal.second);
            if (jp.first == -1) continue;

            int nx = jp.first, ny = jp.second;
            double stepCost = std::sqrt((nx - x) * (nx - x) + (ny - y) * (ny - y));
            double newG = curG + stepCost;

            if (newG < gValue[ny][nx]) {
                gValue[ny][nx] = newG;
                parent[ny][nx] = { x, y };
                pq.emplace(nx, ny, newG);
            }
        }
    }

    std::vector<std::pair<int, int>> jumpPoints;
    int cx = goal.first, cy = goal.second;
    while (cx != -1) {
        jumpPoints.push_back({ cx, cy });
        auto p = parent[cy][cx];
        cx = p.first;
        cy = p.second;
    }
    std::reverse(jumpPoints.begin(), jumpPoints.end());

    std::vector<std::pair<int, int>> Path;
    for (size_t i = 0; i < jumpPoints.size() - 1; ++i) {
        int x1 = jumpPoints[i].first, y1 = jumpPoints[i].second;
        int x2 = jumpPoints[i + 1].first, y2 = jumpPoints[i + 1].second;
        auto line = bresenham(x1, y1, x2, y2);
        if (i == 0) {
            Path.insert(Path.end(), line.begin(), line.end());
        }
        else {
            Path.insert(Path.end(), line.begin() + 1, line.end());
        }
    }
    for (const auto& pt : Path) {
        std::cout << "(" << pt.first << "," << pt.second << ") ";
    }
    std::cout << std::endl;

    return 0;
}