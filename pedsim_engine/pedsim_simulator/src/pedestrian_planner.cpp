#include <pedsim_simulator/pedestrian_planner.h>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

// ── singleton ──────────────────────────────────────────────────────────────

PedestrianPlanner& PedestrianPlanner::instance()
{
    static PedestrianPlanner inst;
    return inst;
}

// ── public ─────────────────────────────────────────────────────────────────

bool PedestrianPlanner::initialize()
{
    std::string yamlPath;
    if (!ros::param::get("/map_path", yamlPath)) {
        ROS_WARN("[PedestrianPlanner] /map_path not set — path planner disabled");
        return false;
    }
    return loadMap(yamlPath);
}

std::vector<Ped::Tvector> PedestrianPlanner::planPath(const Ped::Tvector& start,
                                                        const Ped::Tvector& goal)
{
    if (!ready_) {
        initialize();
        if (!ready_) return {};
    }

    auto [sx, sy] = worldToGrid(start.x, start.y);
    auto [gx, gy] = worldToGrid(goal.x,  goal.y);

    sx = std::max(0, std::min(width_  - 1, sx));
    sy = std::max(0, std::min(height_ - 1, sy));
    gx = std::max(0, std::min(width_  - 1, gx));
    gy = std::max(0, std::min(height_ - 1, gy));

    auto [fsx, fsy] = snapToFree(sx, sy);
    auto [fgx, fgy] = snapToFree(gx, gy);

    // Planning and collision grids share the same inflation (kPlanningInflation
    // == kCollisionBuffer == 0.20m), so a single A* pass suffices.
    auto raw = astarOnGrid(fsx, fsy, fgx, fgy, /*useCollision=*/false);
    if (raw.empty()) {
        ROS_DEBUG("[PedestrianPlanner] A* found no path (%.1f,%.1f)->(%.1f,%.1f)",
                  start.x, start.y, goal.x, goal.y);
        return {};
    }

    // Replace the last grid-snapped point with the exact goal whenever the
    // goal is physically reachable (not within the 0.20m collision buffer).
    // The planning grid uses 0.70m inflation which is conservative — many
    // valid waypoints sit in the 0.20–0.70m zone and are perfectly reachable.
    // Keeping the snapped endpoint instead causes the path to terminate at the
    // wrong y-level, making the agent oscillate without ever reaching the goal.
    int egx = std::max(0, std::min(width_  - 1, gx));
    int egy = std::max(0, std::min(height_ - 1, gy));
    if (passableCollision(egx, egy)) {
        raw.back() = goal;
    }
    // else: goal is inside the 0.20m hard wall — keep the snapped endpoint.

    // Return the raw A* path without string-pulling. Every consecutive pair of
    // cells is a passable 8-connected neighbour, so there is never a wall
    // between the agent's current position and the next subgoal. String-pulling
    // creates long segments that can drift into the buffer zone when social
    // forces push the agent sideways, causing it to get stuck on the surface
    // trying to reach a subgoal on the other side.
    return raw;
}

// ── private: map loading ───────────────────────────────────────────────────

bool PedestrianPlanner::loadMap(const std::string& yamlPath)
{
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(yamlPath);
    } catch (const std::exception& e) {
        ROS_ERROR("[PedestrianPlanner] Cannot load yaml %s: %s", yamlPath.c_str(), e.what());
        return false;
    }

    resolution_ = cfg["resolution"].as<double>(0.1);

    auto origin = cfg["origin"].as<std::vector<double>>();
    originX_ = origin.at(0);
    originY_ = origin.at(1);

    double occupiedThresh = cfg["occupied_thresh"].as<double>(0.65);
    bool   negate         = cfg["negate"].as<int>(0) != 0;
    std::string imageName = cfg["image"].as<std::string>();

    std::string dir = yamlPath.substr(0, yamlPath.find_last_of('/'));
    std::string pgmPath = dir + "/" + imageName;

    if (!loadImage(pgmPath, occupiedThresh, negate)) return false;

    // Collision grid: small physical buffer. The hard-stop fires here,
    // giving agents room to move through any corridor the planner approves.
    collisionGrid_ = grid_;
    int collCells = static_cast<int>(std::ceil(PedestrianPlanner::kCollisionBuffer / resolution_));
    {
        std::vector<bool> tmp = collisionGrid_;
        for (int y = 0; y < height_; y++) {
            for (int x = 0; x < width_; x++) {
                if (tmp[y * width_ + x]) continue;
                for (int dy = -collCells; dy <= collCells; dy++) {
                    for (int dx = -collCells; dx <= collCells; dx++) {
                        if (dx*dx + dy*dy > collCells*collCells) continue;
                        int nx = x+dx, ny = y+dy;
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_)
                            collisionGrid_[ny * width_ + nx] = false;
                    }
                }
            }
        }
    }

    // Planning grid: inflate to seal corridors narrower than 2× kPlanningInflation
    // so A* never routes pedestrians into spaces they can't physically traverse.
    int cells = static_cast<int>(std::ceil(PedestrianPlanner::kPlanningInflation / resolution_));
    inflateGrid(cells);

    ROS_INFO("[PedestrianPlanner] Ready: %dx%d cells, res=%.3fm, planning inflation %d cells",
             width_, height_, resolution_, cells);
    ready_ = true;
    return true;
}

bool PedestrianPlanner::loadImage(const std::string& path,
                                   double occupiedThresh, bool negate)
{
    // Read as grayscale regardless of whether the source is PGM, PNG (RGBA),
    // or any other format OpenCV supports.
    cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        ROS_ERROR("[PedestrianPlanner] Cannot load map image: %s", path.c_str());
        return false;
    }

    width_  = img.cols;
    height_ = img.rows;
    grid_.resize(static_cast<std::size_t>(width_ * height_));

    const double maxval = 255.0;
    for (int row = 0; row < height_; row++) {
        for (int col = 0; col < width_; col++) {
            double norm = static_cast<double>(img.at<uint8_t>(row, col)) / maxval;
            // ROS convention: bright = free (norm near 1 → pOcc near 0).
            // negate=true inverts (dark = free).
            double pOcc = negate ? norm : 1.0 - norm;
            grid_[row * width_ + col] = (pOcc < occupiedThresh);
        }
    }
    return true;
}

void PedestrianPlanner::inflateGrid(int cells)
{
    std::vector<bool> inflated = grid_;
    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            if (grid_[y * width_ + x]) continue;  // free cell, nothing to expand
            for (int dy = -cells; dy <= cells; dy++) {
                for (int dx = -cells; dx <= cells; dx++) {
                    if (dx * dx + dy * dy > cells * cells) continue;
                    int nx = x + dx, ny = y + dy;
                    if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_)
                        inflated[ny * width_ + nx] = false;
                }
            }
        }
    }
    grid_ = std::move(inflated);
}

// ── private: planning ──────────────────────────────────────────────────────

std::vector<Ped::Tvector> PedestrianPlanner::astarOnGrid(int sx, int sy,
                                                           int gx, int gy,
                                                           bool useCollision) const
{
    auto cell_ok = [&](int x, int y) {
        return useCollision ? passableCollision(x, y) : passable(x, y);
    };

    if (!cell_ok(sx, sy) || !cell_ok(gx, gy)) return {};
    if (sx == gx && sy == gy) return {gridToWorld(gx, gy)};

    struct Node {
        int   x, y;
        float g, f;
        bool operator>(const Node& o) const { return f > o.f; }
    };

    const int N = width_ * height_;
    auto idx = [this](int x, int y) { return y * width_ + x; };

    std::vector<float> g(N, std::numeric_limits<float>::infinity());
    std::vector<int>   parent(N, -1);
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    g[idx(sx, sy)] = 0.f;
    open.push({sx, sy, 0.f, 0.f});

    const int   DX[] = { 1,-1, 0, 0, 1, 1,-1,-1};
    const int   DY[] = { 0, 0, 1,-1, 1,-1, 1,-1};
    const float DC[] = {1.f,1.f,1.f,1.f,1.414f,1.414f,1.414f,1.414f};

    while (!open.empty()) {
        Node cur = open.top(); open.pop();
        int ci = idx(cur.x, cur.y);

        if (cur.g > g[ci]) continue;  // stale entry

        if (cur.x == gx && cur.y == gy) {
            std::vector<Ped::Tvector> path;
            int at = ci;
            while (at != -1) {
                path.push_back(gridToWorld(at % width_, at / width_));
                at = parent[at];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 8; i++) {
            int nx = cur.x + DX[i], ny = cur.y + DY[i];
            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
            if (!cell_ok(nx, ny)) continue;
            int ni = idx(nx, ny);
            float ng = g[ci] + DC[i];
            if (ng < g[ni]) {
                g[ni]      = ng;
                parent[ni] = ci;
                float h = std::hypot(float(nx - gx), float(ny - gy));
                open.push({nx, ny, ng, ng + h});
            }
        }
    }

    return {};
}

// Keep old name as a thin wrapper so existing callers still compile.
std::vector<Ped::Tvector> PedestrianPlanner::astar(int sx, int sy,
                                                     int gx, int gy) const
{
    return astarOnGrid(sx, sy, gx, gy, false);
}

bool PedestrianPlanner::hasLineOfSight(int x0, int y0, int x1, int y1) const
{
    int dx =  std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (!passable(x0, y0)) return false;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        // Supercover: when stepping diagonally, also check the two axis-aligned
        // corner cells. Standard Bresenham only checks the cell it lands on and
        // will pass through a wall corner diagonally — the agent then gets stuck
        // trying to reach a subgoal on the other side of that corner.
        bool stepX = (e2 >= dy);
        bool stepY = (e2 <= dx);
        if (stepX && stepY) {
            if (!passable(x0 + sx, y0) || !passable(x0, y0 + sy)) return false;
        }
        if (stepX) { err += dy; x0 += sx; }
        if (stepY) { err += dx; y0 += sy; }
    }
    return true;
}

std::vector<Ped::Tvector> PedestrianPlanner::smoothPath(
    const std::vector<Ped::Tvector>& path) const
{
    if (path.size() <= 2) return path;

    std::vector<Ped::Tvector> smooth;
    smooth.push_back(path[0]);

    std::size_t anchor = 0;
    for (std::size_t i = 1; i + 1 < path.size(); i++) {
        auto [ax, ay] = worldToGrid(path[anchor].x, path[anchor].y);
        auto [bx, by] = worldToGrid(path[i + 1].x,  path[i + 1].y);
        if (!hasLineOfSight(ax, ay, bx, by)) {
            smooth.push_back(path[i]);
            anchor = i;
        }
    }
    smooth.push_back(path.back());
    return smooth;
}

// ── private: grid helpers ──────────────────────────────────────────────────

std::pair<int,int> PedestrianPlanner::worldToGrid(double wx, double wy) const
{
    int gx = static_cast<int>(std::round((wx - originX_) / resolution_));
    int gy = height_ - 1 - static_cast<int>(std::round((wy - originY_) / resolution_));
    return {gx, gy};
}

Ped::Tvector PedestrianPlanner::gridToWorld(int gx, int gy) const
{
    double wx = originX_ + gx * resolution_;
    double wy = originY_ + (height_ - 1 - gy) * resolution_;
    return Ped::Tvector(wx, wy, 0.0);
}

bool PedestrianPlanner::passable(int gx, int gy) const
{
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return false;
    return grid_[gy * width_ + gx];
}

bool PedestrianPlanner::passableCollision(int gx, int gy) const
{
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return false;
    return collisionGrid_[gy * width_ + gx];
}

Ped::Tvector PedestrianPlanner::clampToFreeSpace(const Ped::Tvector& oldPos,
                                                   const Ped::Tvector& newPos) const
{
    if (!ready_) return newPos;

    auto clampIdx = [](int v, int max) { return std::max(0, std::min(max - 1, v)); };

    auto [ngx, ngy] = worldToGrid(newPos.x, newPos.y);
    ngx = clampIdx(ngx, width_); ngy = clampIdx(ngy, height_);

    // Use the collision grid (0.20 m inflation) for the physical hard stop.
    // The planning grid (0.70 m) is only for routing — agents have room to
    // move freely within corridors the planner approves.
    if (passableCollision(ngx, ngy)) return newPos;

    // Try sliding along X (keep new X, old Y)
    auto [sxA, syA] = worldToGrid(newPos.x, oldPos.y);
    sxA = clampIdx(sxA, width_); syA = clampIdx(syA, height_);
    if (passableCollision(sxA, syA))
        return Ped::Tvector(newPos.x, oldPos.y, newPos.z);

    // Try sliding along Y (old X, new Y)
    auto [sxB, syB] = worldToGrid(oldPos.x, newPos.y);
    sxB = clampIdx(sxB, width_); syB = clampIdx(syB, height_);
    if (passableCollision(sxB, syB))
        return Ped::Tvector(oldPos.x, newPos.y, newPos.z);

    // Full revert
    return oldPos;
}

std::pair<int,int> PedestrianPlanner::snapToFree(int gx, int gy) const
{
    if (passable(gx, gy)) return {gx, gy};

    const int DX[] = {0,0,1,-1,1,1,-1,-1};
    const int DY[] = {1,-1,0,0,1,-1,1,-1};

    std::queue<std::pair<int,int>> q;
    std::vector<bool> visited(width_ * height_, false);
    visited[gy * width_ + gx] = true;
    q.push({gx, gy});

    while (!q.empty()) {
        auto [cx, cy] = q.front(); q.pop();
        for (int i = 0; i < 8; i++) {
            int nx = cx + DX[i], ny = cy + DY[i];
            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
            int ni = ny * width_ + nx;
            if (visited[ni]) continue;
            visited[ni] = true;
            if (passable(nx, ny)) return {nx, ny};
            q.push({nx, ny});
        }
    }

    return {gx, gy};  // fallback
}

std::pair<int,int> PedestrianPlanner::snapToFreeCollision(int gx, int gy) const
{
    if (passableCollision(gx, gy)) return {gx, gy};

    const int DX[] = {0,0,1,-1,1,1,-1,-1};
    const int DY[] = {1,-1,0,0,1,-1,1,-1};

    std::queue<std::pair<int,int>> q;
    std::vector<bool> visited(width_ * height_, false);
    visited[gy * width_ + gx] = true;
    q.push({gx, gy});

    while (!q.empty()) {
        auto [cx, cy] = q.front(); q.pop();
        for (int i = 0; i < 8; i++) {
            int nx = cx + DX[i], ny = cy + DY[i];
            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
            int ni = ny * width_ + nx;
            if (visited[ni]) continue;
            visited[ni] = true;
            if (passableCollision(nx, ny)) return {nx, ny};
            q.push({nx, ny});
        }
    }

    return {gx, gy};  // fallback
}

