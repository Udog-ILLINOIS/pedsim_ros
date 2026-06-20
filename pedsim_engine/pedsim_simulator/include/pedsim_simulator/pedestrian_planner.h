#pragma once

#include <pedsim/ped_vector.h>
#include <string>
#include <utility>
#include <vector>

class PedestrianPlanner {
public:
    // Tunable parameters — edit pedestrian_planner.cpp for descriptions.
    static constexpr double kPlanningInflation = 0.20;
    static constexpr double kCollisionBuffer   = 0.20;
    static constexpr double kSubGoalLookahead  = 0.40;
    static constexpr double kMinPlanDistance   = 0.00;
    // Minimum clearance each intermediate path cell must have from any wall.
    // Paths that narrow below this value are rejected before the agent enters.
    // Set to agentRadius (0.40m) so the agent's body always fits at every
    // step along the planned route — no routing into funnels.

    static PedestrianPlanner& instance();

    bool initialize();
    bool isReady() const { return ready_; }

    // Returns smoothed world-space waypoints from start to goal, skipping the
    // start position itself. Returns empty if planning fails; caller falls back
    // to direct-to-waypoint behaviour.
    std::vector<Ped::Tvector> planPath(const Ped::Tvector& start,
                                        const Ped::Tvector& goal);

    // Hard wall constraint. If newPos is in an occupied cell, wall-slides back
    // toward oldPos (try X-only, Y-only, then full revert). Uses the A* grid
    // (0.55 m inflation) so the boundary matches the planning clearance.
    Ped::Tvector clampToFreeSpace(const Ped::Tvector& oldPos,
                                   const Ped::Tvector& newPos) const;

private:
    PedestrianPlanner() = default;
    PedestrianPlanner(const PedestrianPlanner&) = delete;
    PedestrianPlanner& operator=(const PedestrianPlanner&) = delete;

    bool loadMap(const std::string& yamlPath);
    bool loadImage(const std::string& imgPath, double occupiedThresh, bool negate);
    void inflateGrid(int cells);

    bool hasLineOfSight(int x0, int y0, int x1, int y1) const;
    std::vector<Ped::Tvector> astar(int sx, int sy, int gx, int gy) const;
    std::vector<Ped::Tvector> smoothPath(const std::vector<Ped::Tvector>& raw) const;

    std::pair<int,int> worldToGrid(double wx, double wy) const;
    Ped::Tvector       gridToWorld(int gx, int gy) const;
    bool               passable(int gx, int gy) const;
    bool               passableCollision(int gx, int gy) const;
    std::pair<int,int> snapToFree(int gx, int gy) const;
    std::pair<int,int> snapToFreeCollision(int gx, int gy) const;
    std::vector<Ped::Tvector> astarOnGrid(int sx, int sy, int gx, int gy,
                                           bool useCollision) const;

    std::vector<bool>  grid_;           // inflated by kPlanningInflation — used for A*
    std::vector<bool>  collisionGrid_;  // inflated by kCollisionBuffer   — physical hard stop
    int    width_      = 0;
    int    height_     = 0;
    double resolution_ = 0.1;
    double originX_    = 0.0;
    double originY_    = 0.0;
    bool   ready_      = false;
};
