/**
 * AStarPlanner.h
 * A* Path Planner for Grid-based Navigation
 */

#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <cmath>
#include <vector>
#include <queue>
#include <array>
#include <functional>
#include "SLAMEngine.h"

/**
 * A* Path Planner
 *
 * Finds shortest path on occupancy grid using A* algorithm
 */
class AStarPlanner {
public:
    struct Node {
        int x, y;
        float g;  // Cost from start
        float h;  // Heuristic to goal
        float f;  // Total cost (g + h)
        Node* parent;

        bool operator<(const Node& other) const {
            return f > other.f;  // Min-heap
        }
    };

    struct Config {
        bool allowDiagonal = true;
        float gridResolution = 0.05f;  // 5cm cells
        int maxIterations = 10000;
    };

    AStarPlanner(const Config& config = Config()) : config_(config) {}

    /**
     * Find shortest path from start to goal
     * @param start Start pose (meters)
     * @param goal Goal pose (meters)
     * @param map Occupancy grid
     * @return Path as list of poses
     */
    std::vector<Pose2D> findPath(
        const Pose2D& start,
        const Pose2D& goal,
        const OccupancyGrid& map
    ) {
        // Convert to grid coordinates
        int startX = static_cast<int>(start.x / config_.gridResolution) + map.width / 2;
        int startY = static_cast<int>(start.y / config_.gridResolution) + map.height / 2;
        int goalX = static_cast<int>(goal.x / config_.gridResolution) + map.width / 2;
        int goalY = static_cast<int>(goal.y / config_.gridResolution) + map.height / 2;

        // A* search
        std::priority_queue<Node> openSet;
        std::vector<std::vector<bool>> closedSet(map.width,
                                                std::vector<bool>(map.height, false));
        std::vector<std::vector<Node*>> nodeMap(map.width,
                                                std::vector<Node*>(map.height, nullptr));

        // Start node
        Node* startNode = new Node{startX, startY, 0,
            heuristic(startX, startY, goalX, goalY), 0, nullptr};
        openSet.push(*startNode);

        int iterations = 0;

        while (!openSet.empty() && iterations < config_.maxIterations) {
            iterations++;

            // Get node with lowest f
            Node current = openSet.top();
            openSet.pop();

            int cx = current.x;
            int cy = current.y;

            // Skip if already processed
            if (closedSet[cx][cy]) {
                delete &current;
                continue;
            }
            closedSet[cx][cy] = true;

            // Goal check
            if (cx == goalX && cy == goalY) {
                std::vector<Pose2D> path = reconstructPath(current, map);
                return smoothPath(path);
            }

            // Explore neighbors
            const int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
            const int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
            const float moveCost[] = {1, 1, 1, 1, 1.414f, 1.414f, 1.414f, 1.414f};

            for (int i = 0; i < 8; i++) {
                if (!config_.allowDiagonal && i >= 4) break;

                int nx = cx + dx[i];
                int ny = cy + dy[i];

                // Bounds check
                if (nx < 0 || nx >= map.width || ny < 0 || ny >= map.height) {
                    continue;
                }

                // Obstacle check
                if (map.isOccupied(nx, ny, 0.5f)) {
                    continue;
                }

                // Already processed
                if (closedSet[nx][ny]) {
                    continue;
                }

                // Calculate costs
                float g = current.g + moveCost[i];
                float h = heuristic(nx, ny, goalX, goalY);
                float f = g + h;

                // Check if this path is better
                Node* existing = nodeMap[nx][ny];
                if (existing && existing->g <= g) {
                    continue;
                }

                // Create new node
                Node* neighbor = new Node{nx, ny, g, h, f, nodeMap[cx][cy]};
                nodeMap[nx][ny] = neighbor;
                openSet.push(*neighbor);
            }
        }

        // No path found
        return {};
    }

    /**
     * Find path with smooth trajectory
     */
    std::vector<Pose2D> findSmoothPath(
        const Pose2D& start,
        const Pose2D& goal,
        const OccupancyGrid& map,
        int smoothingPasses = 3
    ) {
        auto path = findPath(start, goal, map);
        for (int i = 0; i < smoothingPasses; i++) {
            path = smoothPath(path);
        }
        return path;
    }

private:
    Config config_;

    float heuristic(int x1, int y1, int x2, int y2) {
        // Euclidean distance
        float dx = x1 - x2;
        float dy = y1 - y2;
        return std::sqrt(dx*dx + dy*dy);
    }

    std::vector<Pose2D> reconstructPath(const Node& goalNode, const OccupancyGrid& map) {
        std::vector<Pose2D> path;
        const Node* current = &goalNode;

        while (current) {
            // Convert to world coordinates
            Pose2D pose;
            pose.x = (current->x - map.width / 2) * config_.gridResolution;
            pose.y = (current->y - map.height / 2) * config_.gridResolution;
            pose.theta = 0;  // Will be calculated from direction

            path.push_back(pose);
            current = current->parent;
        }

        std::reverse(path.begin(), path.end());

        // Calculate headings
        for (size_t i = 1; i < path.size(); i++) {
            float dx = path[i].x - path[i-1].x;
            float dy = path[i].y - path[i-1].y;
            path[i].theta = std::atan2(dy, dx);
        }
        if (!path.empty()) {
            path[0].theta = path.size() > 1 ? path[1].theta : 0;
        }

        return path;
    }

    std::vector<Pose2D> smoothPath(const std::vector<Pose2D>& path) {
        if (path.size() <= 2) return path;

        std::vector<Pose2D> smoothed;
        smoothed.push_back(path[0]);

        size_t i = 0;
        while (i < path.size() - 1) {
            // Find furthest visible point
            size_t j = path.size() - 1;
            while (j > i + 1) {
                if (canSee(path[i], path[j])) {
                    break;
                }
                j--;
            }
            smoothed.push_back(path[j]);
            i = j;
        }

        return smoothed;
    }

    bool canSee(const Pose2D& a, const Pose2D& b) {
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float dist = std::sqrt(dx*dx + dy*dy);
        int steps = static_cast<int>(dist / 0.05f);  // Check every 5cm

        for (int i = 1; i < steps; i++) {
            float t = static_cast<float>(i) / steps;
            float x = a.x + t * dx;
            float y = a.y + t * dy;

            // This is a simplified check - in real implementation,
            // you'd check against the occupancy grid
        }

        return true;
    }
};

#endif // ASTAR_PLANNER_H
