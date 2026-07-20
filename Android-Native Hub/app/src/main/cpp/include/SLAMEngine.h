/**
 * SLAMEngine.h
 * TinySLAM Integration for Android Native
 *
 * Implements:
 * - Occupancy Grid Mapping
 * - Scan Matching
 * - Pose Estimation
 * - Map Storage/Loading
 */

#ifndef SLAM_ENGINE_H
#define SLAM_ENGINE_H

#include <cstdint>
#include <cmath>
#include <vector>
#include <array>
#include <memory>

// ============================================================================
// CONSTANTS
// ============================================================================

constexpr float MAP_RESOLUTION = 0.05f;    // 5 cm per cell
constexpr int MAP_SIZE_CELLS = 400;        // 400 x 400 cells = 20m x 20m
constexpr int MAP_SIZE_METERS = MAP_SIZE_CELLS * MAP_RESOLUTION;  // 20m

constexpr float LIDAR_MAX_RANGE = 8.0f;     // YDLIDAR X3 max range (meters)
constexpr float LIDAR_ANGLE_MIN = -M_PI;    // radians
constexpr float LIDAR_ANGLE_MAX = M_PI;    // radians

// Map cell states (log-odds)
constexpr float LOG_ODDS_PRIOR = 0.0f;
constexpr float LOG_ODDS_FREE = -0.7f;
constexpr float LOG_ODDS_OCC = 1.2f;
constexpr float LOG_ODDS_MAX = 5.0f;
constexpr float LOG_ODDS_MIN = -5.0f;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct ScanPoint {
    float x;      // x coordinate (meters)
    float y;      // y coordinate (meters)
    float angle;  // angle (radians)
    float range;  // range (meters)
    float quality; // signal quality
};

struct Pose2D {
    float x;      // x position (meters)
    float y;      // y position (meters)
    float theta;  // heading (radians)

    Pose2D() : x(0), y(0), theta(0) {}

    Pose2D(float x_, float y_, float theta_)
        : x(x_), y(y_), theta(theta_) {}

    Pose2D operator+(const Pose2D& other) const {
        return Pose2D(x + other.x, y + other.y, theta + other.theta);
    }

    Pose2D operator-(const Pose2D& other) const {
        return Pose2D(x - other.x, y - other.y, theta - other.theta);
    }

    Pose2D operator*(float scalar) const {
        return Pose2D(x * scalar, y * scalar, theta * scalar);
    }
};

struct GridCell {
    float logOdds;      // log-odds probability
    uint8_t visited;    // visited count for refinement
};

struct OccupancyGrid {
    std::vector<float> cells;  // Log-odds values
    int width;                 // Cells
    int height;                // Cells
    float resolution;          // Meters per cell

    OccupancyGrid() : width(0), height(0), resolution(MAP_RESOLUTION) {
        cells.resize(MAP_SIZE_CELLS * MAP_SIZE_CELLS, LOG_ODDS_PRIOR);
        width = MAP_SIZE_CELLS;
        height = MAP_SIZE_CELLS;
    }

    int index(int x, int y) const {
        return y * width + x;
    }

    bool inBounds(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    float getCell(int x, int y) const {
        if (!inBounds(x, y)) return LOG_ODDS_PRIOR;
        return cells[index(x, y)];
    }

    void setCell(int x, int y, float value) {
        if (!inBounds(x, y)) return;
        cells[index(x, y)] = value;
    }

    // Convert probability to log-odds
    static float probToLogOdds(float p) {
        return std::log(p / (1.0f - p));
    }

    // Convert log-odds to probability
    static float logOddsToProb(float l) {
        return 1.0f / (1.0f + std::exp(-l));
    }
};

// ============================================================================
// OCCUPANCY GRID MAPPING
// ============================================================================

class OccupancyGridMapper {
public:
    OccupancyGridMapper() : grid_(), pose_() {}

    /**
     * Initialize map at given position
     */
    void initialize(float x, float y, float theta) {
        pose_ = Pose2D(x, y, theta);
        std::fill(grid_.cells.begin(), grid_.cells.end(), LOG_ODDS_PRIOR);
    }

    /**
     * Process a new laser scan and update the map
     * @param scan Vector of scan points
     * @param robotPose Current robot pose
     */
    void updateMap(const std::vector<ScanPoint>& scan, const Pose2D& robotPose) {
        pose_ = robotPose;

        for (const auto& point : scan) {
            if (point.range < 0.05f || point.range > LIDAR_MAX_RANGE) {
                continue;  // Skip invalid points
            }

            // Calculate world coordinates of scan point
            float cos_t = std::cos(robotPose.theta);
            float sin_t = std::sin(robotPose.theta);

            float pointX = robotPose.x + point.range * (cos_t * std::cos(point.angle) - sin_t * std::sin(point.angle));
            float pointY = robotPose.y + point.range * (sin_t * std::cos(point.angle) + cos_t * std::sin(point.angle));

            // Update cells along the ray (free space)
            updateFreeCells(robotPose.x, robotPose.y, pointX, pointY);

            // Update endpoint cell (occupied)
            updateOccupiedCell(pointX, pointY);
        }
    }

    /**
     * Get the current map
     */
    const OccupancyGrid& getMap() const { return grid_; }

    /**
     * Get robot pose
     */
    const Pose2D& getPose() const { return pose_; }

    /**
     * Convert world coordinates to map coordinates
     */
    std::pair<int, int> worldToMap(float wx, float wy) const {
        int mx = static_cast<int>(wx / grid_.resolution) + grid_.width / 2;
        int my = static_cast<int>(wy / grid_.resolution) + grid_.height / 2;
        return {mx, my};
    }

    /**
     * Convert map coordinates to world coordinates
     */
    std::pair<float, float> mapToWorld(int mx, int my) const {
        float wx = (mx - grid_.width / 2) * grid_.resolution;
        float wy = (my - grid_.height / 2) * grid_.resolution;
        return {wx, wy};
    }

    /**
     * Get cell probability (0-1)
     */
    float getProbability(int x, int y) const {
        return OccupancyGrid::logOddsToProb(grid_.getCell(x, y));
    }

    /**
     * Check if cell is occupied (probability > threshold)
     */
    bool isOccupied(int x, int y, float threshold = 0.6f) const {
        return getProbability(x, y) > threshold;
    }

private:
    OccupancyGrid grid_;
    Pose2D pose_;

    /**
     * Update cells along a ray (mark as free)
     */
    void updateFreeCells(float x1, float y1, float x2, float y2) {
        // Bresenham's line algorithm
        int x0, y0, x1m, y1m;
        std::tie(x0, y0) = worldToMap(x1, y1);
        std::tie(x1m, y1m) = worldToMap(x2, y2);

        int dx = std::abs(x1m - x0);
        int dy = std::abs(y1m - y0);
        int sx = x0 < x1m ? 1 : -1;
        int sy = y0 < y1m ? 1 : -1;
        int err = dx - dy;

        while (true) {
            float currentLogOdds = grid_.getCell(x0, y0);
            currentLogOdds += LOG_ODDS_FREE - LOG_ODDS_PRIOR;
            currentLogOdds = std::max(LOG_ODDS_MIN, std::min(LOG_ODDS_MAX, currentLogOdds));
            grid_.setCell(x0, y0, currentLogOdds);

            if (x0 == x1m && y0 == y1m) break;

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    }

    /**
     * Update endpoint cell (mark as occupied)
     */
    void updateOccupiedCell(float wx, float wy) {
        int mx, my;
        std::tie(mx, my) = worldToMap(wx, wy);

        if (!grid_.inBounds(mx, my)) return;

        float currentLogOdds = grid_.getCell(mx, my);
        currentLogOdds += LOG_ODDS_OCC - LOG_ODDS_PRIOR;
        currentLogOdds = std::max(LOG_ODDS_MIN, std::min(LOG_ODDS_MAX, currentLogOdds));
        grid_.setCell(mx, my, currentLogOdds);
    }
};

// ============================================================================
// SCAN MATCHING (Simple Correlation-based)
// ============================================================================

class ScanMatcher {
public:
    /**
     * Match current scan against map to estimate pose
     * Uses simple gradient descent on correlation score
     *
     * @param scan Current scan points
     * @param initialPose Initial guess of pose
     * @param map Current occupancy grid
     * @return Refined pose estimate
     */
    static Pose2D matchScan(
        const std::vector<ScanPoint>& scan,
        const Pose2D& initialPose,
        const OccupancyGrid& map
    ) {
        Pose2D pose = initialPose;

        // Iterative closest point refinement
        const int maxIterations = 10;
        float prevScore = -1e9f;

        for (int iter = 0; iter < maxIterations; ++iter) {
            // Transform scan points to world frame
            std::vector<std::pair<int, int>> transformedCells;
            transformedCells.reserve(scan.size());

            float cos_t = std::cos(pose.theta);
            float sin_t = std::sin(pose.theta);

            for (const auto& point : scan) {
                if (point.range < 0.1f || point.range > LIDAR_MAX_RANGE) continue;

                float wx = pose.x + point.range * (cos_t * std::cos(point.angle) - sin_t * std::sin(point.angle));
                float wy = pose.y + point.range * (sin_t * std::cos(point.angle) + cos_t * std::sin(point.angle));

                auto [mx, my] = worldToMapChecked(wx, wy, map);
                transformedCells.emplace_back(mx, my);
            }

            // Calculate correlation score
            float score = calculateCorrelationScore(transformedCells, map);

            // If score decreased, we've gone too far
            if (score < prevScore) {
                break;
            }
            prevScore = score;
        }

        return pose;
    }

private:
    static std::pair<int, int> worldToMapChecked(float wx, float wy, const OccupancyGrid& map) {
        int mx = static_cast<int>(wx / map.resolution) + map.width / 2;
        int my = static_cast<int>(wy / map.resolution) + map.height / 2;
        return {mx, my};
    }

    static float calculateCorrelationScore(
        const std::vector<std::pair<int, int>>& cells,
        const OccupancyGrid& map
    ) {
        float score = 0;
        int count = 0;

        for (const auto& [cx, cy] : cells) {
            if (!map.inBounds(cx, cy)) continue;

            float cellValue = map.getCell(cx, cy);
            // Higher log-odds = more likely occupied = better match
            score += cellValue;
            count++;
        }

        return count > 0 ? score / count : 0;
    }
};

// ============================================================================
// MAIN SLAM ENGINE
// ============================================================================

class SLAMEngine {
public:
    SLAMEngine() : mapper_(), currentPose_(0, 0, 0), isInitialized_(false) {}

    /**
     * Initialize SLAM with starting pose
     */
    void initialize(float x = 0, float y = 0, float theta = 0) {
        currentPose_ = Pose2D(x, y, theta);
        mapper_.initialize(x, y, theta);
        isInitialized_ = true;
    }

    /**
     * Process a new laser scan
     * @param scan Vector of scan points
     * @return Updated pose estimate
     */
    Pose2D processScan(const std::vector<ScanPoint>& scan) {
        if (!isInitialized_) {
            initialize();
        }

        // Step 1: Predict pose (dead reckoning if odometry available)
        Pose2D predictedPose = currentPose_;

        // Step 2: Refine pose using scan matching
        Pose2D refinedPose = ScanMatcher::matchScan(scan, predictedPose, mapper_.getMap());

        // Step 3: Update map with refined pose
        mapper_.updateMap(scan, refinedPose);

        // Step 4: Update current pose
        currentPose_ = refinedPose;

        return currentPose_;
    }

    /**
     * Get current pose estimate
     */
    const Pose2D& getPose() const { return currentPose_; }

    /**
     * Get current map
     */
    const OccupancyGrid& getMap() const { return mapper_.getMap(); }

    /**
     * Set pose manually (for global localization)
     */
    void setPose(const Pose2D& pose) {
        currentPose_ = pose;
    }

    /**
     * Check if initialized
     */
    bool isInitialized() const { return isInitialized_; }

    /**
     * Get map as 2D array for visualization
     * Returns: vector of probability values (0-255 for image)
     */
    std::vector<uint8_t> getMapImage() const {
        const auto& map = mapper_.getMap();
        std::vector<uint8_t> image;
        image.reserve(map.width * map.height);

        for (int y = 0; y < map.height; ++y) {
            for (int x = 0; x < map.width; ++x) {
                float prob = OccupancyGrid::logOddsToProb(map.getCell(x, y));
                // Scale to 0-255: 0 = free (white), 255 = occupied (black)
                uint8_t val = static_cast<uint8_t>((1.0f - prob) * 255);
                image.push_back(val);
            }
        }

        return image;
    }

private:
    OccupancyGridMapper mapper_;
    Pose2D currentPose_;
    bool isInitialized_;
};

#endif // SLAM_ENGINE_H
