/**
 * PurePursuit.h
 * Path Tracking Controller for 4WD Differential Drive (non-holonomic)
 *
 * Project dùng 4WD bánh thường (không mecanum). Controller chỉ output
 * (vx, omega), vy luôn = 0.
 */

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <cmath>
#include <vector>
#include "SLAMEngine.h"
#include "RobotMotorCommand.h"

/**
 * Pure Pursuit Controller
 *
 * Algorithm:
 * 1. Find lookahead point on path
 * 2. Compute curvature to lookahead point
 * 3. Generate velocity command
 */
class PurePursuitController {
public:
    struct Config {
        float lookaheadDist = 0.5f;      // meters
        float minLookaheadDist = 0.3f;  // meters
        float maxLookaheadDist = 1.0f;  // meters
        float lookaheadGain = 0.5f;     // velocity-based lookahead
        float targetSpeed = 0.3f;       // m/s
        float maxAngularVel = 3.0f;      // rad/s
        float wheelbase = 0.25f;         // meters (L)
    };

    PurePursuitController(const Config& config = Config()) : config_(config) {}

    /**
     * Compute velocity command to follow path
     * @param path Path points to follow
     * @param currentPose Current robot pose
     * @param currentSpeed Current forward speed (m/s)
     * @return Velocity command
     */
    VelocityCommand computeCommand(
        const std::vector<Pose2D>& path,
        const Pose2D& currentPose,
        float currentSpeed = 0.0f
    ) {
        VelocityCommand cmd;
        cmd.timestamp_ms = 0;

        if (path.empty()) {
            cmd.vx = 0;
            cmd.vy = 0;
            cmd.omega = 0;
            return cmd;
        }

        // Calculate dynamic lookahead distance
        float lookahead = config_.lookaheadDist;
        lookahead = std::max(config_.minLookaheadDist,
                            std::min(config_.maxLookaheadDist,
                                    config_.lookaheadGain * std::abs(currentSpeed)));

        // Find lookahead point
        auto lookaheadPoint = findLookaheadPoint(path, currentPose, lookahead);

        if (!lookaheadPoint.has_value()) {
            // Path ended, stop
            cmd.vx = 0;
            cmd.vy = 0;
            cmd.omega = 0;
            return cmd;
        }

        Pose2D target = lookaheadPoint.value();

        // Transform target to robot local frame
        float dx = target.x - currentPose.x;
        float dy = target.y - currentPose.y;

        float cos_t = std::cos(currentPose.theta);
        float sin_t = std::sin(currentPose.theta);

        float localX = cos_t * dx + sin_t * dy;
        float localY = -sin_t * dx + cos_t * dy;

        // Calculate distance to lookahead
        float L = std::sqrt(localX * localX + localY * localY);
        if (L < 0.001f) {
            L = 0.001f;
        }

        // Pure pursuit curvature
        // kappa = 2*ly / L^2
        float kappa = 2.0f * localY / (L * L);

        // Angular velocity
        float speed = (currentSpeed > 0) ? currentSpeed : config_.targetSpeed;
        cmd.omega = kappa * speed;

        // Clamp angular velocity
        cmd.omega = std::max(-config_.maxAngularVel,
                            std::min(config_.maxAngularVel, cmd.omega));

        // Forward velocity (maintain current or target speed)
        cmd.vx = speed * std::cos(cmd.omega * 0.1f);  // Slight speed reduction on turns
        cmd.vy = 0;  // 4WD differential: không có lateral (vy = 0 luôn)

        return cmd;
    }

    /**
     * Get distance to goal
     */
    float distanceToGoal(const std::vector<Pose2D>& path, const Pose2D& pose) {
        if (path.empty()) return 0;
        const auto& goal = path.back();
        float dx = goal.x - pose.x;
        float dy = goal.y - pose.y;
        return std::sqrt(dx*dx + dy*dy);
    }

    /**
     * Check if goal reached
     */
    bool isGoalReached(const std::vector<Pose2D>& path,
                      const Pose2D& pose,
                      float threshold = 0.2f) {
        return distanceToGoal(path, pose) < threshold;
    }

private:
    Config config_;

    std::optional<Pose2D> findLookaheadPoint(
        const std::vector<Pose2D>& path,
        const Pose2D& pose,
        float lookaheadDist
    ) {
        if (path.empty()) return std::nullopt;

        // Find closest point
        int closestIdx = 0;
        float minDist = 1e9f;
        float accumDist = 0;

        for (int i = 0; i < (int)path.size() - 1; i++) {
            float dx = path[i].x - pose.x;
            float dy = path[i].y - pose.y;
            float dist = std::sqrt(dx*dx + dy*dy);

            if (dist < minDist) {
                minDist = dist;
                closestIdx = i;
            }

            // Accumulate distance along path
            if (i > 0) {
                dx = path[i].x - path[i-1].x;
                dy = path[i].y - path[i-1].y;
                accumDist += std::sqrt(dx*dx + dy*dy);
            }
        }

        // Find point at lookahead distance
        float targetDist = accumDist + lookaheadDist;
        accumDist = 0;

        for (int i = 1; i < (int)path.size(); i++) {
            float dx = path[i].x - path[i-1].x;
            float dy = path[i].y - path[i-1].y;
            float segmentLen = std::sqrt(dx*dx + dy*dy);

            if (accumDist + segmentLen >= targetDist) {
                // Interpolate
                float t = (targetDist - accumDist) / segmentLen;
                Pose2D result;
                result.x = path[i-1].x + t * dx;
                result.y = path[i-1].y + t * dy;
                result.theta = path[i].theta;
                return result;
            }
            accumDist += segmentLen;
        }

        // Return last point if path shorter than lookahead
        return path.back();
    }
};

#endif // PURE_PURSUIT_H
