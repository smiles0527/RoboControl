/**
 * ControlWorkbench Path Generation Library
 * Version: 1.0.0
 * 
 * Path generation for autonomous:
 * - Hermite spline paths
 * - Bezier curves
 * - Velocity profiling
 * - Path injection
 * - Turn radius limiting
 * 
 * Copyright (c) 2024 ControlWorkbench
 * MIT License
 */

#pragma once

#include "telemetry.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace cwb {

// =============================================================================
// PATH POINT
// =============================================================================

/**
 * A point along a path with position, velocity, and curvature.
 */
struct PathPoint {
    double x;
    double y;
    double heading;      // radians
    double velocity;     // in/sec
    double curvature;    // 1/inches
    double distance;     // distance from start
    
    PathPoint() : x(0), y(0), heading(0), velocity(0), curvature(0), distance(0) {}
    PathPoint(double x, double y) : x(x), y(y), heading(0), velocity(0), curvature(0), distance(0) {}
    PathPoint(double x, double y, double h) : x(x), y(y), heading(h), velocity(0), curvature(0), distance(0) {}
    
    double distance_to(const PathPoint& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

// =============================================================================
// PATH CONSTRAINTS
// =============================================================================

/**
 * Constraints for path generation.
 */
struct PathConstraints {
    double max_velocity = 60.0;       // in/sec
    double max_acceleration = 120.0;  // in/sec²
    double max_deceleration = 120.0;  // in/sec²
    double max_centripetal_accel = 60.0;  // in/sec²
    double track_width = 12.0;        // inches
    double point_spacing = 1.0;       // inches between path points
    
    // Limit velocity based on curvature
    double curvature_velocity(double curvature) const {
        if (std::abs(curvature) < 0.001) return max_velocity;
        // v = sqrt(a_c / curvature)
        return std::min(max_velocity, std::sqrt(max_centripetal_accel / std::abs(curvature)));
    }
};

// =============================================================================
// PATH GENERATOR
// =============================================================================

/**
 * Generate smooth paths from waypoints.
 * 
 * Usage:
 *   cwb::PathGenerator gen;
 *   gen.set_constraints(constraints);
 *   
 *   std::vector<cwb::Waypoint> waypoints = {
 *       {0, 0}, {24, 24}, {48, 24}
 *   };
 *   
 *   auto path = gen.generate(waypoints);
 *   chassis.follow_path(path);
 */
class PathGenerator {
public:
    enum class SplineType { Cubic, Quintic };
    
    PathGenerator();
    
    /**
     * Set path constraints.
     */
    void set_constraints(const PathConstraints& constraints) { constraints_ = constraints; }
    
    /**
     * Set spline type.
     */
    void set_spline_type(SplineType type) { spline_type_ = type; }
    
    /**
     * Generate path from waypoints.
     */
    std::vector<PathPoint> generate(const std::vector<Waypoint>& waypoints);
    
    /**
     * Generate path with explicit headings at each waypoint.
     */
    std::vector<PathPoint> generate_with_headings(const std::vector<PathPoint>& waypoints);
    
    /**
     * Generate a simple straight line path.
     */
    std::vector<PathPoint> straight_line(double x1, double y1, double x2, double y2);
    
    /**
     * Generate an arc path.
     */
    std::vector<PathPoint> arc(double cx, double cy, double radius, 
                               double start_angle, double end_angle);
    
    /**
     * Inject points along a path (increase density).
     */
    std::vector<PathPoint> inject_points(const std::vector<PathPoint>& path, double spacing);
    
    /**
     * Smooth a path using gradient descent.
     */
    std::vector<PathPoint> smooth_path(const std::vector<PathPoint>& path, 
                                        double weight_data = 0.5, 
                                        double weight_smooth = 0.3,
                                        double tolerance = 0.001);
    
    /**
     * Calculate curvature at each point.
     */
    void calculate_curvatures(std::vector<PathPoint>& path);
    
    /**
     * Calculate velocity profile.
     */
    void calculate_velocities(std::vector<PathPoint>& path);

private:
    PathConstraints constraints_;
    SplineType spline_type_ = SplineType::Cubic;
    
    // Hermite spline interpolation
    PathPoint hermite_point(const PathPoint& p0, const PathPoint& p1, 
                           const PathPoint& m0, const PathPoint& m1, double t);
    
    // Calculate tangent for a waypoint
    PathPoint calculate_tangent(const std::vector<Waypoint>& waypoints, size_t index);
};

// =============================================================================
// BEZIER CURVE
// =============================================================================

/**
 * Bezier curve path generation.
 */
class BezierPath {
public:
    /**
     * Generate a cubic Bezier curve.
     * @param p0 Start point
     * @param p1 Control point 1
     * @param p2 Control point 2
     * @param p3 End point
     * @param num_points Number of points to generate
     */
    static std::vector<PathPoint> cubic(
        const PathPoint& p0, const PathPoint& p1,
        const PathPoint& p2, const PathPoint& p3,
        int num_points = 50);
    
    /**
     * Generate a quadratic Bezier curve.
     */
    static std::vector<PathPoint> quadratic(
        const PathPoint& p0, const PathPoint& p1, const PathPoint& p2,
        int num_points = 50);
    
    /**
     * Calculate control points for smooth transition between waypoints.
     */
    static std::pair<PathPoint, PathPoint> calculate_control_points(
        const PathPoint& prev, const PathPoint& current, const PathPoint& next,
        double smoothness = 0.5);
};

// =============================================================================
// PATH UTILITIES
// =============================================================================

/**
 * Path utility functions.
 */
class PathUtils {
public:
    /**
     * Mirror a path for opposite alliance.
     * @param flip_x If true, mirror across Y axis
     * @param flip_y If true, mirror across X axis
     */
    static std::vector<PathPoint> mirror(const std::vector<PathPoint>& path, 
                                          bool flip_x = true, bool flip_y = false);
    
    /**
     * Translate a path.
     */
    static std::vector<PathPoint> translate(const std::vector<PathPoint>& path,
                                             double dx, double dy);
    
    /**
     * Rotate a path around a point.
     */
    static std::vector<PathPoint> rotate(const std::vector<PathPoint>& path,
                                          double angle, double cx = 0, double cy = 0);
    
    /**
     * Reverse a path (for driving backwards).
     */
    static std::vector<PathPoint> reverse(const std::vector<PathPoint>& path);
    
    /**
     * Combine multiple paths.
     */
    static std::vector<PathPoint> combine(const std::vector<std::vector<PathPoint>>& paths);
    
    /**
     * Get total path length.
     */
    static double total_length(const std::vector<PathPoint>& path);
    
    /**
     * Find closest point on path to a position.
     */
    static size_t find_closest(const std::vector<PathPoint>& path, double x, double y);
    
    /**
     * Calculate estimated time to complete path.
     */
    static double estimated_time(const std::vector<PathPoint>& path);
};

// =============================================================================
// IMPLEMENTATION
// =============================================================================

#ifdef CWB_IMPLEMENTATION

// PathGenerator
PathGenerator::PathGenerator() {}

std::vector<PathPoint> PathGenerator::generate(const std::vector<Waypoint>& waypoints) {
    if (waypoints.size() < 2) return {};
    
    // Convert waypoints to path points
    std::vector<PathPoint> control_points;
    for (const auto& wp : waypoints) {
        control_points.push_back({wp.x, wp.y, std::isnan(wp.heading) ? 0 : wp.heading});
    }
    
    // Calculate headings if not specified
    for (size_t i = 0; i < control_points.size(); i++) {
        if (i == 0) {
            double dx = control_points[1].x - control_points[0].x;
            double dy = control_points[1].y - control_points[0].y;
            control_points[0].heading = std::atan2(dx, dy);
        } else if (i == control_points.size() - 1) {
            double dx = control_points[i].x - control_points[i-1].x;
            double dy = control_points[i].y - control_points[i-1].y;
            control_points[i].heading = std::atan2(dx, dy);
        } else {
            double dx = control_points[i+1].x - control_points[i-1].x;
            double dy = control_points[i+1].y - control_points[i-1].y;
            control_points[i].heading = std::atan2(dx, dy);
        }
    }
    
    return generate_with_headings(control_points);
}

std::vector<PathPoint> PathGenerator::generate_with_headings(const std::vector<PathPoint>& waypoints) {
    if (waypoints.size() < 2) return {};
    
    std::vector<PathPoint> path;
    
    // Generate spline between each pair of waypoints
    for (size_t i = 0; i < waypoints.size() - 1; i++) {
        const auto& p0 = waypoints[i];
        const auto& p1 = waypoints[i + 1];
        
        double dist = p0.distance_to(p1);
        int num_points = static_cast<int>(dist / constraints_.point_spacing) + 1;
        
        // Tangent magnitudes (proportional to distance)
        double mag = dist * 0.5;
        
        PathPoint m0 = {mag * std::sin(p0.heading), mag * std::cos(p0.heading)};
        PathPoint m1 = {mag * std::sin(p1.heading), mag * std::cos(p1.heading)};
        
        for (int j = 0; j < num_points; j++) {
            double t = static_cast<double>(j) / num_points;
            auto point = hermite_point(p0, p1, m0, m1, t);
            path.push_back(point);
        }
    }
    
    // Add final point
    path.push_back(waypoints.back());
    
    // Calculate curvatures and velocities
    calculate_curvatures(path);
    calculate_velocities(path);
    
    return path;
}

PathPoint PathGenerator::hermite_point(const PathPoint& p0, const PathPoint& p1,
                                        const PathPoint& m0, const PathPoint& m1, double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    
    // Hermite basis functions
    double h00 = 2*t3 - 3*t2 + 1;
    double h10 = t3 - 2*t2 + t;
    double h01 = -2*t3 + 3*t2;
    double h11 = t3 - t2;
    
    PathPoint result;
    result.x = h00 * p0.x + h10 * m0.x + h01 * p1.x + h11 * m1.x;
    result.y = h00 * p0.y + h10 * m0.y + h01 * p1.y + h11 * m1.y;
    
    // Calculate heading from tangent
    double dx = (6*t2 - 6*t) * p0.x + (3*t2 - 4*t + 1) * m0.x + 
                (-6*t2 + 6*t) * p1.x + (3*t2 - 2*t) * m1.x;
    double dy = (6*t2 - 6*t) * p0.y + (3*t2 - 4*t + 1) * m0.y + 
                (-6*t2 + 6*t) * p1.y + (3*t2 - 2*t) * m1.y;
    result.heading = std::atan2(dx, dy);
    
    return result;
}

std::vector<PathPoint> PathGenerator::straight_line(double x1, double y1, double x2, double y2) {
    std::vector<PathPoint> path;
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dist = std::sqrt(dx * dx + dy * dy);
    double heading = std::atan2(dx, dy);
    
    int num_points = static_cast<int>(dist / constraints_.point_spacing) + 1;
    
    for (int i = 0; i <= num_points; i++) {
        double t = static_cast<double>(i) / num_points;
        PathPoint p;
        p.x = x1 + dx * t;
        p.y = y1 + dy * t;
        p.heading = heading;
        p.curvature = 0;
        p.distance = dist * t;
        p.velocity = constraints_.max_velocity;
        path.push_back(p);
    }
    
    return path;
}

std::vector<PathPoint> PathGenerator::arc(double cx, double cy, double radius,
                                           double start_angle, double end_angle) {
    std::vector<PathPoint> path;
    
    double arc_length = std::abs(end_angle - start_angle) * radius;
    int num_points = static_cast<int>(arc_length / constraints_.point_spacing) + 1;
    
    for (int i = 0; i <= num_points; i++) {
        double t = static_cast<double>(i) / num_points;
        double angle = start_angle + (end_angle - start_angle) * t;
        
        PathPoint p;
        p.x = cx + radius * std::cos(angle);
        p.y = cy + radius * std::sin(angle);
        p.heading = angle + M_PI / 2;  // Tangent to circle
        if (end_angle < start_angle) p.heading -= M_PI;
        p.curvature = 1.0 / radius;
        p.distance = arc_length * t;
        path.push_back(p);
    }
    
    calculate_velocities(path);
    return path;
}

std::vector<PathPoint> PathGenerator::inject_points(const std::vector<PathPoint>& path, double spacing) {
    if (path.size() < 2) return path;
    
    std::vector<PathPoint> result;
    result.push_back(path[0]);
    
    for (size_t i = 1; i < path.size(); i++) {
        double dist = path[i-1].distance_to(path[i]);
        int num_inject = static_cast<int>(dist / spacing);
        
        for (int j = 1; j <= num_inject; j++) {
            double t = static_cast<double>(j) / (num_inject + 1);
            PathPoint p;
            p.x = path[i-1].x + (path[i].x - path[i-1].x) * t;
            p.y = path[i-1].y + (path[i].y - path[i-1].y) * t;
            p.heading = path[i-1].heading + (path[i].heading - path[i-1].heading) * t;
            result.push_back(p);
        }
        
        result.push_back(path[i]);
    }
    
    return result;
}

std::vector<PathPoint> PathGenerator::smooth_path(const std::vector<PathPoint>& path,
                                                   double weight_data, double weight_smooth,
                                                   double tolerance) {
    if (path.size() < 3) return path;
    
    std::vector<PathPoint> smooth = path;
    double change = tolerance;
    
    while (change >= tolerance) {
        change = 0;
        
        for (size_t i = 1; i < path.size() - 1; i++) {
            double x_old = smooth[i].x;
            double y_old = smooth[i].y;
            
            smooth[i].x += weight_data * (path[i].x - smooth[i].x) +
                          weight_smooth * (smooth[i-1].x + smooth[i+1].x - 2 * smooth[i].x);
            smooth[i].y += weight_data * (path[i].y - smooth[i].y) +
                          weight_smooth * (smooth[i-1].y + smooth[i+1].y - 2 * smooth[i].y);
            
            change += std::abs(smooth[i].x - x_old) + std::abs(smooth[i].y - y_old);
        }
    }
    
    // Recalculate headings after smoothing
    for (size_t i = 0; i < smooth.size(); i++) {
        if (i < smooth.size() - 1) {
            double dx = smooth[i+1].x - smooth[i].x;
            double dy = smooth[i+1].y - smooth[i].y;
            smooth[i].heading = std::atan2(dx, dy);
        } else {
            smooth[i].heading = smooth[i-1].heading;
        }
    }
    
    return smooth;
}

void PathGenerator::calculate_curvatures(std::vector<PathPoint>& path) {
    if (path.size() < 3) return;
    
    path[0].curvature = 0;
    path.back().curvature = 0;
    
    for (size_t i = 1; i < path.size() - 1; i++) {
        const auto& p1 = path[i-1];
        const auto& p2 = path[i];
        const auto& p3 = path[i+1];
        
        double x1 = p1.x, y1 = p1.y;
        double x2 = p2.x, y2 = p2.y;
        double x3 = p3.x, y3 = p3.y;
        
        // Use Menger curvature formula: ? = 4*A / (|p1-p2| * |p2-p3| * |p3-p1|)
        // where A is the signed area of the triangle
        double a = std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));  // |p1-p2|
        double b = std::sqrt((x3-x2)*(x3-x2) + (y3-y2)*(y3-y2));  // |p2-p3|
        double c = std::sqrt((x1-x3)*(x1-x3) + (y1-y3)*(y1-y3));  // |p3-p1|
        
        // Signed area using cross product: A = 0.5 * |((p2-p1) × (p3-p1))|
        double cross = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        double area = std::abs(cross) * 0.5;
        
        // Curvature = 4*A / (a*b*c)
        double denom = a * b * c;
        if (denom > 1e-10) {
            path[i].curvature = 4.0 * area / denom;
        } else {
            path[i].curvature = 0;  // Collinear or coincident points
        }
    }
}

void PathGenerator::calculate_velocities(std::vector<PathPoint>& path) {
    if (path.empty()) return;
    
    // Calculate distance from start
    path[0].distance = 0;
    for (size_t i = 1; i < path.size(); i++) {
        path[i].distance = path[i-1].distance + path[i-1].distance_to(path[i]);
    }
    
    // Forward pass: limit by curvature and acceleration
    path[0].velocity = 0;  // Start from stop
    for (size_t i = 1; i < path.size(); i++) {
        double max_v = constraints_.curvature_velocity(path[i].curvature);
        double dist = path[i].distance - path[i-1].distance;
        double accel_limit = std::sqrt(path[i-1].velocity * path[i-1].velocity + 
                                       2 * constraints_.max_acceleration * dist);
        path[i].velocity = std::min({max_v, accel_limit, constraints_.max_velocity});
    }
    
    // Backward pass: limit by deceleration
    path.back().velocity = 0;  // End at stop
    for (int i = static_cast<int>(path.size()) - 2; i >= 0; i--) {
        double dist = path[i+1].distance - path[i].distance;
        double decel_limit = std::sqrt(path[i+1].velocity * path[i+1].velocity + 
                                       2 * constraints_.max_deceleration * dist);
        path[i].velocity = std::min(path[i].velocity, decel_limit);
    }
}

// BezierPath
std::vector<PathPoint> BezierPath::cubic(const PathPoint& p0, const PathPoint& p1,
                                          const PathPoint& p2, const PathPoint& p3,
                                          int num_points) {
    std::vector<PathPoint> path;
    
    for (int i = 0; i <= num_points; i++) {
        double t = static_cast<double>(i) / num_points;
        double t2 = t * t;
        double t3 = t2 * t;
        double mt = 1 - t;
        double mt2 = mt * mt;
        double mt3 = mt2 * mt;
        
        PathPoint p;
        p.x = mt3 * p0.x + 3 * mt2 * t * p1.x + 3 * mt * t2 * p2.x + t3 * p3.x;
        p.y = mt3 * p0.y + 3 * mt2 * t * p1.y + 3 * mt * t2 * p2.y + t3 * p3.y;
        
        // Calculate tangent for heading
        double dx = 3 * mt2 * (p1.x - p0.x) + 6 * mt * t * (p2.x - p1.x) + 3 * t2 * (p3.x - p2.x);
        double dy = 3 * mt2 * (p1.y - p0.y) + 6 * mt * t * (p2.y - p1.y) + 3 * t2 * (p3.y - p2.y);
        p.heading = std::atan2(dx, dy);
        
        path.push_back(p);
    }
    
    return path;
}

std::vector<PathPoint> BezierPath::quadratic(const PathPoint& p0, const PathPoint& p1,
                                              const PathPoint& p2, int num_points) {
    std::vector<PathPoint> path;
    
    for (int i = 0; i <= num_points; i++) {
        double t = static_cast<double>(i) / num_points;
        double mt = 1 - t;
        
        PathPoint p;
        p.x = mt * mt * p0.x + 2 * mt * t * p1.x + t * t * p2.x;
        p.y = mt * mt * p0.y + 2 * mt * t * p1.y + t * t * p2.y;
        
        double dx = 2 * mt * (p1.x - p0.x) + 2 * t * (p2.x - p1.x);
        double dy = 2 * mt * (p1.y - p0.y) + 2 * t * (p2.y - p1.y);
        p.heading = std::atan2(dx, dy);
        
        path.push_back(p);
    }
    
    return path;
}

std::pair<PathPoint, PathPoint> BezierPath::calculate_control_points(
    const PathPoint& prev, const PathPoint& current, const PathPoint& next, double smoothness) {
    
    double dx = next.x - prev.x;
    double dy = next.y - prev.y;
    double d = std::sqrt(dx * dx + dy * dy);
    
    double d1 = current.distance_to(prev);
    double d2 = current.distance_to(next);
    
    double scale1 = smoothness * d1 / d;
    double scale2 = smoothness * d2 / d;
    
    PathPoint c1, c2;
    c1.x = current.x - dx * scale1;
    c1.y = current.y - dy * scale1;
    c2.x = current.x + dx * scale2;
    c2.y = current.y + dy * scale2;
    
    return {c1, c2};
}

// PathUtils
std::vector<PathPoint> PathUtils::mirror(const std::vector<PathPoint>& path,
                                          bool flip_x, bool flip_y) {
    std::vector<PathPoint> result = path;
    
    for (auto& p : result) {
        if (flip_x) {
            p.x = 144 - p.x;  // VRC field is 144" x 144"
            p.heading = -p.heading;
        }
        if (flip_y) {
            p.y = 144 - p.y;
            p.heading = M_PI - p.heading;
        }
    }
    
    return result;
}

std::vector<PathPoint> PathUtils::translate(const std::vector<PathPoint>& path,
                                             double dx, double dy) {
    std::vector<PathPoint> result = path;
    for (auto& p : result) {
        p.x += dx;
        p.y += dy;
    }
    return result;
}

std::vector<PathPoint> PathUtils::rotate(const std::vector<PathPoint>& path,
                                          double angle, double cx, double cy) {
    std::vector<PathPoint> result = path;
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);
    
    for (auto& p : result) {
        double x = p.x - cx;
        double y = p.y - cy;
        p.x = x * cos_a - y * sin_a + cx;
        p.y = x * sin_a + y * cos_a + cy;
        p.heading += angle;
    }
    
    return result;
}

std::vector<PathPoint> PathUtils::reverse(const std::vector<PathPoint>& path) {
    std::vector<PathPoint> result = path;
    std::reverse(result.begin(), result.end());
    
    // Flip headings
    for (auto& p : result) {
        p.heading += M_PI;
        while (p.heading > M_PI) p.heading -= 2 * M_PI;
    }
    
    // Recalculate distance
    if (!result.empty()) {
        result[0].distance = 0;
        for (size_t i = 1; i < result.size(); i++) {
            result[i].distance = result[i-1].distance + result[i-1].distance_to(result[i]);
        }
    }
    
    return result;
}

std::vector<PathPoint> PathUtils::combine(const std::vector<std::vector<PathPoint>>& paths) {
    std::vector<PathPoint> result;
    double total_dist = 0;
    
    for (const auto& path : paths) {
        for (auto p : path) {
            p.distance += total_dist;
            result.push_back(p);
        }
        if (!path.empty()) {
            total_dist = result.back().distance;
        }
    }
    
    return result;
}

double PathUtils::total_length(const std::vector<PathPoint>& path) {
    if (path.empty()) return 0;
    return path.back().distance;
}

size_t PathUtils::find_closest(const std::vector<PathPoint>& path, double x, double y) {
    size_t closest = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < path.size(); i++) {
        double dx = path[i].x - x;
        double dy = path[i].y - y;
        double dist = dx * dx + dy * dy;
        if (dist < min_dist) {
            min_dist = dist;
            closest = i;
        }
    }
    
    return closest;
}

double PathUtils::estimated_time(const std::vector<PathPoint>& path) {
    if (path.size() < 2) return 0;
    
    double total_time = 0;
    for (size_t i = 1; i < path.size(); i++) {
        double dist = path[i].distance - path[i-1].distance;
        double avg_vel = (path[i].velocity + path[i-1].velocity) / 2;
        if (avg_vel > 0.01) {
            total_time += dist / avg_vel;
        }
    }
    
    return total_time;
}

#endif // CWB_IMPLEMENTATION

} // namespace cwb
