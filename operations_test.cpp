#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <random>

#include "file_read.h"
#include "operations.h"
#include "constants.h"

// Simple distance calculations
double distanceToOrigin(const Point2D& p) {
    return std::sqrt(p.x * p.x + p.y * p.y);
}

double distancePointToLine(const Point2D& p, const Line& line) {
    return std::fabs(line.a * p.x + line.b * p.y + line.c) / 
           std::sqrt(line.a * line.a + line.b * line.b);
}

// Convert polar to Cartesian coordinates
std::vector<Point2D> convertToCarterisan(const std::vector<double>& ranges, const Scan& params) {
    std::vector<Point2D> points;
    
    for (size_t i = 0; i < ranges.size(); i++) {
        double range = ranges[i];
        
        // Filter invalid ranges
        if (range < params.range_min || range > params.range_max) continue;
        
        double angle = params.angle_min + i * params.angle_increment;
        
        Point2D point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);
        points.push_back(point);
    }
    return points;
}

// Create line equation from two points
Line createLineFromPoints(const Point2D& p1, const Point2D& p2) {
    Line line;
    
    // Line equation: ax + by + c = 0
    line.a = p2.y - p1.y;
    line.b = p1.x - p2.x;  // Note: p1.x - p2.x (not p2.x - p1.x)
    line.c = -(line.a * p1.x + line.b * p1.y);  // FIXED: was using p2.y
    
    // Normalize for consistent distance calculations
    double norm = std::sqrt(line.a * line.a + line.b * line.b);
    if (norm > almostZero) {
        line.a /= norm;
        line.b /= norm;
        line.c /= norm;
    }
    
    return line;
}

// Find intersection of two lines using Cramer's rule
bool computeLineIntersection(const Line& line1, const Line& line2, Point2D& result) {
    double det = line1.a * line2.b - line2.a * line1.b;
    
    // Check if lines are parallel
    if (std::fabs(det) < almostZero) return false;
    
    // FIXED: Signs were wrong!
    result.x = -(line1.c * line2.b - line1.b * line2.c) / det;
    result.y = -(line1.a * line2.c - line2.a * line1.c) / det;
    
    return true;
}

// Calculate angle between two lines (0-90 degrees)
double computeAngleBetweenLines(const Line& line1, const Line& line2) {
    double m1 = (std::fabs(line1.b) > almostZero) ? -line1.a / line1.b : std::numeric_limits<double>::infinity();
    double m2 = (std::fabs(line2.b) > almostZero) ? -line2.a / line2.b : std::numeric_limits<double>::infinity();
    
    double angle_rad;
    
    if (std::isinf(m1) || std::isinf(m2)) {
        angle_rad = (std::isinf(m1) && std::isinf(m2)) ? 0 : 
                    std::atan(std::fabs(std::isinf(m1) ? m2 : m1));
    } else {
        angle_rad = std::atan(std::fabs((m1 - m2) / (1 + m1 * m2)));
    }
    
    double angle_deg = angle_rad * 180.0 / M_PI;
    return (angle_deg > 90) ? 180 - angle_deg : angle_deg;
}

// RANSAC helper functions
std::vector<int> getAvailableIndices(const std::vector<bool>& used) {
    std::vector<int> indices;
    for (size_t i = 0; i < used.size(); i++) {
        if (!used[i]) indices.push_back(i);
    }
    return indices;
}

std::vector<int> findInliers(const std::vector<Point2D>& points,
                             const std::vector<int>& availableIndices,
                             const Line& line,
                             double threshold) {
    std::vector<int> inliers;
    for (int idx : availableIndices) {
        if (distancePointToLine(points[idx], line) < threshold) {
            inliers.push_back(idx);
        }
    }
    return inliers;
}

// Find best line using RANSAC algorithm
Line findBestLineRANSAC(const std::vector<Point2D>& points,
                        const std::vector<int>& availableIndices,
                        std::vector<int>& bestInliers,
                        const RANSACparameters& config,
                        std::mt19937& gen) {
    Line bestLine;
    bestInliers.clear();
    
    std::uniform_int_distribution<> dis(0, availableIndices.size() - 1);
    
    for (int iter = 0; iter < config.maxIterations; ++iter) {
        // Pick two random points
        int idx1 = availableIndices[dis(gen)];
        int idx2 = availableIndices[dis(gen)];
        if (idx1 == idx2) continue;
        
        // Create line and find inliers
        Line candidateLine = createLineFromPoints(points[idx1], points[idx2]);
        std::vector<int> inliers = findInliers(points, availableIndices, 
                                               candidateLine, config.distanceThreshold);
        
        // Keep if best so far
        if (inliers.size() > bestInliers.size()) {
            bestInliers = inliers;
            bestLine = candidateLine;
        }
    }
    
    return bestLine;
}

// Detect all lines in point cloud
std::vector<Line> detectLines(const std::vector<Point2D>& points,
                              const RANSACparameters& config) {
    std::vector<Line> detectedLines;
    std::vector<bool> used(points.size(), false);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    while (true) {
        std::vector<int> availableIndices = getAvailableIndices(used);
        
        if (availableIndices.size() < config.minPoints) break;
        
        std::vector<int> bestInliers;
        Line bestLine = findBestLineRANSAC(points, availableIndices,
                                          bestInliers, config, gen);
        
        if (bestInliers.size() >= config.minPoints) {
            bestLine.pointIndices = bestInliers;
            detectedLines.push_back(bestLine);
            
            for (int idx : bestInliers) {
                used[idx] = true;
            }
        } else {
            break;
        }
    }
    
    return detectedLines;
}

// Find valid intersections between detected lines
std::vector<Intersection> findValidIntersections(const std::vector<Line>& lines,
                                                 double minAngleThreshold) {
    std::vector<Intersection> validIntersections;
    
    // Check all line pairs
    for (size_t i = 0; i < lines.size(); ++i) {
        for (size_t j = i + 1; j < lines.size(); ++j) {
            Point2D point;
            
            if (!computeLineIntersection(lines[i], lines[j], point)) continue;
            
            double angle = computeAngleBetweenLines(lines[i], lines[j]);
            
            if (angle >= minAngleThreshold) {
                Intersection inter;
                inter.point = point;
                inter.line1_idx = i;
                inter.line2_idx = j;
                inter.angle_degrees = angle;
                inter.distance_to_robot = distanceToOrigin(point);
                
                validIntersections.push_back(inter);
            }
        }
    }
    
    return validIntersections;
}