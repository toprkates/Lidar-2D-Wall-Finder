#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <random>

#include "file_read.h"
#include "operations.h"
#include "constants.h"

//finds the distance
double distanceToOrigin(const Point2D& p) {
    return std::sqrt(p.x * p.x + p.y * p.y);
}

double distancePointToLine(const Point2D& p, const Line& line) {
    /* 
    - Calculate signed distance and take absolute value
    - |ax+by +c| / sqrt(a^2 + b^2)
    - this function gives us the distance between the dot and line
    - this is used in graph anaysis in math, actually
    */
    return std::fabs(line.a * p.x + line.b * p.y + line.c) / 
           std::sqrt(line.a * line.a + line.b * line.b);
}

//using angles and distance from the origin where robot lies, finds the exact coordinates
//x=line.cosx, y=line.siny
std::vector<Point2D> convertToCarterisan(const std::vector<double>& ranges, const Scan& params) {
    std::vector<Point2D> points;

    //find each point's, whose range we know, coordinates
    for (int i=0; i<ranges.size(); i++) {
        double range = ranges[i];

        //Filter out dots that are not in our range
        if (range < params.range_min || range > params.range_max) continue;

        //calculate the angle
        //angle = starting_angle + (reading_index * angular_increase)
        double angle = params.angle_min + i*params.angle_increment;

        //convert to xy coordinates through trigonometry
        Point2D point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);

        points.push_back(point);
    }
    return points;
}   


Line createLineFromPoints(const Point2D& point1, const Point2D& point2) {
    Line line;
    /*
    - calculating the coefficients of the line thats made by point1 and point2
    - using ax +by +c = 0
    - a = y2 - y1
    - b = x2 - x1
    - c = -(a*x1 + b*y1) [using either x1,y1 or x2,y2]
    */
    line.a = point2.y - point1.y;
    line.b = point2.x - point1.x;
    line.c = -(line.a * point1.x + line.b * point1.y);

    // normalize the equation: divide by sqrt(a^2 + b^2)
    // ensuring consistent distances
    double norm = std::sqrt(line.a*line.a + line.b*line.b);
    if (norm > almostZero) {    // trying not to divide with zero or near-zero 
        line.a /= norm;
        line.b /= norm;
        line.c /= norm;
    }

    return line;
}

bool computeLineIntersection (const Line& line1, const Line& line2, Point2D& result) {
    /*
    - a1*x + b1*y + c1 = 0
    - a2*x + b2*y + c2 = 0
    - Using Cramer's rule, we can find the intercept of two given lines.
    -              a1*x + b1*y = -c1
    -              a2*x + b2*y = -c2
    - 
    - det(A) = a1*b2 - a2*b1
    - x = (-c1*b2 - (-c2)*b1) / det = (-c1*b2 + c2*b1) / det
    - y = (a1*(-c2) - a2*(-c1)) / det = (-a1*c2 + a2*c1) / det
    */

    double det = line1.a * line2.b - line2.a * line1.b;

    //if determinant is zero or almost zero, it means that these lines are parallel to each other
    if (std::fabs(det) < almostZero) return false;

    result.x = (-line1.c * line2.b + line2.c * line1.b) / det;
    result.y = (-line1.a * line2.c + line2.a * line1.c) / det;

    return true;
}

double computeAngleBetweenLines(const Line& line1, const Line& line2) {
    /*
    - using the default ax + by + c, slope of this line is m = -a/b 
    - (if line is perpendicular to x-axis, its slope is infinity)
    */
    double m1 = (std::fabs(line1.b) > almostZero) ? -line1.a / line1.b : std::numeric_limits<double>::infinity();
    double m2 = (std::fabs(line2.b) > almostZero) ? -line2.a / line2.b : std::numeric_limits<double>::infinity();

    double angle_rad;
    //Check for the infinity or near infinity slopes
    if (std::isinf(m1) || std::isinf(m2)) {
        //Parallel to each other
        if (std::isinf(m1) && std::isinf(m2)) {
            angle_rad = 0;
        } else {
            //angle = arctan(|m|)
            angle_rad = std::atan(std::fabs(std::isinf(m1) ? m2 : m1));
        }
    } else {
        /*         | m1 - m2 |
        - tan(x) = |---------|
        -          |1 + m1*m2|
        */         
        angle_rad = std::atan(std::fabs((m1 - m2) / (1 + m1 * m2)));
    }

    double angle_deg = angle_rad * 180.0 / M_PI;
    return (angle_deg > 90) ? 180 - angle_deg : angle_deg;
}

//RANSAC Detect Functions
//Checking Available Dots
std::vector<int> getAvailableIndices(const std::vector<bool>& used) {
    std::vector<int> indices;
    for (size_t i=0; i < used.size(); i++) {
        if (!used[i])
            indices.push_back(i);
    }
    return indices;
}

//Finds all the points that lie close enough to the line, and returns their indices
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

Line findBestLineRANSAC(const std::vector<Point2D>& points,
                        const std::vector<int>& availableIndices,
                        std::vector<int>& bestInliers,
                        const RANSACparameters& config,
                        std::mt19937& gen) {
    /*
    - RANSAC (Random Sample Consensus) Algorithm:
    - Randomly select 2 points
    - Create a line through those points
    - Count how many other points fit this line (inliers)
    - Repeat many times
    - Keep the line with most inliers (best fit)
    */

    Line bestLine;      // Will store the best line found
    bestInliers.clear(); // Clear output parameter
    
    // Random number generator for selecting points
    std::uniform_int_distribution<> dis(0, availableIndices.size() - 1);
    
    // Try many random samples (Monte Carlo approach)
    for (int iter = 0; iter < config.maxIterations; ++iter) {
        // Randomly select 2 different points
        int idx1 = availableIndices[dis(gen)];
        int idx2 = availableIndices[dis(gen)];
        
        // Skip if we got the same point twice
        if (idx1 == idx2) continue;
        
        // Create a candidate line through these two points
        Line candidateLine = createLineFromPoints(points[idx1], points[idx2]);
        
        // Find all points that fit this line (inliers)
        std::vector<int> inliers = findInliers(points, availableIndices, 
                                               candidateLine, config.distanceThreshold);
        
        // Keep this line if it's better than previous best
        // "Better" means more points fit the line
        if (inliers.size() > bestInliers.size()) {
            bestInliers = inliers;    // Update best inliers
            bestLine = candidateLine;  // Update best line
        }
    }
    
    return bestLine;
}

std::vector<Line> detectLines(const std::vector<Point2D>& points, 
                              const RANSACparameters& config) {
    std::vector<Line> detectedLines;              // Output: all lines found
    std::vector<bool> used(points.size(), false); // Track which points are assigned
    
    // Random number generator (seeded from hardware)
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Keep finding lines until we run out of points
    while (true) {
        // Get list of points not yet assigned to any line
        std::vector<int> availableIndices = getAvailableIndices(used);
        
        // Stop if not enough points remain for a valid line
        if (availableIndices.size() < config.minPoints) {
            break;
        }
        
        // Find the best line in remaining points
        std::vector<int> bestInliers;
        Line bestLine = findBestLineRANSAC(points, availableIndices, 
                                          bestInliers, config, gen);
        
        // Check if we found a valid line (enough inliers)
        if (bestInliers.size() >= config.minPoints) {
            // Store the line with its inlier points
            bestLine.pointIndices = bestInliers;
            detectedLines.push_back(bestLine);
            
            // Mark these points as used so we don't use them again
            for (int idx : bestInliers) {
                used[idx] = true;
            }
        } else {
            // No more valid lines can be found
            break;
        }
    }
    
    return detectedLines;
}

std::vector<Intersection> findValidIntersections(const std::vector<Line>& lines, 
                                                double minAngleThreshold) {
    std::vector<Intersection> validIntersections;
    
    // Check every pair of lines (combinatorial: n choose 2)
    for (size_t i = 0; i < lines.size(); ++i) {
        for (size_t j = i + 1; j < lines.size(); ++j) {
            Point2D point;
            
            // Try to find where these two lines intersect
            if (!computeLineIntersection(lines[i], lines[j], point)) {
                continue;  // Lines are parallel - no intersection
            }
            
            // Calculate the angle between the two lines
            double angle = computeAngleBetweenLines(lines[i], lines[j]);
            
            // Only keep intersections with sharp enough angles
            // This filters out near-parallel line intersections
            if (angle >= minAngleThreshold) {
                // Create intersection record with all relevant data
                Intersection inter;
                inter.point = point;                           // Where lines meet
                inter.line1_idx = i;                          // Index of first line
                inter.line2_idx = j;                          // Index of second line
                inter.angle_degrees = angle;                   // Angle between lines
                inter.distance_to_robot = distanceToOrigin(point); // How far from robot
                
                validIntersections.push_back(inter);
            }
        }
    }
    
    return validIntersections;
}

