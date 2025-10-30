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

//finds a points distance to a line
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
    double minX=1e9, maxX=-1e9, minY=1e9, maxY=-1e9;
    for (auto& p : points) {
        minX = std::min(minX, p.x);
        maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }
    std::cout << "Point range X: " << minX << " to " << maxX << " Y: " << minY << " to " << maxY << std::endl;
    return points;
}   

//creating a line from points
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
    line.b = point1.x - point2.x;
    line.c = -(line.a * point1.x + line.b * point1.y);

    //normalize the equation: divide by sqrt(a^2 + b^2) to ensure consisten distances
    double norm = std::sqrt(line.a*line.a + line.b*line.b);
    if (norm > almostZero) {    // trying not to divide with zero or near-zero 
        line.a /= norm;
        line.b /= norm;
        line.c /= norm;

        //ensure c is always positive for consistency
        if (line.c < 0) {
            line.a = -line.a;
            line.b = -line.b;
            line.c = -line.c;
        }
    }

    return line;
}

//find if the lines intersect with each other, if they do find the itercept point
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

//find the angle between intercepting lines
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
                            double threshold, double maxGap) {
    std::vector<int> inliers;
    for (int idx : availableIndices) {
        if (distancePointToLine(points[idx], line) < threshold) {
            inliers.push_back(idx);
        }
    }

    std::vector<int> filteredInliers;

    for (int idx: inliers) {
        bool hasNearbyPoint = false;
        for (int otherIdx: inliers) {
            if (idx == otherIdx) continue;

            double dx = points[idx].x - points[otherIdx].x;
            double dy = points[idx].y - points[otherIdx].y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist < maxGap) {
                hasNearbyPoint = true;
                break;
            }
        }
        if (hasNearbyPoint) filteredInliers.push_back(idx);
    }

    return filteredInliers;
}

//ransac algorithm
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

    Line bestLine;      //best line found will be stored
    bestInliers.clear(); //clearing the output parameter
    
    //random number generator for selecting points
    std::uniform_int_distribution<> dis(0, availableIndices.size() - 1);
    
    //trying random samples (Monte Carlo approach)
    for (int iter = 0; iter < config.maxIterations; ++iter) {
        // Randomly select 2 different points
        int idx1 = availableIndices[dis(gen)];
        int idx2 = availableIndices[dis(gen)];
        
        //skipping if we got the same point twice
        if (idx1 == idx2) continue;
        
        //creating a candidate line through these two points
        Line candidateLine = createLineFromPoints(points[idx1], points[idx2]);
        
        //finding all points that fit this line (inliers)
        std::vector<int> inliers = findInliers(points, availableIndices, 
                                               candidateLine, config.distanceThreshold, 0.5);
        
        //keep this line if it has more points previous
        if (inliers.size() > bestInliers.size()) {
            bestInliers = inliers;    //updating best inliers
            bestLine = candidateLine;  //updating best line
        }
    }
    
    return bestLine;
}

//detect lines
std::vector<Line> detectLines(const std::vector<Point2D>& points, 
                              const RANSACparameters& config) {
    std::vector<Line> detectedLines;              // Output: all lines found
    std::vector<bool> used(points.size(), false); // Track which points are assigned
    
    // Random number generator (seeded from hardware)
    std::random_device rd;
    std::mt19937 gen(rd());
    
    //keep finding lines until running out of points
    while (true) {
        //get list of points not assigned to any line
        std::vector<int> availableIndices = getAvailableIndices(used);
        
        //stop if not enough points remain for a valid line
        if (availableIndices.size() < config.minPoints) {
            break;
        }
        
        //find the best line in remaining points
        std::vector<int> bestInliers;
        Line bestLine = findBestLineRANSAC(points, availableIndices, 
                                          bestInliers, config, gen);
        
        //check if we found a valid line (enough inliers)
        if (bestInliers.size() >= config.minPoints) {
            // Store the line with its inlier points
            bestLine.pointIndices = bestInliers;
            detectedLines.push_back(bestLine);
            
            //mark these points as used so we don't use them again
            for (int idx : bestInliers) {
                used[idx] = true;
            }
        } else {
            //no more valid lines can be found
            break;
        }
    }
    
    return detectedLines;
}

//classic intersection test using linear algebra
bool isOnSegment(const Line& line1, const Line& line2, const std::vector<Point2D>& allPoints) {
    Point2D l1Start = allPoints[line1.pointIndices.front()];
    Point2D l1End   = allPoints[line1.pointIndices.back()];
    Point2D l2Start = allPoints[line2.pointIndices.front()];
    Point2D l2End   = allPoints[line2.pointIndices.back()];

    auto onSegment = [](Point2D p, Point2D q, Point2D r) {
        return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
               q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
    };

    auto orientation = [](Point2D p, Point2D q, Point2D r) {
        /*check the area of the three points with 
        -   |q.x-p.x  q.y-p.y|
        -   |r.x-q.x  r.y-q.y|
        - this also gives us the direction of the points, if they are lined up clockwise or not
        */
        double val = (q.y - p.y)*(r.x - q.x) - (q.x - p.x)*(r.y - q.y);
        if (std::fabs(val) < almostZero) return 0; //collinear
        return (val > 0) ? 1 : 2; //clock or counterclockwise
    };

    int o1 = orientation(l1Start, l1End, l2Start);
    int o2 = orientation(l1Start, l1End, l2End);
    int o3 = orientation(l2Start, l2End, l1Start);
    int o4 = orientation(l2Start, l2End, l1End);

    //general intersection case
    if (o1 != o2 && o3 != o4) return true;

    //special collinear cases
    if (o1 == 0 && onSegment(l1Start, l2Start, l1End)) return true;
    if (o2 == 0 && onSegment(l1Start, l2End, l1End)) return true;
    if (o3 == 0 && onSegment(l2Start, l1Start, l2End)) return true;
    if (o4 == 0 && onSegment(l2Start, l1End, l2End)) return true;

    return false;
}

//look for intersections and find them if there is any
std::vector<Intersection> findValidIntersections(const std::vector<Line>& lines, 
                        const std::vector<Point2D>& points, double minAngleThreshold) {
    std::vector<Intersection> validIntersections;

    //checking every pair of lines (combinatorial: n choose 2)
    for (size_t i = 0; i < lines.size(); ++i) {
        for (size_t j = i + 1; j < lines.size(); ++j) {
            Point2D point;

            //trying to find where these two lines intersect
            if (!computeLineIntersection(lines[i], lines[j], point))
                continue;  // Lines are parallel - no intersection
                        
            if (!isOnSegment(lines[i], lines[j], points))
                continue;

            //calculating the angle between the two lines
            double angle = computeAngleBetweenLines(lines[i], lines[j]);
            
            //only keep intersections with sharp enough angles
            //this filters out near-parallel line intersections
            if (angle >= minAngleThreshold || angle <= 90 - minAngleThreshold) {
                //creating intersection record with all relevant data
                Intersection inter;
                inter.point = point;  //lines meet
                inter.line1_idx = i;  //index of first line
                inter.line2_idx = j;  //index of second line
                if (angle >= minAngleThreshold) inter.angle_degrees = angle;
                else if (angle <= (90 - minAngleThreshold)) inter.angle_degrees = 90 - angle;
                inter.distance_to_robot = distanceToOrigin(point); //how far from robot
                
                validIntersections.push_back(inter);
            }
        }
    }
    return validIntersections;
}