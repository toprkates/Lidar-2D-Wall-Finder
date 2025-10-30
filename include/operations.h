#ifndef OPERATIONS_H
#define OPERATIONS_H

#include <vector>
#include <cmath>
#include <random>

#include "file_read.h"
#include "constants.h"


double distanceToOrigin(const Point2D& p);
double distancePointToLine(const Point2D& p, const Line& line);

std::vector<Point2D> convertToCarterisan(const std::vector<double>& ranges, const Scan& params);
Line createLineFromPoints(const Point2D& point1, const Point2D& point2);
bool computeLineIntersection (const Line& line1, const Line& line2, Point2D& result);
double computeAngleBetweenLines(const Line& line1, const Line& line2);
std::vector<int> getAvailableIndices(const std::vector<bool>& used);

std::vector<int> findInliers(const std::vector<Point2D>& points,
                            const std::vector<int>& availableIndices, 
                            const Line& line,
                            double threshold, double maxGap = 0.5);

Line findBestLineRANSAC(const std::vector<Point2D>& points,
                        const std::vector<int>& availableIndices,
                        std::vector<int>& bestInliers,
                        const RANSACparameters& config,
                        std::mt19937& gen);
                        
std::vector<Line> detectLines(const std::vector<Point2D>& points, 
                              const RANSACparameters& config);
std::vector<Intersection> findValidIntersections(const std::vector<Line>& lines, 
                        const std::vector<Point2D>& points, double minAngleThreshold);
#endif