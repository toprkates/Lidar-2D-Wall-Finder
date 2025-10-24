#include <iostream> 
#include <fstream> 
#include <string> 
#include <vector>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <random>
#include <limits>
#include <SFML/Graphics.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Point2D
{
    double x; 
    double y;
};

struct LidarScanParams
{
    double angle_min = 0.0;
    double angle_max = 0.0;
    double angle_increment = 0.0;
    double range_min = 0.0;
    double range_max = 0.0;
};

struct Line
{
    double a, b, c;
    std::vector<int> pointIndices;
    Point2D p_start;
    Point2D p_end;
    
    void computeFromPoints(const Point2D& p1, const Point2D& p2)
    {
        a = p2.y - p1.y;
        b = p1.x - p2.x;
        c = -(a * p1.x + b * p1.y);
        
        double norm = sqrt(a*a + b*b);
        if (norm > 1e-10) {
            a /= norm;
            b /= norm;
            c /= norm;
        }
    }
    
    double distanceToPoint(const Point2D& p) const
    {
        return fabs(a * p.x + b * p.y + c);
    }
};

struct Intersection
{
    Point2D point;
    int line1_idx;
    int line2_idx;
    double angle_degrees;
    double distance_to_robot;
};

void trim(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

bool parseKeyValue(const std::string& line, const std::string& key, double& value)
{
    std::stringstream ss(line);
    std::string crrKey;
    char sep;
    
    ss >> crrKey;
    if (crrKey == key) 
    {
        ss >> sep;
        if (sep == '=')
        {
            ss >> value;
            return true;
        }
    }
    return false; 
}

Point2D projectPointToLine(const Point2D& p, const Line& l)
{
    double d = l.a * p.x + l.b * p.y + l.c;
    return { p.x - d * l.a, p.y - d * l.b };
}

std::vector<Line> detectLinesRANSAC(const std::vector<Point2D>& points, 
                                     int minPoints = 8,
                                     double distanceThreshold = 0.05,
                                     int maxIterations = 1000)
{
    std::vector<Line> detectedLines;
    std::vector<bool> used(points.size(), false);
    std::random_device rd;
    std::mt19937 gen(rd());
    
    while (true)
    {
        std::vector<int> availableIndices;
        for (size_t i = 0; i < points.size(); ++i) {
            if (!used[i]) {
                availableIndices.push_back(i);
            }
        }
        
        if (availableIndices.size() < minPoints) {
            break;
        }
        
        Line bestLine;
        std::vector<int> bestInliers;
        int maxInliers = 0;
        
        for (int iter = 0; iter < maxIterations; ++iter)
        {
            std::uniform_int_distribution<> dis(0, availableIndices.size() - 1);
            int idx1 = availableIndices[dis(gen)];
            int idx2 = availableIndices[dis(gen)];
            
            if (idx1 == idx2) continue;
            
            Line candidateLine;
            candidateLine.computeFromPoints(points[idx1], points[idx2]);
            
            std::vector<int> inliers;
            for (int idx : availableIndices)
            {
                if (candidateLine.distanceToPoint(points[idx]) < distanceThreshold) {
                    inliers.push_back(idx);
                }
            }
            
            if (inliers.size() > maxInliers)
            {
                maxInliers = inliers.size();
                bestInliers = inliers;
                bestLine = candidateLine;
            }
        }
        
        if (maxInliers >= minPoints)
        {
            Point2D firstProj = projectPointToLine(points[bestInliers[0]], bestLine);
            bestLine.p_start = firstProj;
            bestLine.p_end = firstProj;

            double dir_x = -bestLine.b;
            double dir_y = bestLine.a;

            double min_t = firstProj.x * dir_x + firstProj.y * dir_y;
            double max_t = min_t;

            for (size_t i = 1; i < bestInliers.size(); ++i)
            {
                Point2D p_proj = projectPointToLine(points[bestInliers[i]], bestLine);
                double t = p_proj.x * dir_x + p_proj.y * dir_y;

                if (t < min_t)
                {
                    min_t = t;
                    bestLine.p_start = p_proj;
                }
                if (t > max_t)
                {
                    max_t = t;
                    bestLine.p_end = p_proj;
                }
            }

            bestLine.pointIndices = bestInliers;
            detectedLines.push_back(bestLine);
            
            for (int idx : bestInliers) {
                used[idx] = true;
            }
        }
        else
        {
            break;
        }
    }
    
    return detectedLines;
}

bool computeIntersection(const Line& line1, const Line& line2, Point2D& intersection)
{
    double det = line1.a * line2.b - line2.a * line1.b;
    
    if (fabs(det) < 1e-10) {
        return false;
    }
    
    intersection.x = (line1.b * line2.c - line2.b * line1.c) / det;
    intersection.y = (line2.a * line1.c - line1.a * line2.c) / det;
    
    return true;
}

bool isPointOnSegment(const Point2D& p, const Line& segment, double tolerance = 0.01)
{
    if (segment.distanceToPoint(p) > tolerance) {
        return false;
    }
    
    double min_x = std::min(segment.p_start.x, segment.p_end.x) - tolerance;
    double max_x = std::max(segment.p_start.x, segment.p_end.x) + tolerance;
    double min_y = std::min(segment.p_start.y, segment.p_end.y) - tolerance;
    double max_y = std::max(segment.p_start.y, segment.p_end.y) + tolerance;
    
    return (p.x >= min_x && p.x <= max_x &&
            p.y >= min_y && p.y <= max_y);
}

double computeAngleBetweenLines(const Line& line1, const Line& line2)
{
    double dot_product = line1.a * line2.a + line1.b * line2.b;

    if (dot_product > 1.0) dot_product = 1.0;
    if (dot_product < -1.0) dot_product = -1.0;
    
    double angle_rad = acos(fabs(dot_product));
    double angle_deg = angle_rad * 180.0 / M_PI;
    
    return angle_deg;
}

double distanceToRobot(const Point2D& point)
{
    return sqrt(point.x * point.x + point.y * point.y);
}

// Convert world coordinates to screen coordinates
sf::Vector2f worldToScreen(const Point2D& p, float centerX, float centerY, float scale)
{
    return sf::Vector2f(centerX + p.x * scale, centerY - p.y * scale);
}

void drawRadarGrid(sf::RenderWindow& window, float centerX, float centerY, float scale, float maxRange)
{
    sf::Color gridColor(50, 50, 50);
    sf::Color axisColor(80, 80, 80);
    
    // Draw concentric circles
    for (int i = 1; i <= 5; i++)
    {
        float radius = (maxRange / 5.0f) * i * scale;
        sf::CircleShape circle(radius);
        circle.setPosition(centerX - radius, centerY - radius);
        circle.setFillColor(sf::Color::Transparent);
        circle.setOutlineColor(gridColor);
        circle.setOutlineThickness(1);
        window.draw(circle);
    }
    
    // Draw radial lines
    for (int angle = 0; angle < 360; angle += 30)
    {
        float rad = angle * M_PI / 180.0f;
        float x = centerX + cos(rad) * maxRange * scale;
        float y = centerY - sin(rad) * maxRange * scale;
        
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(centerX, centerY), gridColor),
            sf::Vertex(sf::Vector2f(x, y), gridColor)
        };
        window.draw(line, 2, sf::Lines);
    }
    
    // Draw axes
    sf::Vertex xAxis[] = {
        sf::Vertex(sf::Vector2f(centerX - maxRange * scale, centerY), axisColor),
        sf::Vertex(sf::Vector2f(centerX + maxRange * scale, centerY), axisColor)
    };
    window.draw(xAxis, 2, sf::Lines);
    
    sf::Vertex yAxis[] = {
        sf::Vertex(sf::Vector2f(centerX, centerY - maxRange * scale), axisColor),
        sf::Vertex(sf::Vector2f(centerX, centerY + maxRange * scale), axisColor)
    };
    window.draw(yAxis, 2, sf::Lines);
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <file.toml>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error: Cannot open " << filename << std::endl;
        return 1;
    }
    
    LidarScanParams params;
    std::vector<double> ranges;
    std::string line;
    bool inScanSection = false;
    bool readingRanges = false;

    while (std::getline(file, line)) 
    {
        size_t brckt;
        double rangeVal;
        trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        if (line == "[scan]") { 
            inScanSection = true; 
            continue; 
        } else if (line[0] == '[') { 
            inScanSection = false; 
            readingRanges = false;
            continue; 
        }

        if (readingRanges) {
            if (line.find(']') != std::string::npos) {
                readingRanges = false; 
                line = line.substr(0, line.find(']'));
            }

            std::replace(line.begin(), line.end(), ',', ' ');
            std::stringstream ss(line);
            double rangevalue;
            while (ss >> rangevalue) {
                ranges.push_back(rangevalue);
            }
        } 
        else if (inScanSection)
        {
            if (!parseKeyValue(line, "angle_min", params.angle_min) &&
                !parseKeyValue(line, "angle_max", params.angle_max) &&
                !parseKeyValue(line, "angle_increment", params.angle_increment) &&
                !parseKeyValue(line, "range_min", params.range_min) &&
                !parseKeyValue(line, "range_max", params.range_max)) {
                
                if (line.rfind("ranges", 0) == 0) {
                    readingRanges = true;
                    brckt = line.find('[');
                    if (brckt != std::string::npos) {
                        line = line.substr(brckt + 1);
                        
                        if (line.find(']') != std::string::npos) {
                           readingRanges = false;
                           line = line.substr(0, line.find(']'));
                        }
                        
                        std::replace(line.begin(), line.end(), ',', ' ');
                        std::stringstream ss(line);
                        while (ss >> rangeVal) {
                            ranges.push_back(rangeVal);
                        }
                    }
                }
            }
        }
    }

    file.close();
    
    std::vector<Point2D> points;
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        double curRange = ranges[i];

        if (std::isinf(curRange) || curRange < params.range_min || curRange > params.range_max) {
            continue;
        }

        double curAngle = params.angle_min + i * params.angle_increment;
        Point2D point;
        point.x = curRange * std::cos(curAngle);
        point.y = curRange * std::sin(curAngle);
        points.push_back(point);
    }
    
    std::vector<Line> detectedLines = detectLinesRANSAC(points, 10, 0.025, 1000);
    
    std::vector<Intersection> validIntersections;
    double minAngleThreshold = 60.0;
    
    for (size_t i = 0; i < detectedLines.size(); ++i)
    {
        for (size_t j = i + 1; j < detectedLines.size(); ++j)
        {
            Point2D intersectionPoint;
            if (computeIntersection(detectedLines[i], detectedLines[j], intersectionPoint))
            {
                if (isPointOnSegment(intersectionPoint, detectedLines[i]) &&
                    isPointOnSegment(intersectionPoint, detectedLines[j]))
                {
                    double angle = computeAngleBetweenLines(detectedLines[i], detectedLines[j]);
                    double dist = distanceToRobot(intersectionPoint);
                    
                    if (angle >= minAngleThreshold)
                    {
                        Intersection inter;
                        inter.point = intersectionPoint;
                        inter.line1_idx = i;
                        inter.line2_idx = j;
                        inter.angle_degrees = angle;
                        inter.distance_to_robot = dist;
                        validIntersections.push_back(inter);
                    }
                }
            }
        }
    }
    
    // SFML Visualization
    sf::RenderWindow window(sf::VideoMode(1200, 800), "LIDAR Visualization");
    
    float centerX = 400.0f;
    float centerY = 400.0f;
    float scale = 30.0f; // pixels per meter
    float maxRange = 7.0f;
    
    sf::Font font;
    // Try to load a default font (you may need to provide a .ttf file)
    if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
        // Fallback if font not found - visualization will work without text
        std::cerr << "Warning: Could not load font. Text will not display." << std::endl;
    }
    
    // Segment colors
    std::vector<sf::Color> segmentColors = {
        sf::Color(0, 255, 0),    // Green
        sf::Color(255, 255, 0),  // Yellow
        sf::Color(0, 255, 255),  // Cyan
        sf::Color(255, 0, 255),  // Magenta
        sf::Color(255, 128, 0),  // Orange
        sf::Color(128, 255, 0)   // Lime
    };
    
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        
        window.clear(sf::Color(20, 20, 20));
        
        // Draw radar grid
        drawRadarGrid(window, centerX, centerY, scale, maxRange);
        
        // Draw all LIDAR points (small gray dots)
        for (const auto& p : points)
        {
            sf::Vector2f screenPos = worldToScreen(p, centerX, centerY, scale);
            sf::CircleShape dot(2);
            dot.setPosition(screenPos.x - 2, screenPos.y - 2);
            dot.setFillColor(sf::Color(100, 100, 100));
            window.draw(dot);
        }
        
        // Draw detected segments
        for (size_t i = 0; i < detectedLines.size(); ++i)
        {
            sf::Color color = segmentColors[i % segmentColors.size()];
            
            sf::Vector2f start = worldToScreen(detectedLines[i].p_start, centerX, centerY, scale);
            sf::Vector2f end = worldToScreen(detectedLines[i].p_end, centerX, centerY, scale);
            
            sf::Vertex line[] = {
                sf::Vertex(start, color),
                sf::Vertex(end, color)
            };
            window.draw(line, 2, sf::Lines);
            
            // Draw segment label
            sf::Text label;
            label.setFont(font);
            label.setString("s" + std::to_string(i + 1));
            label.setCharacterSize(14);
            label.setFillColor(color);
            label.setPosition((start.x + end.x) / 2, (start.y + end.y) / 2);
            window.draw(label);
        }
        
        // Draw robot position
        sf::CircleShape robot(8);
        robot.setPosition(centerX - 8, centerY - 8);
        robot.setFillColor(sf::Color::Red);
        window.draw(robot);
        
        // Draw valid intersections
        for (const auto& inter : validIntersections)
        {
            sf::Vector2f screenPos = worldToScreen(inter.point, centerX, centerY, scale);
            
            // Draw distance line from robot
            sf::Vertex distLine[] = {
                sf::Vertex(sf::Vector2f(centerX, centerY), sf::Color(255, 0, 0, 150)),
                sf::Vertex(screenPos, sf::Color(255, 0, 0, 150))
            };
            window.draw(distLine, 2, sf::Lines);
            
            // Draw intersection marker
            sf::CircleShape marker(6);
            marker.setPosition(screenPos.x - 6, screenPos.y - 6);
            marker.setFillColor(sf::Color::Yellow);
            marker.setOutlineColor(sf::Color::White);
            marker.setOutlineThickness(2);
            window.draw(marker);
            
            // Draw info text
            sf::Text info;
            info.setFont(font);
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) 
               << inter.angle_degrees << "° / " << inter.distance_to_robot << "m";
            info.setString(ss.str());
            info.setCharacterSize(12);
            info.setFillColor(sf::Color::White);
            info.setPosition(screenPos.x + 10, screenPos.y - 10);
            window.draw(info);
        }
        
        // Draw legend
        float legendX = 850;
        float legendY = 50;
        
        sf::Text legendTitle;
        legendTitle.setFont(font);
        legendTitle.setString("Legend");
        legendTitle.setCharacterSize(18);
        legendTitle.setFillColor(sf::Color::White);
        legendTitle.setPosition(legendX, legendY);
        window.draw(legendTitle);
        
        float yOffset = legendY + 30;
        
        // Add legend entries
        std::vector<std::pair<std::string, sf::Color>> legendItems = {
            {"All LIDAR points", sf::Color(100, 100, 100)},
            {"Robot position", sf::Color::Red}
        };
        
        for (size_t i = 0; i < detectedLines.size(); ++i)
        {
            std::string label = "s" + std::to_string(i + 1) + " (" + 
                              std::to_string(detectedLines[i].pointIndices.size()) + " pts)";
            legendItems.push_back({label, segmentColors[i % segmentColors.size()]});
        }
        
        legendItems.push_back({"60+ deg intersection", sf::Color::Yellow});
        legendItems.push_back({"Distance line", sf::Color(255, 0, 0, 150)});
        
        for (const auto& item : legendItems)
        {
            sf::CircleShape legendDot(5);
            legendDot.setPosition(legendX, yOffset);
            legendDot.setFillColor(item.second);
            window.draw(legendDot);
            
            sf::Text legendText;
            legendText.setFont(font);
            legendText.setString(item.first);
            legendText.setCharacterSize(12);
            legendText.setFillColor(sf::Color::White);
            legendText.setPosition(legendX + 15, yOffset - 2);
            window.draw(legendText);
            
            yOffset += 20;
        }
        
        window.display();
    }
    
    return 0;
}
