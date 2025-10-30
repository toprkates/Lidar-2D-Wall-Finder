#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <iostream>
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <vector>
#include <string>

//Struct for holding x and y positions of each dot
struct Point2D {
    double x;
    double y;
};

//using ax + by + c 
//we keep the line, also the dots that make up the line in this structure
struct Line {
    double a, b, c;
    std::vector<int> pointIndices;
};

struct RANSACparameters {
    int minPoints = 8;  //minimum points required to form a line
    double distanceThreshold = 0.05;  //max distance for point to be on the line
    int maxIterations = 1000;   //number of random samples to try per line
};

struct Intersection {
    Point2D point;  // The (x, y) location where lines intersect
    int line1_idx;  // Index of first line in the lines array
    int line2_idx;  // Index of second line in the lines array
    double angle_degrees;   // Angle between the two lines (0-90 degrees)
    double distance_to_robot;   // Distance from robot (at origin) to intersection point
};

const float almostZero = 1e-10;
#define M1_P 3.14159265358
//--------------------------------------
//--------------------------------------

#define HEADER "[header]"
#define ERROR_VECTOR {-8.8}

struct Header {
    std::string stamp;
    std::string frame_id; 
};

struct Scan {
    double angle_min;
    double angle_max;
    double angle_increment;

    double time_increment;
    double scan_time;

    double range_min;
    double range_max;
};

//--------------------------------------
//--------------------------------------

const float screen_X = 1200.f;
const float screen_Y = 800.f;
const float frame_X = 600.f;
const float frame_Y = 600.f;
const float margin_X = (screen_X-frame_X)/2;
const float margin_Y = (screen_Y-frame_Y)/2;
const float originX = frame_X/2+margin_X;
const float originY = frame_Y/2+margin_Y;

const float gridscale = 100.f;

const sf::Color gray = sf::Color(150, 200, 255, 60);
const sf::Color darkGray = sf::Color(176, 200, 224);
const sf::Color darkGreen = sf::Color(5, 137, 0);
#endif