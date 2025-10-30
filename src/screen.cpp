#include <iostream>
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <vector>
#include <string>
#include <cmath>

#include "constants.h"

//finds the distance between two points
double distancePointsToPoints(const Point2D& p, const Point2D& q) {
    return std::sqrt((p.x - q.x)*(p.x - q.x) + (p.y - q.y)*(p.y - q.y));
}

//create dots
int addDot (sf::RenderWindow& window, float size, bool dotAlign, float posX, 
            float posY, sf::Color color) {
    sf::CircleShape circle(size, 60); //default number of corners for circle shape
    circle.setFillColor(color); //default color of circle

    //centralize the origin by moving the origin to the circle's center
    if (dotAlign) circle.setOrigin({circle.getRadius(), circle.getRadius()});
    circle.setPosition({posX, posY});

    window.draw(circle);
    return 1;
}

//create texts
int addText (sf::RenderWindow& window, sf::Font& font, std::string message, 
            float size, sf::Color color, bool textAlign, float posX, float posY, bool setRotate) {
    sf::Text text(font, message, size);
    text.setFillColor(color);
    sf::FloatRect bounds = text.getLocalBounds();
    
    //if text wants to be aligned, we will put the origin of this text to its center
    if (textAlign) text.setOrigin(bounds.size/2.f);
    text.setPosition({posX, posY});

    if (setRotate) text.setRotation(sf::degrees(90.f));

    window.draw(text);
    return 1;
}

//create lines
int addLine (sf::RenderWindow& window, float sizeX, float sizeY, float posX, float posY, sf::Color color, float angle, bool centerContent) {
    sf::RectangleShape rect({sizeX, sizeY});
    rect.setFillColor(color);
    if (centerContent) rect.setOrigin({sizeX/2, sizeY/2});
    rect.setPosition({posX, posY});
    rect.setRotation(sf::degrees(angle));

    window.draw(rect);

    return 0;
}

// every 100.f is 1 in this coordinate scaling (its defined by gridscale)
float convertCoordinateX(float x, float scale) {
    float convx = x*scale + originX;  
    return convx;
}

float convertCoordinateY(float y, float scale) {
    float convy = -y*scale + originY;  // Note the negative sign - inverts Y axis
    return convy;
}

//creating the main frame of our graph
void mainFrame(sf::RenderWindow& window, sf::Font& font, float sizeX, float sizeY,
               float posX, float posY, int thickness) {
    //creating the frame that will hold the graph, transparent by default for now
    sf::RectangleShape rect({sizeX, sizeY});
    rect.setFillColor(sf::Color::Transparent);
    rect.setOutlineThickness(thickness);
    rect.setOutlineColor(sf::Color::Transparent);
    rect.setPosition({posX, posY});
    
    //putting numbers on x-axis with with relevant numbers
    int xAxis = -3;
    for (float i=margin_X; i<=sizeX + margin_X; i+=gridscale) {
        if (xAxis != -3) addText(window, font, std::to_string(xAxis), thickness*5, sf::Color::Black, false, i, screen_Y - margin_Y+5.f, false);
        xAxis++;
    }

    //putting numbers on y-axis with relevant numbers
    int yAxis = +3;
    for (float j=margin_Y; j<=sizeY + margin_Y; j+=gridscale) {
        addText(window, font, std::to_string(yAxis), thickness*5, sf::Color::Black, false, margin_X-15.f, j, false);
        yAxis--;
    }

    window.draw(rect);
}


//dots layout
void screen(sf::RenderWindow& window, float scale, sf::Font font, 
            float startX, float startY, float endX, float endY) {
    std::string numStr;

    //putting dots seperately
    for (float i=startX; i<=endX; i+=5.f*scale) {
        for (float j=startY; j<=endY; j+=5.f*scale) {
            //check for the corners
            if ((i==startX && j==startY) || (i==endX && j==startY) || (i ==startX && j==endY) || (i==endX && j==endY)) 
                addDot(window, 2.f, true, i, j, gray);
            else addDot(window, 2.f, true, i, j, gray);
        }
    }
    //Vertical lines (vary X)
    for (float x=startX+100.f; x<endX; x+=100.f) {
        if (!(x==startX || x==endX)) {
            if (x == margin_X+(endX-startX)/2) addLine(window, 2.f, endY-startY, x, startY, darkGray, 0.f, false);
            else addLine(window, 2.f, endY-startY, x, startY, gray, 0.f, false);
        }
    }
    //Horizontal lines (vary Y)
    for (float y=startY+100.f; y<endY; y+=100.f) { //the window's y axis + 100f get us the start of the actual start of frame's y axis, until the end of the graph we put lines  
        if (!(y==startY || y==endY)){
            if (y == margin_Y+(endY-startY)/2) addLine(window, endX-startX, 2.f, startX, y, darkGray, 0.f, false);
            else addLine(window, endX-startX, 2.f, startX, y, gray, 0.f, false); // the horizontal(yatay) lines are end - start size, x varies here
        } 
    }
}

//Draws Dots that are in range
void drawInRangeDots(sf::RenderWindow& window, sf::Font& font, Point2D& point, int precision, sf::Color color) {
    double posX=point.x, posY=point.y;

    posX = convertCoordinateX(posX, gridscale);
    posY = convertCoordinateY(posY, gridscale);

    addDot(window, 3, true, posX, posY, color);
}

//create the robot
void robot(sf::RenderWindow& window, sf::Font& font) {
    addDot(window, 7, true, frame_X/2+margin_X, frame_Y/2+margin_Y, sf::Color::Red);
    addText(window, font, "Robot", 10, sf::Color::Red, true, frame_X/2+margin_X, frame_Y/2+margin_Y+10.f, false);
}

//draw a line between two points
void drawLineBetweenPoints(sf::RenderWindow& window, const Point2D& p1, const Point2D& p2,
                          sf::Color color, float thickness = 2.0f) {
    //Convert coordinates
    float screenX1 = convertCoordinateX(p1.x, gridscale);
    float screenY1 = convertCoordinateY(p1.y, gridscale);
    float screenX2 = convertCoordinateX(p2.x, gridscale);
    float screenY2 = convertCoordinateY(p2.y, gridscale);
    
    float dx = screenX2 - screenX1;
    float dy = screenY2 - screenY1;
    float length = std::sqrt(dx * dx + dy * dy);
    float angle = std::atan2(dy, dx) * 180.0f / M_PI;
    
    addLine(window, length, thickness, screenX1, screenY1, color, angle, false);
}

//draw a dashed line between two points
void drawDashedLineBetweenPoints(sf::RenderWindow& window, const Point2D& p1, const Point2D& p2,
                                 sf::Color color, float thickness = 2.0f, float dashLength = 10.0f) {
    //converting to screen coordinates
    float screenX1 = convertCoordinateX(p1.x, gridscale);
    float screenY1 = convertCoordinateY(p1.y, gridscale);
    float screenX2 = convertCoordinateX(p2.x, gridscale);
    float screenY2 = convertCoordinateY(p2.y, gridscale);
    
    //calculate distance and direction
    float dx = screenX2 - screenX1;
    float dy = screenY2 - screenY1;
    float totalLength = std::sqrt(dx * dx + dy * dy);
    float angle = std::atan2(dy, dx) * 180.0f / M_PI;
    
    //normalize direction
    float ndx = dx / totalLength;
    float ndy = dy / totalLength;
    
    //draw dashes
    bool drawDash = true;
    for (float d = 0; d < totalLength; d += dashLength) {
        if (drawDash) {
            float currentX = screenX1 + ndx * d;
            float currentY = screenY1 + ndy * d;
            float currentDashLength = std::min(dashLength * 0.6f, totalLength - d);
            
            addLine(window, currentDashLength, thickness, currentX, currentY, color, angle, false);
        }
        drawDash = !drawDash;
    }
}

//draw a line
void drawDetectedLine(sf::RenderWindow& window, const std::vector<Point2D>& points, 
                     const Line& line, sf::Color color) {
    if (line.pointIndices.size() < 2) return;
    
    Point2D firstPoint = points[line.pointIndices.front()];
    Point2D lastPoint = points[line.pointIndices.back()];
    
    //calculate line direction
    float dx = lastPoint.x - firstPoint.x;
    float dy = lastPoint.y - firstPoint.y;
    float length = std::sqrt(dx*dx + dy*dy);
    
    drawLineBetweenPoints(window, firstPoint, lastPoint, color, 3.0f);
}

//mark intersections
void drawIntersectionMarker(sf::RenderWindow& window, sf::Font& font, 
                           const Intersection& intersection, bool showLabel = true) {
    //convert to screen coordinates
    float screenX = convertCoordinateX(intersection.point.x, gridscale);
    float screenY = convertCoordinateY(intersection.point.y, gridscale);
    
    addDot(window, 4, true, screenX, screenY, sf::Color::Red);     
    addDot(window, 3, true, screenX, screenY, sf::Color::White);    
    addDot(window, 2, true, screenX, screenY, sf::Color::Red);      

    if (showLabel) {
        std::string angleStr = std::to_string((int)intersection.angle_degrees) + "\u00B0";
        std::string distStr = std::to_string((int)(intersection.distance_to_robot * 100)) + "cm";
        std::string label = angleStr + " " + distStr;
        
        addText(window, font, label, 15, sf::Color::Red, false, 
               screenX + 15, screenY - 5, false);
    }
}

//side bar for showing
void drawLegend(sf::RenderWindow& window, sf::Font& font, int numLines, 
                int numIntersections, std::vector<Line>& detectedLines,
                std::vector<Point2D>& allDots,
                std::vector<Intersection>& validIntersections) {
    float legendX = frame_X+ margin_X + 15;
    float legendY = margin_Y + 10;
    float lineHeight = 18;
    float currentY = legendY;
    
    //title
    addText(window, font, "Legend:", 12, sf::Color::Black, false, legendX, currentY, false);
    currentY += lineHeight + 5;
    
    //raw points
    addDot(window, 3, true, legendX + 5, currentY + 5, darkGray);
    addText(window, font, "Raw LIDAR Points", 10, sf::Color::Black, false, 
           legendX + 15, currentY, false);
    currentY += lineHeight;
    
    //detected points
    addDot(window, 3, true, legendX + 5, currentY + 5, sf::Color::Green);
    addText(window, font, "In Line LIDAR Points", 10, sf::Color::Black, false, 
           legendX + 15, currentY, false);
    currentY += lineHeight;
    
    //detected lines
    addDot(window, 5, true, legendX + 5, currentY + 5, darkGreen);
    std::string linesText = "Detected Lines (" + std::to_string(numLines) + ")";
    addText(window, font, linesText, 10, sf::Color::Black, false, 
           legendX + 15, currentY, false);
    currentY += lineHeight;

    //write down the lines seperately
    for (int i=0; i<detectedLines.size(); i++) {
        addDot(window, 3, true, legendX + 10, currentY + 5, darkGreen);
        std::string linesText = 
            "L-" + std::to_string(i+1) + ": " + "(" + 
            std::to_string(distancePointsToPoints(allDots[detectedLines[i].pointIndices.front()], allDots[detectedLines[i].pointIndices.back()])).substr(0, 4) + "m) " +
            std::to_string(detectedLines[i].pointIndices.size()) + " points ";
        addText(window, font, linesText, 10, sf::Color::Black, false, 
           legendX + 20, currentY, false);
        currentY += lineHeight;
    }
    
    //intersections
    addDot(window, 6, true, legendX + 5, currentY + 5, sf::Color::Red);     
    addDot(window, 4, true, legendX + 5, currentY + 5, sf::Color::White);    
    addDot(window, 2, true, legendX + 5, currentY + 5, sf::Color::Red);      
    
    std::string intersText = "Intersections (" + std::to_string(numIntersections) + ")";
    addText(window, font, intersText, 10, sf::Color::Black, false, legendX + 15, currentY, false);
    currentY += lineHeight;

    for (int i=0; i<validIntersections.size(); i++) {
        std::string eachintersText = 
            "L-" + std::to_string(validIntersections[i].line1_idx+1) +
            " and " + "L-" + std::to_string(validIntersections[i].line2_idx+1) + 
            " Intercepts at (" + std::to_string((int)validIntersections[i].point.x) + ","
            + std::to_string((int)validIntersections[i].point.y) + ") " +
            "(" + std::to_string(validIntersections[i].distance_to_robot).substr(0, 4) + "m)";

        addText(window, font, eachintersText, 10, sf::Color::Black, false, legendX + 10, currentY, false);
        currentY += lineHeight;
    }
    
    //robot
    addDot(window, 7, true, legendX + 5, currentY + 5, sf::Color::Red);
    addText(window, font, "Robot Position (0,0)", 10, sf::Color::Black, false, 
           legendX + 15, currentY, false);
}



