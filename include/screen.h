#ifndef SCREEN_H
#define SCREEN_H
#include <string>
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>

#include "operations.h"
#include "constants.h"

int draw();
void robot(sf::RenderWindow& window, sf::Font& font);
void dots(sf::RenderWindow& window, sf::Font& font, std::vector<double> dotsPosArr);
void screen(sf::RenderWindow& window, float scale, sf::Font font, float startX, float startY, float endX, float endY);
void mainFrame(sf::RenderWindow& window, sf::Font& font, float sizeX, float sizeY, float posX, float posY, int thickness);
int addLine (sf::RenderWindow& window, float sizeX, float sizeY, float posX, float posY, sf::Color color, float angle, bool centerContent);
float convertCoordinateX(float x, float scale);
float convertCoordinateY(float y, float scale);
int addText (sf::RenderWindow& window, sf::Font& font, std::string message, float size, sf::Color color, bool textAlign, float posX, float posY, bool setRotate);
int addDot (sf::RenderWindow& window, float size, bool dotAlign, float posX, float posY, sf::Color color);
void drawInRangeDots(sf::RenderWindow& window, sf::Font& font, Point2D& point, int precision, sf::Color color);
void drawIntersectionMarker(sf::RenderWindow& window, sf::Font& font, const Intersection& intersection, bool showLabel = true);
void drawDetectedLine(sf::RenderWindow& window, const std::vector<Point2D>& points, const Line& line, sf::Color color);
void drawLegend(sf::RenderWindow& window, sf::Font& font, int numLines, int numIntersections, std::vector<Line>& detectedLines, std::vector<Point2D>& allDots, std::vector<Intersection>& validIntersections);
void drawDashedLineBetweenPoints(sf::RenderWindow& window, const Point2D& p1, const Point2D& p2,
                                 sf::Color color, float thickness = 2.0f, float dashLength = 10.0f);

#endif