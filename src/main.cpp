#include <iostream>
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <vector>
#include <string>
#include <filesystem>

#include "constants.h"
#include "file_read.h"
#include "operations.h"
#include "screen.h"

#define TEST_DATA "scan_data_NaN.toml"
#define MERMELAT_URL "https://gist.githubusercontent.com/Mermalat/9b923dd7b053aa442fbc73b0f9d5d28a/raw/337861cf6c0a9ec2dcdf7a3cfbe119a19924e995/sdata"

#define URL1 "http://abilgisayar.kocaeli.edu.tr/lidar1.toml"
#define URL2 "http://abilgisayar.kocaeli.edu.tr/lidar2.toml"
#define URL3 "http://abilgisayar.kocaeli.edu.tr/lidar3.toml"
#define URL4 "http://abilgisayar.kocaeli.edu.tr/lidar4.toml"
#define URL5 "http://abilgisayar.kocaeli.edu.tr/lidar5.toml"

#define DOWNLOADED_FILE "downloaded_lidar.toml"

void drawAllLines(sf::RenderWindow& window, sf::Font& font, 
                  sf::Font& boldFont,
                  const std::vector<Point2D>& dotsPOS,
                  const std::vector<Line>& detectedLines) {
    
    for (size_t i = 0; i < detectedLines.size(); ++i) {
        //draw points belonging to this line
        for (int idx : detectedLines[i].pointIndices) {
            Point2D point = dotsPOS[idx];
            drawInRangeDots(window, font, point, 6, sf::Color::Green);
        }
        
        //draw line segment
        drawDetectedLine(window, dotsPOS, detectedLines[i], darkGreen);
        
        //add labels
        if (!detectedLines[i].pointIndices.empty()) {
            size_t midIdx = detectedLines[i].pointIndices.size() / 2;
            Point2D midPoint = dotsPOS[detectedLines[i].pointIndices[midIdx]];
            float screenX = convertCoordinateX(midPoint.x, gridscale);
            float screenY = convertCoordinateY(midPoint.y, gridscale);
            
            std::string label = "L" + std::to_string(i + 1);
            addText(window, boldFont, label, 12, sf::Color::Black, false, screenX, screenY - 15, false);
        }
    }
}

void drawAllIntersections(sf::RenderWindow& window, sf::Font& arial,
                         const std::vector<Intersection>& validIntersections) {
    
    Point2D robotPos = {0.0, 0.0};
    
    for (const Intersection& inter : validIntersections) {
        // Draw dashed line from robot to intersection
        drawDashedLineBetweenPoints(window, robotPos, inter.point, 
                                   sf::Color::Red, 2.f, 10.f);
        
        // Draw intersection marker with label
        drawIntersectionMarker(window, arial, inter, true);
    }
}

void drawUIElements(sf::RenderWindow& window, sf::Font& arial) {
    screen(window, 2.f, arial, margin_X, margin_Y, frame_X+margin_X, frame_Y+margin_Y);
    mainFrame(window, arial, frame_X, frame_Y, margin_X, margin_Y, 2);
    addText(window, arial, std::to_string((int)frame_X), 10, sf::Color::Black, 
            false, frame_X/2+margin_X, margin_Y-30.f, false);
    addText(window, arial, std::to_string((int)frame_Y), 10, sf::Color::Black, 
            false, margin_X-50.f, screen_Y/2, false);
}

int main() {
    //downloading TOML files from web
    int choice;
    std::string url;
    bool localData = false;
    std::cout << "which file you want to process?" << std::endl << "1 2 3 4 5 ";
    std::cin >> choice;

    if (choice == 1) url = URL1;
    else if (choice == 2) url = URL2;
    else if (choice == 3) url = URL3;
    else if (choice == 4) url = URL4;
    else if (choice == 5) url = URL5;
    else if (choice == 10) url = MERMELAT_URL;
    else localData = true;

    std::cout << "Downloading TOML file from: " << url << std::endl;
    if (!downloadTomlFile(url, DOWNLOADED_FILE)) {
        std::cerr << "Failed to download TOML file. Using local test data instead." << std::endl;
        localData = true;
    }

    //choose which file to use
    std::string dataFile;
    if (localData) dataFile = TEST_DATA;
    else dataFile = DOWNLOADED_FILE;
    std::cout << "Using data file: " << dataFile << std::endl;

    //Load the Head of the File, Lidar Scanned Parameters, Ranges and Intensities
    Header header = readHeader(dataFile);
    Scan scan = readScan(dataFile);
    std::vector<double> rangesPositions = readRanges(dataFile);
    std::vector<double> intensities = readIntensities(dataFile);
    std::vector<Point2D> dotsPOS = convertToCarterisan(rangesPositions, scan);

    RANSACparameters ransacConfig;
    ransacConfig.minPoints = 8;              //minimum points to form a line
    ransacConfig.distanceThreshold = 0.01;   //1 cm tolerance
    ransacConfig.maxIterations = 10*10000;   //number of random samples

    std::vector<Line> detectedLines = detectLines(dotsPOS, ransacConfig);
    std::vector<Intersection> validIntersections = findValidIntersections(detectedLines, dotsPOS, 60.0);

    std::cout << "\n=== RANSAC Results ===" << std::endl;
    std::cout << "Points: " << dotsPOS.size() << std::endl;
    std::cout << "Lines: " << detectedLines.size() << std::endl;
    std::cout << "Intersections: " << validIntersections.size() << std::endl;

    for (const auto& inter : validIntersections) {
        std::cout << "Intersection at world coords: (" << inter.point.x << ", " << inter.point.y << ")\n";

        float screenX = convertCoordinateX(inter.point.x, gridscale);
        float screenY = convertCoordinateY(inter.point.y, gridscale);

        std::cout << "Line " << inter.line1_idx+1 << ": " << detectedLines[inter.line1_idx].a << "x + "<< detectedLines[inter.line1_idx].b << "y + " << detectedLines[inter.line1_idx].c << std::endl;
        std::cout << "Line " << inter.line2_idx+1 << ": " << detectedLines[inter.line2_idx].a << "x + "<< detectedLines[inter.line2_idx].b << "y + " << detectedLines[inter.line2_idx].c << std::endl;
        std::cout << "  Screen coords: (" << screenX << ", " << screenY << ")\n";
        std::cout << "  Between lines: " << inter.line1_idx+1 << " and " << inter.line2_idx+1 << "\n";
        std::cout << "  Angle: " << inter.angle_degrees << " degrees\n\n";

        if (localData) std::cout << "Local Data Used" << std::endl;
        else std::cout << "The Used URL: " << url << std::endl;
    }

    //create the window for drawing
    sf::RenderWindow window(sf::VideoMode({(unsigned int) screen_X, (unsigned int) screen_Y}), header.frame_id + " " + header.stamp);

    //loading font
    sf::Font arial;
    sf::Font boldArial;
    if (!arial.openFromFile("assets/visuals/arial.ttf") || !boldArial.openFromFile("assets/visuals/arialbd.ttf")) {
        std::cerr << "Font could not be opened" << std::endl;
        return -1;
    }

    //main loop for screen
    while (window.isOpen()) {
        //handle events
        while (std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
            if (auto key = event->getIf<sf::Event::KeyPressed>()) {
                if (key->code == sf::Keyboard::Key::Escape)
                    window.close();
            }
        }
        
        window.clear(sf::Color::White);

        //draw everything in layers
        drawUIElements(window, arial);
        
        //draw all raw points
        for (Point2D point : dotsPOS)
            drawInRangeDots(window, arial, point, 5, darkGray);

        //draw detected lines 
        drawAllLines(window, arial, boldArial, dotsPOS, detectedLines);
        
        //draw intersections
        drawAllIntersections(window, arial, validIntersections);
        
        //draw robot and legend
        robot(window, boldArial);
        drawLegend(window, arial, detectedLines.size(), validIntersections.size(), detectedLines, dotsPOS, validIntersections);

        window.display();
    }
}
