# 2D Wall Finder using Raw Lidar Data
5 files are given, which are created by lidar sensor. Our objection was to create a simulation on the computer, show the observations of the robot and draw up the lines that are found by RANSAC algorithm. Then, we calculated the distance to the robot and whether any of the lines intercept with each other. 

## Overview
The system processes raw point cloud data, detects linear environmental features using RANSAC, and identifies potential docking targets based on geometric constraints. For more information, please check out the ***Project Report.pdf***

## Features

- **TOML Data Parser**: Custom parser for LIDAR scan data in TOML format (no external dependencies)
- **Data Preprocessing**: Filters invalid readings (NaN values, out-of-range measurements)
- **RANSAC Line Detection**: Robust line detection algorithm resilient to noise and outliers
- **Geometric Analysis**: Computes line intersections and validates them based on angular constraints
- **Real-time Visualization**: *SFML-based* graphical output showing detected lines, intersections, and robot position
- **Remote Data Support**: Downloads LIDAR data from web URLs using libcurl
- **Distance Calculation**: Computes Euclidean distance from robot to target docking points

## System Requirements
Please use VSCode to run this program and download the dependicies manually. (CMake version is coming)

## Dependencies
- C++17 compatible compiler (GCC 7+, Clang 5+, or MSVC 2017+)
- SFML 3.0+ (Simple and Fast Multimedia Library)
- libcurl (for downloading remote TOML files)

## Project Structure

```
.
├── include/
│   └── file_read.h           # File reading interface
│   └── operations.h          # Operations interface
│   └── screen.h              # Screen interface
│   └── constants.h           # Global constants and configuration
├── src/
│   └── main.cpp              # Main application entry point
│   └── file_read.cpp         # TOML parser and data reading functions
│   └── operations.cpp        # RANSAC algorithm and geometric operations
│   └── screen.cpp            # Visualization and rendering functions
├── assets/
│   └── visuals/
│       ├── arial.ttf     # Font file for text rendering
│       └── arialbd.ttf   # Bold font file
└── README.md             # This file
```

## Usage

### Running the Application

When you run the **main.cpp**, you'll be prompted to select a data source:

```
Which file you want to process?
1 2 3 4 5
```

Enter a number (1-5) to download and process LIDAR data from predefined URLs, or enter any other number to use local test data.

### TOML Data Format

The system expects TOML files with the following structure:

```toml
[header]
stamp = "timestamp"
frame_id = "laser_frame"

[scan]
angle_min = -1.5707963705062866
angle_max = 1.5707963705062866
angle_increment = 0.004363323096185923
time_increment = 0.0
scan_time = 0.0
range_min = 0.11999999731779099
range_max = 3.5

ranges = [1.23, 1.45, -1.0, 2.34, ...]
intensities = [47.0, 51.0, 0.0, 55.0, ...]
```

### Configuration Parameters

Key RANSAC parameters can be adjusted in `main.cpp`:

```cpp
RANSACparameters ransacConfig;
ransacConfig.minPoints = 8;              // Minimum points to form a line
ransacConfig.distanceThreshold = 0.01;   // 1 cm tolerance
ransacConfig.maxIterations = 100000;     // Number of random samples
```

Intersection angle threshold (default 60°):
```cpp
std::vector<Intersection> validIntersections = 
    findValidIntersections(detectedLines, dotsPOS, 60.0);
```

## Algorithm Overview

### 1. Data Acquisition and Preprocessing
- Parse TOML file using custom parser
- Filter invalid measurements (NaN, out-of-range)
- Convert polar coordinates (range, angle) to Cartesian (x, y)

### 2. RANSAC Line Detection
The RANSAC (Random Sample Consensus) algorithm:
1. Randomly selects 2 points from the point cloud
2. Fits a line through these points
3. Counts inliers (points within threshold distance)
4. Repeats for maximum iterations
5. Returns the line with the most inliers

### 3. Geometric Analysis
- Computes intersections between all detected line pairs
- Filters intersections based on angular constraint (≥60°)
- Calculates distance from robot (at origin) to valid intersections

### 4. Visualization
- Displays raw LIDAR points
- Shows detected lines with different colors
- Marks valid intersection points
- Draws distance lines from robot to targets

## Key Algorithms

### Distance from Point to Line
For a line defined as `ax + by + c = 0` and point `(x₀, y₀)`:

```
distance = |ax₀ + by₀ + c| / √(a² + b²)
```

### Line Intersection
For two lines `L₁: a₁x + b₁y + c₁ = 0` and `L₂: a₂x + b₂y + c₂ = 0`:

```
det = a₁b₂ - a₂b₁
x = (-c₁b₂ + c₂b₁) / det
y = (-a₁c₂ + a₂c₁) / det
```

### Angle Between Lines
Using slopes `m₁ = -a₁/b₁` and `m₂ = -a₂/b₂`:

```
angle = arctan(|m₁ - m₂| / |1 + m₁m₂|)
```

## Visualization Controls

- **Window Title**: Displays frame ID and timestamp from LIDAR data
- **ESC Key**: Close the application
- **Close Button**: Exit the program

### Display Elements
- **Gray Dots**: Raw LIDAR points
- **Green Dots**: Points assigned to detected lines
- **Dark Green Lines**: Detected linear features
- **Red Circles**: Valid intersection points (docking targets)
- **Red Dashed Lines**: Distance from robot to targets
- **Legend**: Shows statistics (number of lines, intersections, distances)

## Performance Considerations

- **Point Cloud Size**: Typical LIDAR scans contain 720-1440 points
- **RANSAC Iterations**: Default 100,000 iterations (adjustable)
- **Processing Time**: Typically <100ms on modern hardware
- **Memory Usage**: Minimal, primarily storing point clouds and line parameters

## Troubleshooting

### Font Loading Issues
```
Font could not be opened
```
**Solution**: Ensure `arial.ttf` and `arialbd.ttf` are in the `assets/visuals/` directory.

### CURL Download Failures
```
Failed to download TOML file
```
**Solution**: Check internet connection and firewall settings. The program will fall back to local test data.

### No Lines Detected
**Possible causes**:
- Insufficient valid points after filtering
- RANSAC parameters too strict
- Environment lacks linear features

**Solutions**:
- Reduce `distanceThreshold`
- Increase `maxIterations`
- Decrease `minPoints`

## Academic Context

This project was developed as part of a computer engineering course at Kocaeli University. It demonstrates:
- Practical application of the RANSAC algorithm
- Real-time data visualization
- Integration of multiple C++ libraries

## Authors

- Melih Eren MALLI (250201112@kocaeli.edu.tr)
- Toprak ATEŞ (250201116@kocaeli.edu.tr)

