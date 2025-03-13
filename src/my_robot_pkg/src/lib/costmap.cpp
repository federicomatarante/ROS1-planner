#include <iostream>
#include <iomanip>
#include <vector>
#include "costmap.h"
#include "point2d.h"

// Method to check validity of a point inside the map
void CostMap::checkValidity(int r, int c) {
    if (!(r >= 0 && r < rows && c >= 0 && c < cols)) {
        throw std::runtime_error("Point must be inside the map!");
    }
}

// Method to check if a point is an obstacle
bool CostMap::isObstacle(int r, int c) {
    checkValidity(r, c);
    return map[r][c] == -1.0;
}

CostMap::CostMap(const CostMap& other) : rows(other.rows), cols(other.cols) {
    map = other.map; 
}
// Constructor for CostMap
CostMap::CostMap(int r, int c, double defaultValue) : rows(r), cols(c) {
    map = std::vector<std::vector<double>>(rows, std::vector<double>(cols, defaultValue));
}

// Method to set the cost at a specific point (r, c)
void CostMap::setCost(int r, int c, double value) {
    if (value < 0) {
        throw std::runtime_error("Cost must be positive! Got: " + std::to_string(value));
    }
    checkValidity(r, c);
    map[r][c] = value;
}

// Method to set a point as an obstacle
void CostMap::setObstacle(int r, int c) {
    checkValidity(r, c);
    map[r][c] = -1.0;  // Mark as obstacle
}

// Overloaded method to check if a Point2D is an obstacle
bool CostMap::isObstacle(Point2D point) {
    return isObstacle(point.y, point.x);
}

// Method to check validity of a Point2D
void CostMap::checkValidity(Point2D point) {
    checkValidity(point.y, point.x);
}

// Method to print the map (for debugging purposes)
void CostMap::print() const {
    
    std::cout << "Cost Map: \n" ;
    for (const auto& row : map) {
        for (double cell : row) {
            if(cell==-1){
                std::cout << "\u2588"<<"\u2588"<<"\u2588" << "\t";
            } else {
                std::cout << std::fixed << std::setprecision(2) << cell << "\t";
            }
        }
        std::cout << "\n";
    }
}

// Method to get the cost at a specific Point2D
double CostMap::getCost(Point2D point) {
    return getCost(point.y, point.x);
}

// Method to get the cost at a specific point (r, c)
double CostMap::getCost(int r, int c) {
    checkValidity(r, c);
    return map[r][c];
}

// Method to get a list of all obstacles
std::vector<Point2D> CostMap::getObstacles() {
    std::vector<Point2D> obstacles;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (isObstacle(i, j)) {
                obstacles.push_back(Point2D(j, i));
            }
        }
    }
    return obstacles;
}

// Method to get the neighbours of a point, including diagonals if requested
std::vector<Point2D> CostMap::getNeighbours(Point2D point, bool includeDiagonal) {

    checkValidity(point);

    std::vector<Point2D> neighbours;

    int r0 = (point.y != 0) ? point.y - 1 : 0;
    int rf = (point.y != rows - 1) ? point.y + 1 : rows - 1;

    int c0 = (point.x != 0) ? point.x - 1 : 0;
    int cf = (point.x != cols - 1) ? point.x + 1 : cols - 1;

    for (int r = r0; r <= rf; r++) {
        for (int c = c0; c <= cf; c++) {
            // Skip the current point (it is not a neighbor)
            if (r == point.y && c == point.x) continue;

            // Skip diagonal if includeDiagonal is false
            if (!includeDiagonal && (r != point.y && c != point.x)) continue;
            
            Point2D p = Point2D(c, r);
            if (isObstacle(p)) continue;  // Skip obstacles
            neighbours.push_back(p);
        }
    }

    return neighbours;
}
