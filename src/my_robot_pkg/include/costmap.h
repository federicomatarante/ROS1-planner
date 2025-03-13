#ifndef COSTMAP_H
#define COSTMAP_H

#include <vector>
#include <stdexcept>
#include "point2d.h"

/**
 * @class CostMap
 * @brief Represents a 2D grid with cost values and obstacles for pathfinding.
 *
 * The CostMap class stores cost values for each grid cell, allowing pathfinding
 * algorithms to determine movement feasibility and efficiency.
 */
class CostMap {
public:
    int rows; ///< Number of rows in the cost map.
    int cols; ///< Number of columns in the cost map.

    /**
     * @brief Constructs a CostMap with the given dimensions and default cost.
     * @param rows Number of rows.
     * @param cols Number of columns.
     * @param defaultValue Default cost value for each cell (default is 0.0).
     */
    CostMap(int rows, int cols, double defaultValue = 0.0);

    /**
     * @brief Copy constructor.
     * @param other Another CostMap instance to copy.
     */
    CostMap(const CostMap& other);

    /**
     * @brief Checks if the given row and column indices are valid.
     * @param r Row index.
     * @param c Column index.
     * @throws std::out_of_range If indices are out of bounds.
     */
    void checkValidity(int r, int c);

    /**
     * @brief Checks if the given Point2D coordinates are valid.
     * @param point The point to validate.
     * @throws std::out_of_range If the point is out of bounds.
     */
    void checkValidity(Point2D point);

    /**
     * @brief Sets the cost of a specific cell.
     * @param r Row index.
     * @param c Column index.
     * @param value Cost value to set.
     */
    void setCost(int r, int c, double value);

    /**
     * @brief Marks a specific cell as an obstacle.
     * @param r Row index.
     * @param c Column index.
     */
    void setObstacle(int r, int c);

    /**
     * @brief Checks if a specific cell is an obstacle.
     * @param x Row index.
     * @param y Column index.
     * @return True if the cell is an obstacle, false otherwise.
     */
    bool isObstacle(int x, int y);

    /**
     * @brief Checks if a specific point is an obstacle.
     * @param point The point to check.
     * @return True if the point is an obstacle, false otherwise.
     */
    bool isObstacle(Point2D point);

    /**
     * @brief Retrieves the cost of a specific cell.
     * @param r Row index.
     * @param c Column index.
     * @return The cost value of the cell.
     */
    double getCost(int r, int c);

    /**
     * @brief Retrieves the cost of a specific point.
     * @param point The point to check.
     * @return The cost value of the point.
     */
    double getCost(Point2D point);

    /**
     * @brief Prints the cost map to the console.
     */
    void print() const;

    /**
     * @brief Retrieves a list of all obstacle points in the cost map.
     * @return A vector of Point2D representing obstacle locations.
     */
    std::vector<Point2D> getObstacles();

    /**
     * @brief Gets the neighboring points of a given point.
     * @param point The reference point.
     * @param includeDiagonal If true, includes diagonal neighbors.
     * @return A vector of Point2D representing neighboring points.
     */
    std::vector<Point2D> getNeighbours(Point2D point, bool includeDiagonal = false);

private:
    std::vector<std::vector<double>> map; ///< 2D vector representing the cost map grid.
};

#endif // COSTMAP_H