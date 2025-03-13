#ifndef COSTMAP_CALCULATOR_H
#define COSTMAP_CALCULATOR_H

#include <ros/ros.h>
#include <queue>
#include <vector>
#include "costmap.h"
#include "point2d.h"
#include <nav_msgs/OccupancyGrid.h>

/**
 * @class CostMapCalculator
 * @brief Computes a cost map from an occupancy grid using distance-based calculations.
 */
class CostMapCalculator {
private:
    int obstacleThreshold; ///< Threshold value to classify a cell as an obstacle.
    bool diagonal; ///< Whether diagonal movement is allowed.

    /**
     * @brief Computes the distances from obstacles using a distance transform.
     * @param obstaclesMap The cost map containing obstacle data.
     * @return A 2D vector representing distances from obstacles.
     */
    std::vector<std::vector<int>> computeDinstances(CostMap obstaclesMap);

    /**
     * @brief Computes the cost based on distance.
     * @param dinstance Distance from an obstacle.
     * @return The computed cost value.
     */
    double getCost(int dinstance);

public:
    /**
     * @brief Constructs a CostMapCalculator.
     * @param obstacleThreshold Threshold to determine obstacles in the occupancy grid.
     * @param diagonal If true, allows diagonal movement.
     */
    CostMapCalculator(int obstacleThreshold, bool diagonal);

    /**
     * @brief Generates a cost map from an occupancy grid.
     * @param gridMap The occupancy grid to convert into a cost map.
     * @return A CostMap object representing the computed cost map.
     */
    CostMap getCostMap(nav_msgs::OccupancyGrid gridMap);
};

#endif // COSTMAP_CALCULATOR_H