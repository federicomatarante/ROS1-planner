#ifndef ASTAR_H
#define ASTAR_H

#include "costmap.h"
#include "point2d.h"
#include <unordered_set>
#include <vector>

/**
 * @class AStar
 * @brief Implements the A* pathfinding algorithm.
 *
 * The A* algorithm finds the shortest path from a start point to a goal
 * point within a given cost map, optionally allowing diagonal movement.
 */
class AStar {
private:
    bool diagonal = false; ///< Whether diagonal movement is allowed.

public:
    /**
     * @brief Constructs an AStar pathfinder.
     * @param diagonal If true, allows diagonal movement.
     */
    AStar(bool diagonal=false);

    /**
     * @brief Computes the shortest path using the A* algorithm.
     * @param costMap The cost map representing the environment.
     * @param start The starting point of the path.
     * @param goal The goal point of the path.
     * @return A vector of Point2D representing the computed path.
     */
    std::vector<Point2D> computePath(CostMap costMap, Point2D start, Point2D goal);
};

#endif // ASTAR_H
