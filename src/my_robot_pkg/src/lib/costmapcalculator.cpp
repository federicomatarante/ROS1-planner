#include "costmapcalculator.h"
#include "costmap.h"
#include <queue>
#include <vector>
#include <point2d.h>
#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>

CostMapCalculator::CostMapCalculator(int obstacleThreshold, bool diagonal)
    : obstacleThreshold(obstacleThreshold), diagonal(diagonal) {}

std::vector<std::vector<int>> CostMapCalculator::computeDinstances(CostMap obstaclesMap) {

    std::queue<Point2D> q;
    std::vector<std::vector<int>> dinstances(obstaclesMap.rows, std::vector<int>(obstaclesMap.cols, -1));
    for (Point2D obstacle : obstaclesMap.getObstacles()) {
        q.push(obstacle);
        dinstances[obstacle.y][obstacle.x] = 0;
    }

    while (!q.empty()) {

        Point2D point = q.front();
        q.pop();

        int currDinstance = dinstances[point.y][point.x];

        for (Point2D neighbour : obstaclesMap.getNeighbours(point, diagonal)) {
            int neighbourDinstance = dinstances[neighbour.y][neighbour.x];
            if (neighbourDinstance == -1) { // If it's not already been touched
                dinstances[neighbour.y][neighbour.x] = currDinstance + 1;
                q.push(neighbour);
            }
        }
    }

    return dinstances;
}

double CostMapCalculator::getCost(int dinstance) {
    return 20.0 / dinstance;
}

CostMap CostMapCalculator::getCostMap(nav_msgs::OccupancyGrid gridMap) {
    CostMap costMap = CostMap(gridMap.info.height, gridMap.info.width,0);
    for (int r = 0; r < costMap.rows; r++) {
        for (int c = 0; c < costMap.cols; c++) {
            int i = r * costMap.cols + c;
            if (gridMap.data[i] == -1 || gridMap.data[i] > obstacleThreshold) {
                costMap.setObstacle(r, c);
            }
        }
    }
    std::vector<std::vector<int>> dinstances = computeDinstances(costMap);
    for (int r = 0; r < costMap.rows; r++) {
        for (int c = 0; c < costMap.cols; c++) {
            int dinstance = dinstances[r][c];
            if (dinstance == 0) continue;
            double cost = getCost(dinstance);
            costMap.setCost(r, c, cost);
        }
    }

    return costMap;
}
