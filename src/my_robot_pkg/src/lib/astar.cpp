#include <iostream>
#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <bits/stdc++.h>
#include <algorithm>
#include "point2d.h"
#include "costmap.h"
#include "astar.h"

using namespace std;

// Custom hash function for Point2D
struct Point2DHash {
    std::size_t operator()(const Point2D& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

// Node struct for A* search
struct Node {
    Point2D point;
    double gScore;
    double fScore;
    Node* parent;
    
    Node(Point2D p, double g, double f, Node* par = nullptr)
        : point(p), gScore(g), fScore(f), parent(par) {}
};

AStar::AStar(bool diagonal) : diagonal(diagonal){};

// A* Algorithm
std::vector<Point2D> AStar::computePath(CostMap costMap,Point2D start, Point2D goal){
    auto cmp = [](Node* a, Node* b) { return a->fScore > b->fScore; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> openSet(cmp);
    std::unordered_map<Point2D, Node*, Point2DHash> allNodes;
    if(costMap.isObstacle(start) || costMap.isObstacle(goal)) return {};
    Node* startNode = new Node(start, 0.0, costMap.getCost(start));
    openSet.push(startNode);
    allNodes[start] = startNode;

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (current->point == goal) {
            // Reconstruct path if reached goal
            std::vector<Point2D> path;
            while (current) {
                path.push_back(current->point);
                current = current->parent;
            }
            std::reverse(path.begin(),path.end());
            return path;
        }

        for (Point2D neighbor : costMap.getNeighbours(current->point, diagonal)) {
            if (costMap.isObstacle(neighbor)) continue;
            
            double tentative_gScore = current->gScore + costMap.getCost(neighbor) + 1.0;
            if (allNodes.find(neighbor) == allNodes.end() || tentative_gScore < allNodes[neighbor]->gScore) {
                Node* neighborNode = new Node(neighbor, tentative_gScore, tentative_gScore, current);
                openSet.push(neighborNode);
                allNodes[neighbor] = neighborNode;
            }
        }
    }
    return {}; // Return empty path if no path found
}
