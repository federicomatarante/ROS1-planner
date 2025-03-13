#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include "costmap.h"
#include "point2d.h"
#include "astar.h"
#include "costmapcalculator.h"
#include <random>

class Planner {
private:
    ros::NodeHandle nh;
    ros::Subscriber goal_sub;
    ros::Subscriber map_sub;
    ros::Subscriber initial_pose_sub;
    ros::Publisher path_pub;

    CostMapCalculator costMapCalculator;
    AStar aStar;
    CostMap* costMap;
    nav_msgs::OccupancyGrid* map;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    tf2_ros::TransformBroadcaster tfBroadcaster;

    Point2D* start;
    geometry_msgs::TransformStamped robotTransform; // Transform for the robot
    ros::Time last_map_update;

    // Converts world coordinates to pixel coordinates (grid indices)
    Point2D mapToPixel(double x, double y) {
        if (!map) {
            ROS_ERROR("mapToPixel: Map pointer is null!");
            return Point2D(0, 0);
        }
        double resolution = map->info.resolution;
        double origin_x = map->info.origin.position.x;
        double origin_y = map->info.origin.position.y;
        int u = static_cast<int>((x - origin_x) / resolution);
        int v = static_cast<int>((y - origin_y) / resolution);
        return Point2D(u, v);
    }

    // Converts pixel coordinates (grid indices) back to world coordinates
    std::vector<double> pixelToMap(int u, int v) {
        if (!map) {
            ROS_ERROR("pixelToMap: Map pointer is null!");
            return {0.0, 0.0};
        }
        double resolution = map->info.resolution;
        double origin_x = map->info.origin.position.x;
        double origin_y = map->info.origin.position.y;
        double x = origin_x + u * resolution;
        double y = origin_y + v * resolution;
        return {x, y};
    }

public:
    Planner(int obstacleThreshold, bool diagonal = false)
        : tfListener(tfBuffer),
          costMapCalculator(obstacleThreshold, diagonal),
          aStar(diagonal),
          costMap(nullptr),
          map(nullptr),
          start(nullptr)
    {
        // Set up subscribers and publisher
        goal_sub = nh.subscribe("/move_base/goal", 10, &Planner::goalCallback, this);
        map_sub = nh.subscribe("/map", 10, &Planner::gridMapCallback, this);
        initial_pose_sub = nh.subscribe("/initialpose", 10, &Planner::initialPoseCallback, this);
        path_pub = nh.advertise<nav_msgs::Path>("/path", 10);

        // Default transform settings
        robotTransform.header.frame_id = "map";
        robotTransform.child_frame_id = "base_link";
    }

    ~Planner() {
        // Clean up allocated memory
        if (map) {
            delete map;
            map = nullptr;
        }
        if (costMap) {
            delete costMap;
            costMap = nullptr;
        }
        if (start) {
            delete start;
            start = nullptr;
        }
    }

    // Callback for receiving the initial pose; also broadcasts the transform
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initialPose) {
        ROS_INFO("Initial Pose Callback triggered!");
        ROS_INFO("Received Initial Pose: (%f, %f)", 
                 initialPose->pose.pose.position.x, 
                 initialPose->pose.pose.position.y);
        
        robotTransform.header.stamp = ros::Time::now();
        robotTransform.header.frame_id = initialPose->header.frame_id;  // Typically "map"
        
        // Update translation
        robotTransform.transform.translation.x = initialPose->pose.pose.position.x;
        robotTransform.transform.translation.y = initialPose->pose.pose.position.y;
        robotTransform.transform.translation.z = initialPose->pose.pose.position.z;
        
        // Update orientation
        robotTransform.transform.rotation = initialPose->pose.pose.orientation;
        
        // Immediately broadcast the new transform
        tfBroadcaster.sendTransform(robotTransform);
    }

    // Callback for receiving a goal pose to plan a path
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
        ROS_INFO("Goal Callback triggered!");
        geometry_msgs::Pose goal_pose = goal->pose;
        ROS_INFO("Received Goal: (%f, %f)", goal_pose.position.x, goal_pose.position.y);

        // Ensure that the map, cost map, and start position are available
        if (!map) {
            ROS_ERROR("Goal Callback: Map pointer is null. Cannot compute path.");
            return;
        }
        if (!costMap) {
            ROS_ERROR("Goal Callback: CostMap pointer is null. Cannot compute path.");
            return;
        }
        if (!start) {
            ROS_ERROR("Goal Callback: Start position is not set. Cannot compute path.");
            return;
        }

        // Convert goal position to pixel coordinates
        Point2D end = mapToPixel(goal_pose.position.x, goal_pose.position.y);

        nav_msgs::Path path;
        path.header.frame_id = "map";
        std::vector<Point2D> steps = aStar.computePath(*costMap, *start, end);

        if (steps.empty()) {
            ROS_WARN("No path found!");
        }
        
        // Build the path message by converting each step back to world coordinates
        for (const Point2D &point : steps) {
            std::vector<double> world_coords = pixelToMap(point.x, point.y);
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = world_coords[0];
            pose.pose.position.y = world_coords[1];
            pose.pose.position.z = 0.0;  // 2D path: z is 0
            pose.pose.orientation.w = 1.0;  // No rotation
            path.poses.push_back(pose);
        }
        path.header.stamp = ros::Time::now();
        path_pub.publish(path);
    }

    // Callback for updating the grid map and computing the cost map
    void gridMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& new_map) {
        ros::Time current_time = ros::Time::now();
        if (last_map_update.isZero() || (current_time - last_map_update).toSec() >= 60.0) {
            ROS_INFO("Grid Map Callback triggered!");
            last_map_update = current_time;

            // Delete previous map if it exists
            if (map) {
                delete map;
                map = nullptr;
            }
            map = new nav_msgs::OccupancyGrid(*new_map);

            // Delete previous costMap if it exists
            if (costMap) {
                delete costMap;
                costMap = nullptr;
            }
            // Compute and store a new cost map
            CostMap new_cost_map = costMapCalculator.getCostMap(*new_map);
            costMap = new CostMap(new_cost_map);
            ROS_INFO("Updated map and cost map.");
        }
    }

    // Update the current robot position based on the transform from "map" to "base_link"
    void listenPosition() {
        try {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
            
            // Ensure that the map is available before converting coordinates
            if (!map) {
                ROS_WARN("listenPosition: Map is not set yet, cannot update start position.");
                return;
            }
            Point2D current_position = mapToPixel(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
            if (!start) {
                start = new Point2D(current_position);
            } else {
                *start = current_position;
            }
        } catch (tf2::TransformException &ex) {
            ROS_WARN("listenPosition: TransformException: %s", ex.what());
        }
    }

    // Main loop for the planner node
    void run() {
        ros::Rate rate(1); 
        while (ros::ok()) {
            listenPosition();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    ROS_INFO("Started planner with safety checks");
    Planner planner(2, false);
    planner.run();
    return 0;
}
