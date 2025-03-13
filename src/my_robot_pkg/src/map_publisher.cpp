#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>

bool loadMapFromImage(const std::string& filename, nav_msgs::OccupancyGrid& map_msg, double resolution)
{
    // Load image with alpha channel (RGBA)
    cv::Mat img = cv::imread(filename, cv::IMREAD_UNCHANGED); 
    if (img.empty()) {
        ROS_ERROR("Failed to load image: %s", filename.c_str());
        return false;
    }

    if (img.channels() != 4) {
        ROS_ERROR("Image does not have an alpha channel: %s", filename.c_str());
        return false;
    }

    map_msg.info.resolution = resolution;
    map_msg.info.width = img.cols;
    map_msg.info.height = img.rows;
    map_msg.info.origin.position.x = 0.0;
    map_msg.info.origin.position.y = 0.0;
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;

    map_msg.data.resize(map_msg.info.width * map_msg.info.height);

    for (int x = 0; x < img.rows; x++) { 
        for (int y = 0; y < img.cols; y++) {  
            cv::Vec4b pixel = img.at<cv::Vec4b>(x, y); 

            int r = pixel[2]; 
            int g = pixel[1];  
            int b = pixel[0];
            int alpha = pixel[3]; 
            if(alpha==0){
                map_msg.data[x * img.cols + y] = -1;
            } else {
                int pixel_value = (r + g + b) / 3;  // Average RGB to get grayscale value
                int obstacle_value = int(100 * (255 - pixel_value) / 255);  // Convert to OccupancyGrid value
                map_msg.data[x * img.cols + y] = obstacle_value;
            }
            
        }
    }

    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    ros::Rate rate(0.1);  

    // Load map from image
    std::string package_path = ros::package::getPath("my_robot_pkg");
    std::string map_file = package_path + "/rsc/diag.png";  
    double resolution = 0.02;  // 2 cm per pixel resolution

    // Create msg
    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.frame_id = "map";

    if (!loadMapFromImage(map_file, map_msg, resolution)) {
        ROS_ERROR("Error loading map. Exiting.");
        return -1;
    }

    while (ros::ok()) {
        map_msg.header.stamp = ros::Time::now();
        map_pub.publish(map_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
