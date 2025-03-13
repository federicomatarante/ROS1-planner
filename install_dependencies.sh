#!/bin/bash

echo "Updating package list..."
sudo apt update

echo "Installing ROS dependencies..."
rosdep install --from-paths src --ignore-src -r -y

echo "Done!"
