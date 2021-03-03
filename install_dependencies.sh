#!/usr/bin/env bash
echo "Cloning the async_web_server_cpp package"
cd ~/deepracer_ws/aws-deepracer-launcher
git clone https://github.com/GT-RAIL/async_web_server_cpp.git
cd async_web_server_cpp
git checkout ros2
apt-get install -y python3-websocket
echo ""
echo "Cloning the web_video_server package"
cd ~/deepracer_ws/aws-deepracer-launcher
git clone https://github.com/RobotWebTools/web_video_server.git
cd web_video_server
git fetch origin pull/111/head:ros2
git checkout ros2
echo ""
echo "Cloning the rplidar_ros package"
cd ~/deepracer_ws/aws-deepracer-launcher
git clone https://github.com/youngday/rplidar_ros2.git
cd ~/deepracer_ws
