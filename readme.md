# AWS DeepRacer core application overview 

<p align="center">
<img src="/media/deepracer_circle_sticker.png" width="250" height="250" >
</p>

The AWS DeepRacer vehicle is a Wi-Fi enabled physical vehicle that can 
drive itself on a physical track by using a reinforcement learning model. This 
repository contains the robot application code shipped with the AWS DeepRacer hardware. 
It also includes examples demonstrating how you can extend the AWS DeepRacer application for new 
scenarios. For example, the [Follow the Leader (FTL)](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project) sample project shows how you can use object detection and 
navigation to modify the vehicle application to follow you as you move about a room.

If you are a first-time user, do the following:

1. Read [What Is AWS DeepRacer?](https://docs.aws.amazon.com/deepracer/latest/developerguide/what-is-deepracer.html). This is the documentation for AWS DeepRacer. It provides more details about the AWS DeepRacer vehicle, training and evaluating models, and more.
1. Read [Getting started with AWS DeepRacer](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md). In this tutorial, you learn how to install the latest AWS DeepRacer code and then build and run the AWS DeepRacer application.
1. Explore the [Follow the Leader (FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project). This sample project changes the behavior of the vehicle application. Your vehicle will try to follow you as you move about a room.
1. Explore the [Mapping sample project with ROS Noetic on Ubuntu 20.04](https://github.com/aws-deepracer/aws-deepracer-mapping-sample-project). This sample project uses SLAM with a RealSense™ D435/D435i camera on ROS to map and localize an environment.
1. Learn about the [Robot Operating System (ROS) 2](http://wiki.ros.org/doc/ROS2). The AWS DeepRacer application is based on [ROS 2 Foxy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/). 

## Build your sample project

Create a simple `HelloDeepRacerWorld` example application to understand the basics needed to build your own sample project. Read the [Create your sample project](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/create-your-sample-project.md) guide.

If you create your own project, please email us at deepraceropensource@amazon.com and we’d be happy to feature it on the [Projects](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/projects.md) list.
