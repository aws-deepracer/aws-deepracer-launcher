# AWS DeepRacer sample projects

## 1. Follow the Leader

*By the AWS DeepRacer team*

Learn to deploy an object-detection model that enables the AWS DeepRacer device to identify and follow an object. Extend this sample project by modifying the code to recognize other objects for your use case. Bring your own custom model, navigation logic, and add-on hardware (optional) to invent your own application.
 
**Project ideas:** Build fun applications (use the AWS DeepRacer as a digital pet or follow an object) or industrial prototypes (warehousing, manufacturing)
 
**Device needed:** AWS DeepRacer or AWS DeepRacer Evo

**Recommended add-on:** Intel Neural Compute Stick 2 for faster processing

**[See Instructions on GitHub](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project)**

## 2. Mapping

*By the AWS DeepRacer team*

In this project, use the AWS DeepRacer car to draw a map with SLAM (Simultaneous Localization and Mapping), a technique for creating a map of an environment by estimating a device’s current location as it moves through a space.
 
**Project ideas:** Develop navigation systems for home robots, such as vacuum cleaners, a fleet of mobile warehouse robots, or parking a self-driving car. 
 
**Device needed:** AWS DeepRacer or AWS DeepRacer Evo

**Required add-on:** Intel RealSense camera

**[See Instructions on GitHub](https://github.com/aws-deepracer/aws-deepracer-mapping-sample-project)**

## 3. Off Road

*By the AWS DeepRacer team*

Use a series of QR codes as waypoints to navigate the AWS DeepRacer device around a custom path. Create your own custom path by placing the basic waypoint codes as a sequence along the track or encode additional instructions in them to invent new applications.

**Project ideas:** Build your own track to race or create fun games (get the AWS DeepRacer to find a way out of a maze)

**Device needed:** AWS DeepRacer or AWS DeepRacer Evo

**[See Instructions on GitHub](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project)**



# Community projects


## 1. DeepBlaster

*By [Chris Miller](https://www.linkedin.com/in/chris-miller-6470751/), CEO of Cloud Brigade*

Use object-detection models to identify objects and fire at them using a motorized toy blaster mounted on an AWS DeepRacer device. It extends the AWS DeepRacer hardware with a 3D-printed, servo-controlled rotating turret and custom bracket. An Arduino microcontroller is used to control the rotation servo, blaster flywheels, and ammo feeder motor over a USB/Serial link. A second camera can be added to the turret to allow the AWS DeepRacer device to scan 360 degrees for objects and fire at them.

**Project ideas:** Sending a robot to put out a fire, multi-tasking (navigate and fire a nerf cannon), competitive racing, strategic games.

**Device needed:** AWS DeepRacer or AWS DeepRacer Evo 

**Add-on:** Toy Blaster, Servo Controlled Turret, Arduino Microcontroller, 2nd Camera (optional)

**[See Instructions on GitHub](https://github.com/CloudBrigade/cloudbrigade-deepblaster)**



## 2. DeepDriver

*By [Jochem Lugtenburg](https://www.linkedin.com/in/jochem-lugtenburg-8285b8141/), Developer at Relive, 2019/2020 DeepRacer Championship Cup finalist*

Mimics a real-world car that starts and stops at traffic lights and stop signs. The logic for identifying different colors in traffic signals and detect stop signs was developed by combining various computer vision capabilities, including OpenCV image-processing functions and object-detection machine learning models. Currently the focus is on stop signs and the traffic lights. Possibilities for extension include training the car to stop when a person or animal walks in front of it and detecting and staying within the bounds of a lane. Possibilities for scenarios with multiple AWS DeepRacer devices on a track include driving slower if another AWS DeepRacer is in front of the car or overtaking other cars using the LiDAR sensor.

**Project ideas:** Multi-racer competition, autonomous vehicle, using AI for safety and perception

**Device needed:** AWS DeepRacer or AWS DeepRacer Evo 

**Add-on:** N/A

**[See Instructions on GitHub](https://github.com/jochem725/deepdriver)**


## 3. RoboCat

*By [Martin Paradesi](https://www.linkedin.com/in/msrparadesi/), [Organizer of AWS Machine Learning Community: NYC meetup](https://www.meetup.com/AWS-Machine-Learning-Community-NYC/)*

Community project that leverages OpenCV image-processing capabilities to detect a mouse in the image from an [infrared camera](https://smile.amazon.com/gp/product/B07DWWSWNH/) mounted on the AWS DeepRacer device. The infrared camera allows the RoboCat to detect the mouse in the dark and scare it away using a pre-configured action space with 3 actions (move forward if a mouse is detected, move back after previous action, and remain stationary).

**Project ideas:** Pest control, object tracking, and in-home safety

**Device needed:** AWS DeepRacer or AWS DeepRacer Evo 

**Add-on:** [ELP Camera](https://www.amazon.com/gp/product/B07DWWSWNH/)

**[See Instructions on GitHub](https://github.com/msrparadesi/robocat_ws)**
