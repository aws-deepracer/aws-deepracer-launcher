# Getting Started with DeepRacer OpenSource

The DeepRacer device has all the required libraries and packages preinstalled to run the core application. We recommend you build the ROS2 packages on the device when 
changing core packages or building new packages. This will make it easier to develop, 
debug, and test the application.

If your device is not upgraded to the latest software stack with Ubuntu 20.04 and ROS2 
Foxy, you can follow the instructions [here](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-vehicle-factory-reset-instructions.html) to flash the device with upgraded software 
stack. After flashing the device with the new image, the software stack should be 
updated to **Ubuntu 20.04 Focal Fossa**, **Intel® OpenVINO™ toolkit 2021.1.110**, 
**ROS2 Foxy Fitzroy** and **Python 3.8**.

The AWS DeepRacer device with the upgraded OpenSource software stack contains the 
following software preinstalled:

1. **ROS2 Foxy and dependencies:** The DeepRacer device has the ROS2 Foxy 
(**ros-foxy-desktop**), CV Bridge(**ros-foxy-cv-bridge**), image transport 
(**ros-foxy-image-transport**), standard messages (**ros-foxy-std-msgs**) and sensor 
messages (**ros-foxy-sensor-msgs**) installed. For more information about
installing ROS2 Foxy, see [Installing ROS 2 via Debian Packages](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/).

1. **Intel OpenVino toolkit and its dependencies:** The AWS DeepRacer core application 
uses Intel OpenVino to optimize the reinforcement learning models and run the inference 
on them. As part of the preinstalled device software, we have installed the Intel 
OpenVino toolkit (**intel-openvino-dev-ubuntu20-2021.1.110**) and its dependencies. 
More details about the steps to install the Intel OpenVino toolkit can be found here 
(https://docs.openvinotoolkit.org/latest/openvino_docs_install_guides_installing_openvino_linux.html).

1. **Tensorflow:** We have installed a custom built **Tensorflow v2.4.0** wheel with no 
AVX support to work on the AWS DeepRacer device. 

1. **AWS DeepRacer packages**:
    1. **aws-deepracer-sample-models** - This debian package installs the three sample 
    models deployed by default on the AWS DeepRacer device.
    2. **aws-deepracer-util** - This debian package installs the utility scripts 
    required by the AWS DeepRacer core application.
    3. **aws-deepracer-device-console** - This debian package installs the UI component 
    of the AWS DeepRacer device console.
    4. **aws-deepracer-core** - This debian package installs all the ROS2 Foxy packages 
    required to provide the functionalities for AWS DeepRacer core application. 

Before you pull the changes from the GitHub repositories, make sure you have the device setup completed and have the latest version of the AWS DeepRacer. For more information about the device setup, connecting to Wi-Fi and software update system, see [Getting Started](https://aws.amazon.com/deepracer/getting-started/) quick start guide for the AWS DeepRacer and AWS DeepRacer Evo devices.

#### Installing the AWS DeepRacer debian packages using CLI:

To install the latest AWS DeepRacer pacakges from the apt repository, open up a terminal on the DeepRacer device and run the following commands:

1. Update the apt cache:
        
        sudo apt-get update

1. Install the latest aws-deepracer-* packages:

        sudo apt-get install aws-deepracer-*


## DeepRacer Core Components

The DeepRacer application includes packages for perception, decision, navigation and general application support:

**Perception**

* [camera_node](https://github.com/aws-racer/aws-deepracer-camera-pkg) - reads and publishes data from the camera. 
* [rplidar_node](https://github.com/youngday/rplidar_ros2) - an open source package that supports reading and publishing LiDAR data from the connected RPLidar device.
* [sensor_fusion_node](https://github.com/aws-racer/aws-deepracer-sensor-fusion-pkg) - combines and publishes camera and LiDAR data (if one is present) as a single message.

**Decision**

* [inference_node](https://github.com/aws-racer/aws-deepracer-inference-pkg) - runs inference on the selected machine learning model.

**Navigation**

* [deepracer_navigation_node](https://github.com/aws-racer/aws-deepracer-navigation-pkg) - collects model inference results and publishes a servo message with throttle and steering angle values based on the action space for the selected machine learning model.
* [servo_node](https://github.com/aws-racer/aws-deepracer-servo-pkg) - maps the input servo throttle and servo angle ratios to raw PWM values thats set on the servo/motor to move the vehicle.

**General Application Support**

* [async_web_server_cpp](https://github.com/GT-RAIL/async_web_server_cpp) - an open source package that supports streaming display_mjpeg and overlay_mjpeg to the front end as a camera stream.
* [ctrl_node](https://github.com/aws-racer/aws-deepracer-ctrl-pkg) - as the main node, it exposes services used by webserver backend API calls.
* [software_update_node](https://github.com/aws-racer/aws-deepracer-systems-pkg) - responsible for the software update system managing the aws-deepracer-core, aws-deepracer-util, aws-deepracer-sample-models, and aws-deepracer-webserver packages.
* [model_loader_node](https://github.com/aws-racer/aws-deepracer-systems-pkg) - responsible for extracting tar.gz model files from USB and those uploaded from console.
* [otg_control_node](https://github.com/aws-racer/aws-deepracer-systems-pkg) - responsible to enable and disable OTG connection and publish connection status.
* [network_monitor_node](https://github.com/aws-racer/aws-deepracer-systems-pkg) - responsible to connect to Wi-Fi based on configuration file on USB and communicate the network connection status.
* [device_info_node](https://github.com/aws-racer/aws-deepracer-device-info-pkg) - provides hardware and software version information.
* [battery_node](https://github.com/aws-racer/i2c_pkg) - reads and publishes vehicle battery level information.
* [model_optimizer_node](https://github.com/aws-racer/aws-deepracer-model-optimizer-pkg) - runs the model optimizer on the selected machine learning model.
* [status_led_node](https://github.com/aws-racer/aws-deepracer-status-led-pkg) - controls blinking and solid light functionality for the status LEDs found on the side of the DeepRacer device. 
* [usb_monitor_node](https://github.com/aws-racer/aws-deepracer-usb-monitor-pkg) - monitors USB connections and published a notification if a required file is found. 
* [web_video_server](https://github.com/RobotWebTools/web_video_server/pull/111) - an open source package that supports streaming the images from a topic through the webserver to the front end.
* [webserver_publisher_node](https://github.com/aws-racer/aws-deepracer-webserver-pkg) - a collection of FLASK APIs called from the front end. These APIs call ROS services and return results to the front end.

## Installing build tools

To install the required build tools, open up a terminal on the DeepRacer device and run the following commands as root user.

1. Install rosinstall if not installed previously:

        apt install python3-rosinstall

1. Initializing rosdep if its not initialized previously:

        rosdep init
        rosdep update

1. We use colcon to build the AWS DeepRacer Core ROS2 packages. More 
details to install and use colcon can be found [here](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#colcon).

        apt install python3-colcon-common-extensions -y

## Building the core packages

To build the core packages on the device, open up a terminal on the DeepRacer device and run the following commands as root user.

1. Stop the deepracer-core.service that is currently running on the device:

        systemctl stop deepracer-core

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the deepracer_launcher package on the DeepRacer device:

        git clone https://github.com/aws-racer/aws-deepracer-launcher.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-launcher
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_ws && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the core packages:

        cd ~/deepracer_ws && colcon build

## Running the latest packages

To launch the built packages on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Navigate to the deepracer workspace:

        cd ~/deepracer_ws

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/install/setup.bash  

1. Launch all the required nodes using the main launch script:

        ros2 launch deepracer_launcher deepracer_launcher.py


## Updating the deepracer-core service:

To ensure the latest packages run when DeepRacer device is booted, update the existing packages from */opt/aws/deepracer/lib* folder with the new packages. Run the following commands to replace the existing installation with the newly built packages.


1. Navigate to the deepracer workspace:

        cd ~/deepracer_ws

1. Copy the newly built files to the */opt/aws/deepracer/lib* folder:

        cp -r ./ /opt/aws/deepracer/lib

1. Restart the deepracer-core.service:

        systemctl restart deepracer-core

Once you have the updated software installed on the device, you should be able to run the AWS DeepRacer core application and modify the existing nodes and packages to suit your needs.

For more information about how the core application works in different
modes, see [Modes of Operation](modes-of-operation.md).