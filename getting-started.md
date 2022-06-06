# Getting started with AWS DeepRacer OpenSource

The AWS DeepRacer device has all the required libraries and packages preinstalled to run the core application. We recommend that you build the ROS 2 packages on the device when 
changing core packages or building new packages. This will make it easier to develop, 
debug, and test the application.

If your device is not upgraded to the latest software stack with Ubuntu 20.04 and ROS 2 
Foxy, you can follow the [instructions](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-ubuntu-update.html) to flash the device with the upgraded software 
stack. After flashing the device with the new image, the software stack should be 
updated to **Ubuntu 20.04 Focal Fossa**, **Intel® OpenVINO™ toolkit 2021.1.110**, 
**ROS 2 Foxy Fitzroy**, and **Python 3.8**.

The AWS DeepRacer device with the upgraded OpenSource software stack contains the 
following preinstalled software:

1. **ROS 2 Foxy and dependencies:** The DeepRacer device has the ROS 2 Foxy 
(**ros-foxy-desktop**), CV Bridge(**ros-foxy-cv-bridge**), image transport 
(**ros-foxy-image-transport**), standard messages (**ros-foxy-std-msgs**), and sensor 
messages (**ros-foxy-sensor-msgs**) installed. For more information about
installing ROS 2 Foxy, see [Installing ROS 2 via Debian Packages](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/).

1. **Intel OpenVino toolkit and its dependencies:** The AWS DeepRacer core application 
uses Intel OpenVino to optimize the reinforcement learning models and run inference 
on them. As part of the preinstalled device software, we have installed the Intel 
OpenVino toolkit (**intel-openvino-dev-ubuntu20-2021.1.110**) and its dependencies. 
For more details about the steps to install the Intel OpenVino toolkit, see [Install Intel® Distribution of OpenVINO™ toolkit for Linux](https://docs.openvinotoolkit.org/latest/openvino_docs_install_guides_installing_openvino_linux.html).

1. **Tensorflow:** We have installed a custom built **Tensorflow v2.4.0** wheel with no 
AVX support to work on the AWS DeepRacer device. 

1. **AWS DeepRacer packages**:
    1. **aws-deepracer-sample-models**: This Debian package installs the three sample 
    models deployed by default on the AWS DeepRacer device.
    2. **aws-deepracer-util**: This Debian package installs the utility scripts 
    required by the AWS DeepRacer core application.
    3. **aws-deepracer-device-console**: This Debian package installs the UI component 
    of the AWS DeepRacer device console.
    4. **aws-deepracer-core**: This Debian package installs all the ROS 2 Foxy packages 
    required to provide the functionalities for the AWS DeepRacer core application. 

Before you pull the changes from the GitHub repositories, make sure you have completed the device setup and have the latest version of the AWS DeepRacer. For more information about the device setup, connecting to Wi-Fi, and the software update system, see the [Getting Started](https://aws.amazon.com/deepracer/getting-started/) guide for the AWS DeepRacer and AWS DeepRacer Evo devices.

## Installing the AWS DeepRacer Debian packages using the CLI

To install the latest AWS DeepRacer pacakges from the `apt` repository, open a terminal on the AWS DeepRacer device and run the following commands:

1. Update the `apt` cache:
        
        sudo apt-get update

1. Install the latest aws-deepracer-* packages:

        sudo apt-get install aws-deepracer-*


## AWS DeepRacer core components

The AWS DeepRacer application includes packages for perception, decision, navigation, and general application support.

**Perception**

* [camera_node](https://github.com/aws-deepracer/aws-deepracer-camera-pkg): This node reads and publishes data from the camera. 
* [rplidar_node](https://github.com/Slamtec/rplidar_ros/tree/ros2): This open-source package supports reading and publishing LiDAR data from the connected RPLidar device.
* [sensor_fusion_node](https://github.com/aws-deepracer/aws-deepracer-sensor-fusion-pkg): This node combines and publishes camera and LiDAR data (if one is present) as a single message.

**Decision**

* [inference_node](https://github.com/aws-deepracer/aws-deepracer-inference-pkg): This node runs inference on the selected machine learning model.

**Navigation**

* [deepracer_navigation_node](https://github.com/aws-deepracer/aws-deepracer-navigation-pkg): This node collects model inference results and publishes a servo message with throttle and steering angle values based on the action space for the selected machine learning model.
* [servo_node](https://github.com/aws-deepracer/aws-deepracer-servo-pkg): This node maps the input servo throttle and servo angle ratios to raw PWM values that set on the servo/motor to move the vehicle.

**General application support**

* [async_web_server_cpp](https://github.com/GT-RAIL/async_web_server_cpp): This open-source package supports streaming `display_mjpeg` and `overlay_mjpeg` to the front end as a camera stream.
* [ctrl_node](https://github.com/aws-deepracer/aws-deepracer-ctrl-pkg): As the main node, this exposes services used by webserver backend API calls.
* [software_update_node](https://github.com/aws-deepracer/aws-deepracer-systems-pkg): This node is responsible for the software update system managing the `aws-deepracer-core`, `aws-deepracer-util`, `aws-deepracer-sample-models`, and `aws-deepracer-webserver` packages.
* [model_loader_node](https://github.com/aws-deepracer/aws-deepracer-systems-pkg): This node is responsible for extracting tar.gz model files from USB and those uploaded from the console.
* [otg_control_node](https://github.com/aws-deepracer/aws-deepracer-systems-pkg): This node is responsible for enabling and disabling the OTG connection and publishing the connection status.
* [network_monitor_node](https://github.com/aws-deepracer/aws-deepracer-systems-pkg): This node is responsible for connecting to WiFi based on the configuration file on USB and communicating the network connection status.
* [device_info_node](https://github.com/aws-deepracer/aws-deepracer-device-info-pkg): This node provides hardware and software version information.
* [battery_node](https://github.com/aws-deepracer/aws-deepracer-i2c-pkg): This node reads and publishes vehicle battery level information.
* [model_optimizer_node](https://github.com/aws-deepracer/aws-deepracer-model-optimizer-pkg): This node runs the model optimizer on the selected machine learning model.
* [status_led_node](https://github.com/aws-deepracer/aws-deepracer-status-led-pkg): This node controls blinking and solid light functionality for the status LEDs found on the side of the AWS DeepRacer device. 
* [usb_monitor_node](https://github.com/aws-deepracer/aws-deepracer-usb-monitor-pkg): This node monitors USB connections and publishes a notification if a required file is found. 
* [web_video_server](https://github.com/RobotWebTools/web_video_server/pull/111): This open-source package support streaming the images from a topic through the webserver to the front end.
* [webserver_publisher_node](https://github.com/aws-deepracer/aws-deepracer-webserver-pkg): This node is a collection of FLASK APIs called from the front end. These APIs call ROS services and return results to the front end.


## (Optional) Use SSH to remotely connect to the AWS DeepRacer device

1. The **Settings** page in the AWS DeepRacer device console provides an interface to enable the SSH server on the AWS DeepRacer device. After enabling SSH on the device, you can remotely log in to AWS DeepRacer using the CLI from your local system and execute commands. For more information about the **Settings** page, see [INspect and Manage Your AWS DeepRacer Vehicle Settings](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-manage-vehicle-settings.html).

1. After enabling the SSH server, remotely connect to the AWS DeepRacer device (the local system should be on the same Wi-Fi as the AWS DeepRacer device):

        ssh deepracer@<<IP_ADDRESS>>


## Installing build tools

To install the required build tools, open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before installing and initializing `rosinstall`:

        sudo su

1. Install `rosinstall` if it was not installed previously:

        apt install python3-rosinstall

1. Initialize `rosdep` if it was not initialized previously:

        rosdep init
        rosdep update

1. Use `colcon` to build the AWS DeepRacer Core ROS 2 packages. For more information about installing and using `colcon`, see [Using colcon to build packages](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#colcon).

        apt install python3-colcon-common-extensions -y

## Building the core packages

To build the core packages on the device, open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user:

        sudo su

1. Stop the `deepracer-core.service` that is currently running on the device:

        systemctl stop deepracer-core

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the `deepracer_launcher` package on the AWS DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-launcher.git

1. Clone the `async_web_server_cpp`, `web_video_server`, and `rplidar_ros` dependency packages on the AWS DeepRacer device:

        cd ~/deepracer_ws/aws-deepracer-launcher && ./install_dependencies.sh

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-launcher
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_ws/aws-deepracer-launcher && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the core packages:

        cd ~/deepracer_ws/aws-deepracer-launcher && colcon build

## Running the latest packages

To launch the built packages on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user:

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-launcher/install/setup.bash  

1. Launch all the required nodes using the main launch script:

        ros2 launch deepracer_launcher deepracer_launcher.py


## Updating the `deepracer-core` service:

To ensure the latest packages run when the AWS DeepRacer device is booted, update the existing packages from the `/opt/aws/deepracer/lib` directory with the new packages. Run the following commands to replace the existing installation with the newly built packages.

1. Switch to the root user:

        sudo su

1. Navigate to the AWS DeepRacer workspace:

        cd ~/deepracer_ws/aws-deepracer-launcher/install

1. Copy the newly built files to the `/opt/aws/deepracer/lib` folder:

        cp -r ./ /opt/aws/deepracer/lib

1. Restart the `deepracer-core.service`:

        systemctl restart deepracer-core

Once you have the updated software installed on the device, you should be able to run the AWS DeepRacer core application and modify the existing nodes and packages to suit your needs.

For more information about how the core application works in different
modes, see [Modes of Operation](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/modes-of-operation.md).


## Start or stop the `deepracer-core` service:

To start and stop the `deepracer-core` service for development or debugging, you can run the following commands.

1. To stop the `deepracer-core.service`:

        sudo systemctl stop deepracer-core

1. To restart the `deepracer-core.service`:

        sudo systemctl restart deepracer-core

## Resources

* [Upgrade to latest software stack with Ubuntu 20.04 and ROS 2 
Foxy](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-ubuntu-update.html)
* [AWS DeepRacer Getting Started guide](https://aws.amazon.com/deepracer/getting-started/)
* [AWS DeepRacer device modes of operation](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/modes-of-operation.md)
