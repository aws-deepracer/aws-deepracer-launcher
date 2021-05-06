# Creating your sample project 

The AWS DeepRacer device software is now open source to enable quick prototyping and development of your own customized robotic applications by reusing or taking inspiration from the existing AWS DeepRacer core application, sample projects, or community projects. The AWS DeepRacer core application and the sample projects are built as a collection of **Robot Operating System (ROS)** packages and nodes running on the base **Ubuntu 20.04 Focal Fossa** image. The AWS DeepRacer device comes with all the prerequisite packages and libraries installed for building and running new applications developed on the ROS 2 Foxy distribution. More details about the  preinstalled set of packages and libraries on the AWS DeepRacer device and installing required build systems can be found in the [Getting Started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer open-source page. For a detailed list of tutorials with step-by-step instructions to build skills in ROS 2, see the ROS 2 [Tutorials](https://docs.ros.org/en/foxy/Tutorials.html).

This document can be used as a quick onboarding tutorial with high-level instructions to build your own sample application on the AWS DeepRacer device. A basic understanding of the ROS2 framework and associated build systems is required to develop new packages and nodes. The sample project is built as a collection of existing and newly built ROS Nodes; implementing your own idea for a sample project would typically follow these steps:

* Learn about the basic building blocks of a ROS application, such as the workspace, packages, nodes, topic, and service.
* Design your application to use various existing ROS packages and nodes built by the open-source ROS community or develop your own ROS nodes.
* Build and test the nodes individually and then as an end-to-end application to verify the functionality.  

## Using the ROS2 CLI

Explore the following resources to learn about using the ROS 2 CLI:
* [Understanding the ROS 2 CLI](https://docs.ros.org/en/foxy/Concepts/About-Command-Line-Tools.html)
* [Cheat sheet](https://github.com/ubuntu-robotics/ros2_cheats_sheet/blob/master/cli/cli_cheats_sheet.pdf)

## Steps to create your own sample project
Follow these steps to create your own sample project.

### Create a package and ROS nodes:

As a first step in creating your ROS application, you will have to create a ROS workspace, create ROS packages in the workspace, and start adding ROS nodes to the packages. If you wish to use or modify a certain AWS DeepRacer core package node, you can do so by directly adding those nodes in your workspace so that you can build them with your sample project. The [AWS DeepRacer Follow the Leader (FTL)](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project) sample project is an example in which the application uses some packages as-is from the core application, modifies some of the packages, and adds new packages as part of the implementation.

ROS uses packages to organize its programs. You can think of a package as all the files that a specific ROS program contains: all its CPP files, Python files, configuration files, compilation files, launch files, and parameters files. All those files in the package are organized with the following structure:

```
|_ launch: Folder contains launch files
|_ src: Folder with source files (CPP, Python)
|_ CMakeLists.txt: List of cmake rules for compilation
|_ package.xml: Package information and dependencies
```

### Edit the ROS node:

You can now edit the ROS node to add the functionality you desire. While developing on the ROS node, try to follow the structure of existing ROS nodes so the packages you develop are consistent with the existing ROS nodes. Also make sure to add [logging](https://docs.ros.org/en/foxy/Concepts/About-Logging.html) at appropriate places so that it's easier to debug at a later stage.

### Build nodes:

ROS uses [`colcon`](https://colcon.readthedocs.io/en/released/user/how-to.html) as a build system to build the nodes and packages that are created. For more information about using the `colcon` commands to build ROS packages, see [Using colcon to build packages](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html). The instructions on AWS DeepRacer GitHub pages should also provide examples demonstrating how to use `colcon` to build the ROS nodes. 

### Run the sample project:

All the nodes that are created as part of the sample project can be launched simultaneously using a ROS launcher. For more information about launching and monitoring multiple nodes using a ROS launcher, see [Launching/monitoring multiple nodes with Launch](https://docs.ros.org/en/foxy/Tutorials/Launch-system.html). After building all the required nodes for the sample project in your workspace, open a fresh terminal and run the launch script you created. 

Depending on how your sample project leverages the existing AWS DeepRacer core application, you may be required to stop the existing DeepRacer core service using this [guide](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md#startstop-the-deepracer-core-service). It is important to stop the service if your sample project reuses a node from the AWS DeepRacer core application which could conflict with the functionality implemented in it. 

For an example demonstrating how to launch a sample project, see the GitHub instructions for the [AWS DeepRacer Follow the Leader (FTL)](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/tree/main/deepracer_follow_the_leader_ws/ftl_launcher) sample project.

### Test:

First, make sure that the ROS nodes you developed or customized for your use case are working as expected and that the pipeline of ROS topics and services works as an end-to-end application. In many cases, ROS topics and service calls are used for the inter-node communication. We recommend testing each and every ROS node that you have modified or developed by checking the inputs and outputs of the ROS topics and services implemented in those nodes.

**ROS topics:**

* Nodes use topics to publish information for other nodes so that they can communicate.
* Topics handle information through messages. Messages are defined in **.msg** files, which are located inside a `msg` directory of a package. 
* A publisher node publishes messages to a certain topic, while a subscriber node subscribes to a topic to receive the messages.
* To learn more, see [Understanding ROS 2 topics](https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html#tasks).

**ROS Services:**

* Services, another method of communication between ROS nodes, are based on a request call-and-response model, in contrast to a topic's publisher-subscriber model.
* Service messages have the extension `.srv` and are located inside a `srv` directory.
* To learn more, see [Understanding ROS 2 topics](https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html#tasks).

ROS also provides ROS CLI commands to test messages and services ([subscribe](https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html#ros2-topic-echo) to the ROS topics (if any) or initiate the [services](https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html#ros2-service-call) (if any) using the CLI). Once you have confidence about the individual nodes, you can check if the pipeline of the nodes you created works as expected end-to-end by launching all the nodes.


## Create an example project

As an example, here are the step-by-step instructions to create your first sample project. This sample project has a basic functionality to blink the tail light with a red-blue siren effect for 5 seconds at the start of the application.

1. Open a terminal on the AWS DeepRacer device (or [SSH into the AWS DeepRacer device](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md#optional-use-ssh-to-remotely-connect-to-the-deepracer-device)): Follow these instructions to open an Ubuntu terminal window on the AWS DeepRacer device:
    1. Connect your AWS DeepRacer device to a monitor. You'll need a HDMI-to-HDMI, HDMI-to-DVI, or similar cable. Insert one end of the cable into the HDMI port on the vehicle's chassis and plug the other end into a supported display port on the monitor.
    1. Connect a USB keyboard to your AWS DeepRacer using the USB port on the device's compute module, after the compute module is booted.
    1. In the **Username** field, enter `deepracer`.
    1. In the **Password** field, enter the device SSH password. If this is your first time logging in to the device, enter `deepracer` in the **Password** field. Reset the password, as required, before moving to the next step. You use the new password for future logins. For security reasons, use a complex or strong password phrase for the new password.
    1. After you're logged in, open a terminal window. You can use **Search** button to find the terminal window application.
1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. For this example project, we depend on the nodes that are running as part of the AWS DeepRacer core application, so we need to source the AWS DeepRacer core application setup bash script:

        source /opt/aws/deepracer/lib/setup.bash

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Create a package and ROS node: Create a new  `hello_deepracer_package` package and a `hello_deepracer_node` Python node using the ROS 2 CLI. For more details about creating your own ROS package and nodes, see [Creating your first ROS 2 package](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html).

        ros2 pkg create --build-type ament_python --node-name hello_deepracer_node hello_deepracer_package

    ```going to create a new package
        package name: hello_deepracer_package
        destination directory: /root/deepracer_ws
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <root@todo.todo>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: []
        node_name: hello_deepracer_node
        creating folder ./hello_deepracer_package
        creating ./hello_deepracer_package/package.xml
        creating source folder
        creating folder ./hello_deepracer_package/hello_deepracer_package
        creating ./hello_deepracer_package/setup.py
        creating ./hello_deepracer_package/setup.cfg
        creating folder ./hello_deepracer_package/resource
        creating ./hello_deepracer_package/resource/hello_deepracer_package
        creating ./hello_deepracer_package/hello_deepracer_package/__init__.py
        creating folder ./hello_deepracer_package/test
        creating ./hello_deepracer_package/test/test_copyright.py
        creating ./hello_deepracer_package/test/test_flake8.py
        creating ./hello_deepracer_package/test/test_pep257.py
        creating ./hello_deepracer_package/hello_deepracer_package/hello_deepracer_node.py
    ```

    The preceding command creates the `hello_deepracer_node.py`, as shown in the following code snippet: 

    ``` 
        def main():
            print('Hi from hello_deepracer_package.')
        
        
        if __name__ == '__main__':
            main()
    ```

1. Edit the ROS node file to add the basic functionality to blink the tail lights with a red-blue siren effect for 5 seconds. Add a ROS client to call the `servo_pkg/set_led_state` service and pass the required color information as part of the request object. For more details about implementing your own simple publisher/subscriber system, see [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html#pypubsub). For more ways you could write a publisher and subscriber, see [Writing a simple service and client (Python)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Service-And-Client.html#pysrvcli).



    Replace the `hello_deepracer_node.py` file content with the following code:


    ```
    import rclpy
    from rclpy.node import Node
        
    from deepracer_interfaces_pkg.srv import SetLedCtrlSrv
        
        
    class HelloDeepRacerWorld(Node):

        def __init__(self):
            super().__init__('hello_deepracer_node')
            
            # Service to dynamically set LED COLOR.
            self.set_led_ctrl_client = self.create_client(
                                            SetLedCtrlSrv,
                                            "servo_pkg/set_led_state"
                                        )

            while not self.set_led_ctrl_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f"{self.set_led_ctrl_client.srv_name} service not available, waiting again..."
                )

            self.color_map = [(0, 0, 255), (255, 0, 0)] # Blue and Red
            self.led_scaling_factor = 39215

            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.counter = 1
                                

        def timer_callback(self):
            self.get_logger().info(f"{self.counter}/10: Setting the tail light LED color values to {self.color_map[self.counter%2]}")
            r, g, b = self.color_map[self.counter%2]
            self.set_tail_light_led_color(r, g, b)
            self.counter += 1
            if self.counter >10:
                self.get_logger().info("Stopping the timer.")
                self.destroy_timer(self.timer)

        def set_tail_light_led_color(self, r, g, b):
            set_led_color_req = SetLedCtrlSrv.Request()
            set_led_color_req.red = r * self.led_scaling_factor
            set_led_color_req.green = g * self.led_scaling_factor
            set_led_color_req.blue = b * self.led_scaling_factor
            self.set_led_ctrl_client.call_async(set_led_color_req)


    def main(args=None):
        rclpy.init(args=args)

        hello_deepracer_node = HelloDeepRacerWorld()

        rclpy.spin(hello_deepracer_node)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        hello_deepracer_node.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()
    ```


### Creating the client
In order to create the client to call the `servo_pkg/set_led_state` service, import the `SetLedCtrlSrv` service interface:

    ```
    import rclpy
    from rclpy.node import Node

    from deepracer_interfaces_pkg.srv import SetLedCtrlSrv
    ```

    As part of the initialization of the node, we wait_for_service to ensure the `servo_pkg/set_led_state` service is available:

    ```
        def __init__(self):
            super().__init__('hello_deepracer_node')
            
            # Service to dynamically set LED COLOR.
            self.set_led_ctrl_client = self.create_client(
                                            SetLedCtrlSrv,
                                            "servo_pkg/set_led_state"
                                        )

            while not self.set_led_ctrl_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f"{self.set_led_ctrl_client.srv_name} service not available, waiting again..."
                )
    ```

    Initialize the information required to populate the request object and call the set led service:

    ```
            self.color_map = [(0, 0, 255), (255, 0, 0)] # Blue and Red
            self.led_scaling_factor = 39215
    ```

    Create a ROS timer to periodically call the service and create a blink effect:

    ```
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.counter = 1
    ```

    Create a function to create a request object and call the  `servo_pkg/set_led_state` service:

    ```
        def set_tail_light_led_color(self, r, g, b):
            set_led_color_req = SetLedCtrlSrv.Request()
            set_led_color_req.red = r * self.led_scaling_factor
            set_led_color_req.green = g * self.led_scaling_factor
            set_led_color_req.blue = b * self.led_scaling_factor
            self.set_led_ctrl_client.call_async(set_led_color_req)
    ```

    Finally, in the timer callback, create the blink effect by calling the set_tail_light_led_color function with the right color values for 10 iterations before destroying the timer:

    ```
        def timer_callback(self):
            self.get_logger().info(f"{self.counter}/10: Setting the tail light LED color values to {self.color_map[self.counter%2]}")
            r, g, b = self.color_map[self.counter%2]
            self.set_tail_light_led_color(r, g, b)
            self.counter += 1
            if self.counter >10:
                self.get_logger().info("Stopping the timer.")
                self.destroy_timer(self.timer)
    ```

1. Build the ROS node: Build the packages in the workspace by running the `colcon` command. For more details about `colcon` build system and various parameters that can be passed, run the following command:

        colcon build

1. Test the ROS node: In a different terminal, navigate to the workspace and run the built node.
    1. Switch to the root user before you source the ROS 2 installation:
        
            sudo su

    1. Source the ROS 2 Foxy setup bash script:
    
            source /opt/ros/foxy/setup.bash
    
    1. Source the setup script for the installed packages:
            
            source ~/deepracer_ws/install/setup.bash

    1. Run the node:
            
            ros2 run hello_deepracer_package hello_deepracer_node

    1. The output should be similar to the following example:

        ```
        root@amss-5su5:~/deepracer_ws# ros2 run hello_deepracer_package hello_deepracer_node
        [INFO] [1618261667.005110450] [hello_deepracer_node]: 1/10: Setting the tail light LED color values to (255, 0, 0)
        [INFO] [1618261667.439754926] [hello_deepracer_node]: 2/10: Setting the tail light LED color values to (0, 0, 255)
        [INFO] [1618261667.939710723] [hello_deepracer_node]: 3/10: Setting the tail light LED color values to (255, 0, 0)
        [INFO] [1618261668.439688996] [hello_deepracer_node]: 4/10: Setting the tail light LED color values to (0, 0, 255)
        [INFO] [1618261668.939659463] [hello_deepracer_node]: 5/10: Setting the tail light LED color values to (255, 0, 0)
        [INFO] [1618261669.439667901] [hello_deepracer_node]: 6/10: Setting the tail light LED color values to (0, 0, 255)
        [INFO] [1618261669.939656696] [hello_deepracer_node]: 7/10: Setting the tail light LED color values to (255, 0, 0)
        [INFO] [1618261670.439707052] [hello_deepracer_node]: 8/10: Setting the tail light LED color values to (0, 0, 255)
        [INFO] [1618261670.939676975] [hello_deepracer_node]: 9/10: Setting the tail light LED color values to (255, 0, 0)
        [INFO] [1618261671.439648711] [hello_deepracer_node]: 10/10: Setting the tail light LED color values to (0, 0, 255)
        [INFO] [1618261671.441297952] [hello_deepracer_node]: Stopping the timer.
        ```

To understand how to create, build, and run your own sample project, see the [aws-deepracer-follow-the-leader-sample-project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project). 

We recommend that you clone the required packages and create a new repository combining all the different ROS nodes as inn the [Follow the Leader (FTL)](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project) repository.

## Troubleshooting and debugging guidelines

* **What if your vehicle isn’t operating or moving the way you want it to?**

    Follow the [instructions](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-calibrate-vehicle.html) to calibrate the mechanics of your AWS DeepRacer Vehicle. This should be done so that the vehicle performance is optimal and it behaves as expected. 
* **Is there a runtime log? How can I configure the ROS 2 logger to help in debugging?**

    All the logs for the ROS nodes that are running by default are streamed to the `/var/log/syslog` file. If you're running your nodes directly on the CLI, then the logs are streamed to the `stdout` on the terminal. To learn more about logging and logger configuration, see [About logging and logger configuration](https://docs.ros.org/en/foxy/Concepts/About-Logging.html).


## FAQ

* **How do I use SSH to connect remotely to the AWS DeepRacer device?**

    * The **Settings** page in the AWS DeepRacer device console provides an interface to enable the SSH server on the AWS DeepRacer device. After enabling the SSH on the device, it is possible to remotely log in to AWS DeepRacer via the CLI from your local system and execute commands. For more information about the **Settings** page, see [Inspect and Manage Your AWS DeepRacer Vehicle Settings](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-manage-vehicle-settings.html).
    * After enabling the SSH server, remotely connect to the AWS  DeepRacer (the local system should be on the same WiFi as the AWS DeepRacer device):
    
            ssh deepracer@<<IP_ADDRESS>>
    
* **How do I disable a node from the default core service?**

    Commenting the `add` action in the AWS DeepRacer launcher [launch file](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/deepracer_launcher/launch/deepracer_launcher.py) should allow you to disable or stop creating a node. 
    * [Stop](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md#startstop-the-deepracer-core-service) the core service.
    * Comment the `add` action line in the launcher file located at the `/opt/aws/deepracer/lib/deepracer_launcher/launch/deepracer_launcher.py` location.
    * [Restart](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md#startstop-the-deepracer-core-service) the core service.

* **How can I power up the steering servo channel?** 

    The steering and speed motors are powered by the vehicle (blue/black) battery. You have to keep it turned on if you need to  test the steering or throttle changes.

* **Do I need to run `colcon build` after every  configuration change? If so, can I specify a single node for rebuilding?**

    Yes, you need to run `colcon build` after every change. The build times are shorter if there are no major changes.  There are also ways you can build the packages that are modified and retain the others to reduce build time (`--symlink-install`). For more information, see [How to](https://colcon.readthedocs.io/en/released/user/how-to.html) and [Build the workspace with colcon](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html#build-the-workspace-with-colcon).
