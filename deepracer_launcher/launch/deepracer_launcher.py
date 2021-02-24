#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    camera_node = Node(
        package='camera_pkg',
        namespace='camera_pkg',
        executable='camera_node',
        name='camera_node'
    )
    ctrl_node = Node(
        package='ctrl_pkg',
        namespace='ctrl_pkg',
        executable='ctrl_node',
        name='ctrl_node'
    )
    deepracer_navigation_node = Node(
        package='deepracer_navigation_pkg',
        namespace='deepracer_navigation_pkg',
        executable='deepracer_navigation_node',
        name='deepracer_navigation_node'
    )
    software_update_node = Node(
        package='deepracer_systems_pkg',
        namespace='deepracer_systems_pkg',
        executable='software_update_node',
        name='software_update_node'
    )
    model_loader_node = Node(
        package='deepracer_systems_pkg',
        namespace='deepracer_systems_pkg',
        executable='model_loader_node',
        name='model_loader_node'
    )
    otg_control_node = Node(
        package='deepracer_systems_pkg',
        namespace='deepracer_systems_pkg',
        executable='otg_control_node',
        name='otg_control_node'
    )
    network_monitor_node = Node(
        package='deepracer_systems_pkg',
        namespace='deepracer_systems_pkg',
        executable='network_monitor_node',
        name='network_monitor_node'
    )
    device_info_node = Node(
        package='device_info_pkg',
        namespace='device_info_pkg',
        executable='device_info_node',
        name='device_info_node'
    )
    battery_node = Node(
        package='i2c_pkg',
        namespace='i2c_pkg',
        executable='battery_node',
        name='battery_node'
    )
    inference_node = Node(
        package='inference_pkg',
        namespace='inference_pkg',
        executable='inference_node',
        name='inference_node'
    )
    model_optimizer_node = Node(
        package='model_optimizer_pkg',
        namespace='model_optimizer_pkg',
        executable='model_optimizer_node',
        name='model_optimizer_node'
    )
    rplidar_node = Node(
        package='rplidar_ros',
        namespace='rplidar_ros',
        executable='rplidarNode',
        name='rplidarNode',
        parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }]
    )
    sensor_fusion_node = Node(
        package='sensor_fusion_pkg',
        namespace='sensor_fusion_pkg',
        executable='sensor_fusion_node',
        name='sensor_fusion_node'
    )
    servo_node = Node(
        package='servo_pkg',
        namespace='servo_pkg',
        executable='servo_node',
        name='servo_node'
    )
    status_led_node = Node(
        package='status_led_pkg',
        namespace='status_led_pkg',
        executable='status_led_node',
        name='status_led_node'
    )
    usb_monitor_node = Node(
        package='usb_monitor_pkg',
        namespace='usb_monitor_pkg',
        executable='usb_monitor_node',
        name='usb_monitor_node'
    )
    webserver_publisher_node = Node(
        package='webserver_pkg',
        namespace='webserver_pkg',
        executable='webserver_publisher_node',
        name='webserver_publisher_node'
    )
    web_video_server_node = Node(
        package='web_video_server',
        namespace='web_video_server',
        executable='web_video_server',
        name='web_video_server'
    )
    ld.add_action(camera_node)
    ld.add_action(ctrl_node)
    ld.add_action(deepracer_navigation_node)
    ld.add_action(software_update_node)
    ld.add_action(model_loader_node)
    ld.add_action(otg_control_node)
    ld.add_action(network_monitor_node)
    ld.add_action(device_info_node)
    ld.add_action(battery_node)
    ld.add_action(inference_node)
    ld.add_action(model_optimizer_node)
    ld.add_action(rplidar_node)
    ld.add_action(sensor_fusion_node)
    ld.add_action(servo_node)
    ld.add_action(status_led_node)
    ld.add_action(usb_monitor_node)
    ld.add_action(webserver_publisher_node)
    ld.add_action(web_video_server_node)
    return ld
