from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import number

from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# https://github.com/ros-drivers/ros2_ouster_drivers/blob/eloquent-devel/ros2_ouster/launch/os1_launch.py
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
import lifecycle_msgs.msg

# https://answers.ros.org/question/306935/ros2-include-a-launch-file-from-a-launch-file/

# https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/launch/zed.launch.py
# Also need to add config to the CMakeLists.txt install(DIRECTORY
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    laser_node = Node(
        package="xv_11_laser_driver",
        executable="neato_laser_publisher",
        output="screen",
        emulate_tty=True,
        parameters=[{"port": "/dev/neato_laser"},
                    {"baud_rate": 115200},
                    {"frame_id": "laser"},
                    {"firmware_number": 2}
        ]
    )
    
    static_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_laser_tf',
        arguments=['0', '0', '0', '1.57', '0', '0', 'base_link', 'laser']
    )
    
    ld.add_action(laser_node)
    ld.add_action(static_laser_tf_node)
    
    return ld
