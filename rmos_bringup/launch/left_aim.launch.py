import os

from unicodedata import name

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch.conditions import IfCondition
import yaml

def generate_launch_description():



    # add node
    daheng_node_l = Node(
        package='rmos_cam_l',
        namespace='rmos_cam_l',
        executable='daheng_camera_l',
        name='daheng_camera_l',
        output='screen',
    )
    basic_armor_detector_node_l = Node(
        package='rmos_detector_l',
        namespace= 'rmos_detector_l',
        executable='basic_detector_l',
        name='basic_detector_l',
        output='screen',
    )
    
   # dl_armor_detector_node_ = Node(
   #     package='rmos_detector',
   #     namespace= 'rmos_detector',
   #     executable='dl_detector',
   #     name='dl_detector',
   #     output='screen',

    #)

   # communicate_node_ = Node(
       # package='rmos_transporter',
       # namespace= 'rmos_transporter',
       # executable='can_comm',
      #  name='can_comm',
     #   output='screen',
    processer_node_l = Node(
        package='rmos_processer_l',
        namespace= 'rmos_processer_l',
        executable='processer_l',
        name='processer_l',
        output='screen',
    )
    communicate_node_l = Node(
        package='rmos_transporter_l',
        namespace= 'rmos_transporter_l',
        executable='usb_comm_l',
        name='usb_comm_l',
        parameters=[os.path.join(get_package_share_directory('rmos_bringup'), 
                    'configure', 'hardware_settings_l.yaml')],
         output='screen',
     )

    # Done
    return LaunchDescription([

        # dl_armor_detector_node_,
        daheng_node_l,
        basic_armor_detector_node_l,
        communicate_node_l,
        processer_node_l,

    ])
