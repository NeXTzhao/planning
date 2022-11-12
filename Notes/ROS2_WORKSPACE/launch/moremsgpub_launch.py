'''
Autor: your name
Data: Do not edit
LastEditTime: 2022-04-05 19:04:46
LastEditors: your name
Brief: 
FilePath: /ROS2_WORKSPACE/src/more_interface/launch/moreMsgPub.py
Copyright:  
'''
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      Node(
        package='more_interface',
        executable='publish_address_book',
        output='screen'
    ) 
])