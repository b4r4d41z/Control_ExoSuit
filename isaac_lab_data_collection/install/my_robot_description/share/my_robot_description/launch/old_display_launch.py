import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_robot_description'
    pkg_share = get_package_share_directory(package_name)

    urdf_file = os.path.join(pkg_share, 'urdf', 'biped_s40.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        arguments=[
            '0', '0', '0.88',   # x y z
            '0', '0', '0',      # roll pitch yaw
            'world', 'base_link'
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        static_tf,
        rviz
    ])
