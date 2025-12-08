from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get path to this package
    pkg_share = get_package_share_directory('my_robot_description')

    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'biped_s40.urdf')

    # Read URDF into a string
    with open(urdf_file, 'r') as inf:
        robot_description = inf.read()

    # Joint state publisher with GUI sliders
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher'
    )

    # Robot state publisher publishes TF and /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Start RViz2 without predefined config
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz
    ])
