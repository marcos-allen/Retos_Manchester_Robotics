import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # USER VARIABLES
    package = 'motor_control'
    launch_file = 'motor_launch.py'

    group1_ns = 'group1'
    group2_ns = 'group2'
    group3_ns = 'group3'

    # Get launch file path
    package_directory = get_package_share_directory(package)
    launch_file_path = os.path.join(package_directory, 'launch', launch_file)

    # Create DIFFERENT sources
    launch_source1 = PythonLaunchDescriptionSource(launch_file_path)
    launch_source2 = PythonLaunchDescriptionSource(launch_file_path)
    launch_source3 = PythonLaunchDescriptionSource(launch_file_path)

    # Include launch descriptions
    include1 = IncludeLaunchDescription(launch_source1)
    include2 = IncludeLaunchDescription(launch_source2)
    include3 = IncludeLaunchDescription(launch_source3)

    # Groups with namespaces
    group1 = GroupAction([
        PushRosNamespace(group1_ns),
        include1
    ])

    group2 = GroupAction([
        PushRosNamespace(group2_ns),
        include2
    ])

    group3 = GroupAction([
        PushRosNamespace(group3_ns),
        include3
    ])

    return LaunchDescription([group1, group2, group3])

