import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    maps_folder = get_share_file("bvs_common", "maps")
    map_names = ["left", "right"]
    map_paths = [ 
        # maps_folder + "/lvms_centerline.csv",
        # maps_folder + "/lvms_leftcenterline_4.csv",
        maps_folder + "/lvms_1m_racingline.csv",
        maps_folder + "/lvms_rightcenterline_4.csv"
    ]
    pit_paths = [ 
        # maps_folder + "/lvms_pitline.csv",
        # maps_folder + "/lvms_leftpitline_4.csv",
        maps_folder + "/lvms_1m_pitline.csv",
        maps_folder + "/lvms_rightpitline_4.csv"
    ]

    ego1_raceline_planner_node = Node(
        name='raceline_planner',
        namespace='/ego1', # Needed for param overwrite to function see: https://githubmemory.com/repo/ros2/launch_ros/issues/233 
        package='raceline_planner',
        executable='raceline_planner_node',
        output='screen',
        parameters=[{
            "raceline_names": map_names,
            "raceline_paths": map_paths,
            "pitline_paths": pit_paths    
        }]
    )

    ego2_raceline_planner_node = Node(
        name='raceline_planner',
        namespace='/ego2', # Needed for param overwrite to function see: https://githubmemory.com/repo/ros2/launch_ros/issues/233 
        package='raceline_planner',
        executable='raceline_planner_node',
        output='screen',
        parameters=[{
            "raceline_names": map_names,
            "raceline_paths": map_paths,
            "pitline_paths": pit_paths    
        }]
    )

    ego1_dummy_sim_node = Node(
        name='dummy_sim_node',
        namespace='/ego1', # Needed for param overwrite to function see: https://githubmemory.com/repo/ros2/launch_ros/issues/233 
        package='raceline_planner',
        executable='dummy_car_sim',
        output='screen',
        parameters=[{
            "dummy_robot_path": maps_folder + "/lvms_leftcenterline_4.csv",
            "detection_prefix": "/ego2",
            "ego_start_ltp_x": 800.,
            "ego_start_ltp_y": -700.,
        }]
    )

    ego2_dummy_sim_node = Node(
        name='dummy_sim_node',
        namespace='/ego2', # Needed for param overwrite to function see: https://githubmemory.com/repo/ros2/launch_ros/issues/233 
        package='raceline_planner',
        executable='dummy_car_sim',
        output='screen',
        parameters=[{
            "dummy_robot_path": maps_folder + "/lvms_leftcenterline_4.csv",
            "detection_prefix": "/ego1",
            "ego_start_ltp_x": 800.,
            "ego_start_ltp_y": -800.,
        }]
    )


    return LaunchDescription([
        # raceline_planner_node,
        # dummy_sim_node # don't launch dummy sim node by default. Should probably use a flag or two different launch files instead.
        ego1_dummy_sim_node,
        ego2_dummy_sim_node,
        ego1_raceline_planner_node,
        ego2_raceline_planner_node
    ])
