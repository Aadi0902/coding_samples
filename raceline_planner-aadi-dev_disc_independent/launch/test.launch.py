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
        
    raceline_planner_node = Node(
        name='raceline_planner',
        namespace='/', # Needed for param overwrite to function see: https://githubmemory.com/repo/ros2/launch_ros/issues/233 
        package='raceline_planner',
        executable='raceline_planner_node',
        output='screen',
        parameters=[{
            "raceline_names": map_names,
            # "raceline_paths": map_paths,
            # "pitline_paths": pit_paths    
            "raceline_topics": ["map_executive/line/race_line/left",
                                "map_executive/line/race_line/right"],
            "pitline_topics": ["map_executive/line/pit_line/left",
                               "map_executive/line/pit_line/right"]
        }]
    )

    ego_start_ltp_x = 535.2617544878366 # LGSVL LVMS 178.6720338436914 # LOR 302.224042097263
    ego_start_ltp_y = -127.68210278095052 # LGSVL LVMS -553.1348929708903 # LOR 9130.685725510128
    opp_start_ltp_x = ego_start_ltp_x
    opp_start_ltp_y = ego_start_ltp_y

    dummy_sim_node = Node(
        name='dummy_sim_node',
        namespace='/', # Needed for param overwrite to function see: https://githubmemory.com/repo/ros2/launch_ros/issues/233 
        package='raceline_planner',
        executable='dummy_car_sim',
        output='screen',
        parameters=[{
            "raceline_names": map_names,
            "ego_start_ltp_x": ego_start_ltp_x,
            "ego_start_ltp_y": ego_start_ltp_y,
            #"opp_start_ltp_x": opp_start_ltp_x,
            #"opp_start_ltp_y": opp_start_ltp_y
        }]
    )

    ghost_node = Node(
        name='ghost_node',
        namespace='/', # Needed for param overwrite to function see: https://githubmemory.com/repo/ros2/launch_ros/issues/233 
        package='raceline_planner',
        executable='ghost_node',
        output='screen',
        parameters=[{
            "detection_prefix_ego":'',
            "raceline_names": map_names,
            "opp_start_ltp_x": opp_start_ltp_x,
            "opp_start_ltp_y": opp_start_ltp_y,
            "opp_vel": 15.,
            "ghost_enable": True,
            "spawn_near_ego": True,
            "spawn_dist": 100.
        }]
    )
    
    return LaunchDescription([
        raceline_planner_node,
        #dummy_sim_node, # don't launch dummy sim node by default. Use a different launch file instead
        #ghost_node
    ])
