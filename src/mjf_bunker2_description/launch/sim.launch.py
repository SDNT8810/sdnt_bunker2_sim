
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    bnk_description_prj = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('mjf_bunker2_description'), 'launch'),
         '/bunker_mjf.launch.py'])
    )

    slam_toolbox_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('slam_toolbox'), 'launch'),
         '/online_async_launch.py']),
      launch_arguments={
          "use_sim_time": "True",
          "autostart": "True",
          "odom_frame": "odom",
          "map_frame": "map",
          "base_frame": "base_link",
          "scan_topic": "/scan",
          "map_update_interval": "1.0",
          "max_laser_range": "30",
          "minimum_travel_distance": "0.1",
          "use_scan_matching": "true",
          "minimum_travel_heading": "1.57",
          "do_loop_closing": "true",
          "slam_methods": "karto",
          "pub_map_odom_transform": "true",
# karto , hector , gmapping
      }.items(),
    )

    return LaunchDescription([
      DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
      bnk_description_prj,
      slam_toolbox_node,
    ])
