import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.descriptions
from nav2_common.launch import RewrittenYaml
from nav2_common.launch import ReplaceString

# from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
  package_name = 'mjf_bunker2_description'
  # Get the launch directory  
  pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)
  default_model_path = os.path.join(pkg_share, 'src/description/bunker.urdf') # New_Bunker, SDNTbunker , New_Model_description , bunker
  world_path = os.path.join(pkg_share, 'world/my_world.sdf')
  rviz_config_path = os.path.join(pkg_share, 'rviz/display.rviz')

  bringup_dir = get_package_share_directory('nav2_bringup')

  namespace = ""
  
  config_file_dir = os.path.join(get_package_share_directory(package_name), "config")
  
  # Arguments and parameters
  use_sim_time = LaunchConfiguration("use_sim_time", default="true")

  params_dir = os.path.join(package_name, "config")
  # nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
  nav2_params = os.path.join(params_dir, 'nav2_params.yaml')
  configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )
  configured_params = RewrittenYaml(
        source_file=os.path.join(get_package_share_directory('mjf_bunker2_description'), 'config/nav2_params.yaml'), root_key="", param_rewrites="", convert_types=True
    )

  robot_state_publisher_node = launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[{'robot_description': launch_ros.descriptions.ParameterValue( launch.substitutions.Command(['xacro ',default_model_path]), value_type=str)  }]
  )

  joint_state_publisher_node = launch_ros.actions.Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
  )

  rviz_node = launch_ros.actions.Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', LaunchConfiguration('rvizconfig')],
    parameters=[os.path.join(pkg_share, 'config/nav2_params.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
  )

  spawn_entity = launch_ros.actions.Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'mjf_bunker2_description', '-topic', 'robot_description'],
    output='screen'
  )
  robot_localization_node = launch_ros.actions.Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[os.path.join(pkg_share, 'config/ekf_localization.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
  )

  navigation2_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(bringup_dir, "launch", "navigation_launch.py")
      ),
      launch_arguments={
          "use_sim_time": "True",
          "params_file": configured_params,
          "autostart": "True",
      }.items(),
  )

  # New additive Nodes:
    # Add namespace to robot_localization parameter files
  namespaced_ekf_localization_params = ReplaceString(
      source_file=os.path.join(config_file_dir, "ekf_localization.yaml"),
      # source_file=os.path.join(config_file_dir, "gps+imu_ekf_localization.yaml"),
      replacements={"namespace": namespace},
  )

  remapping = [
      ("/tf", "tf"),
      ("/tf_static", "tf_static"),
  ]
  

  slam_toolbox_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('slam_toolbox'), 'launch'),
         '/online_async_launch.py']),
      # launch_arguments={
          # "use_sim_time": "True",
          # "autostart": "True",
          # "odom_frame": "odom",
          # "map_frame": "map",
          # "base_frame": "base_link",
          # "scan_topic": "/scan",
          # "map_update_interval": "5.0",
          # "max_laser_range": "30",
          # "minimum_travel_distance": "0.1",
          # "use_scan_matching": "true",
          # "minimum_travel_heading": "1.57",
          # "do_loop_closing": "true",
          # "slam_methods": "hector",
          # "pub_map_odom_transform": "true", # karto , hector , gmapping
      # }.items(),
  )

  robot_localization_local_node = Node(
      namespace=namespace,
      package="robot_localization",
      executable="ekf_node",
      name="ekf_local_filter_node",
      output="screen",
      parameters=[namespaced_ekf_localization_params, {"use_sim_time": use_sim_time}],
      remappings=remapping
      + [
          ("odom/filtered", "odom/filtered/local"),
          # ("imu", "/wit9073can_imu/data"),
      ],
  )

  robot_localization_global_node = Node(
      namespace=namespace,
      package="robot_localization",
      executable="ekf_node",
      name="ekf_global_filter_node",
      output="screen",
      parameters=[namespaced_ekf_localization_params, {"use_sim_time": use_sim_time}],
      remappings=remapping
      + [
          ("odom/filtered", "odom/filtered/global"),
          # ("imu", "/wit9073can_imu/data"),
      ],
  )

  navsat_transform_node = Node(
      namespace=namespace,
      package="robot_localization",
      executable="navsat_transform_node",
      name="navsat_transform_node",
      output="screen",
      parameters=[namespaced_ekf_localization_params, {"use_sim_time": use_sim_time}],
      remappings=remapping
      + [
          ("imu", "/wit9073can_imu/data"),
          ("gps/fix", "/gps/fix"),
          ("odom/filtered", "odom/filtered/global"),
      ],
  )

  gps_static_transform = Node(
      package = "tf2_ros", 
      executable = "static_transform_publisher",
      arguments = ["0", "0", "0", "0", "0", "0", "Robot_Body", "gps"] # x y z yaw pitch roll frame_id child_frame_id
  )

  imu_static_transform = Node(
      package = "tf2_ros", 
      executable = "static_transform_publisher",
      arguments = ["0", "0", "0", "0", "0", "0", "Robot_Body", "imu"] # x y z yaw pitch roll frame_id child_frame_id
  )


  return launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                        description='Flag to enable joint_state_publisher_gui'),
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_path,
                                        description='Absolute path to rviz config file'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                        description='Flag to enable use_sim_time'),
    # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.dylib', '-s', 'libgazebo_ros_factory.dylib', world_path], output='screen'),
    launch.actions.ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.dylib', '-s', 'libgazebo_ros_factory.dylib', world_path], output='screen'),
    joint_state_publisher_node,
    robot_state_publisher_node,
    spawn_entity,
    robot_localization_node,
    rviz_node,
    navigation2_cmd,
    slam_toolbox_node,
    robot_localization_local_node,
    robot_localization_global_node,
    navsat_transform_node,
    gps_static_transform,
    imu_static_transform
  ])
