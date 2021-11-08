import os
import xacro
import tempfile
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import sys


sys.path.append(os.path.dirname(os.path.realpath(__file__)))


PKG_THIS = get_package_share_directory('simulate_realsense_d435i')
PKG_GAZEBO_ROS = get_package_share_directory('gazebo_ros')
SIMULATE_MODEL = 'turtlebot3_burger.urdf.xacro'


def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))
    doc = xacro.process_file(xacro_path, mappings=parameters)
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))
    return urdf_path


def generate_launch_description():
    world_launch_arg = DeclareLaunchArgument("world", default_value='env3')
    
    rviz_config_dir = os.path.join(PKG_THIS, 'rviz', 'urdf.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
    )
        
    xacro_path = os.path.join(PKG_THIS, 'urdf', SIMULATE_MODEL)
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics' : 'true', 'add_plug' : 'true', 'use_sim_time': use_sim_time})    
    model_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments = [urdf]
    )
    
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(PKG_GAZEBO_ROS, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': [PKG_THIS, '/worlds/', LaunchConfiguration('world'), '.world']}.items(),
    )
    
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(PKG_GAZEBO_ROS, 'launch', 'gzclient.launch.py')),
    )
    
    gazebo_sim = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
        output='screen'
    )
    
    return launch.LaunchDescription([
        world_launch_arg,
        rviz_node,
        model_node,
        gazebo_server,
        # gazebo_client,
        gazebo_sim,
    ])
