# Copyright (c) 2020 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# /* Author: Doron Hirshberg */
import os
import xacro
import tempfile
import launch
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))


def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path


def generate_launch_description():
    SIMULATE_WORLD = 'empty_world'
    SIMULATE_MODEL = 'turtlebot3_burger.urdf.xacro'
    
    rviz_config_dir = os.path.join(get_package_share_directory('simulate_realsense_d435i'), 'rviz', 'urdf.rviz')
    xacro_path = os.path.join(get_package_share_directory('simulate_realsense_d435i'), 'urdf', SIMULATE_MODEL)
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics' : 'true', 'add_plug' : 'true', 'use_sim_time': use_sim_time})
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(get_package_share_directory('simulate_realsense_d435i'), 'worlds', SIMULATE_WORLD + '.model')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
        )
    model_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments = [urdf]
        )
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
        launch_arguments={'world': world}.items(),
        )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        )
    gazebo_sim = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
        output='screen'
        )
    return launch.LaunchDescription([
        # rviz_node,
        model_node,
        gazebo_server,
        gazebo_client,
        gazebo_sim,
        ])
