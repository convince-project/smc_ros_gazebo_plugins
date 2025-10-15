# Copyright (c) 2025 - for information on the respective copyright owner
# see the NOTICE file

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List, Optional, Union

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
import os
import json


creations_finished = 0


def generate_gz_launch(gz_conf: dict, gz_partition: str) -> ExecuteProcess:
    """Procedure to start the Gazebo simulator."""
    world = os.path.join(
        get_package_share_directory(gz_conf["world"]["package_name"]),
        gz_conf["world"]["path"],
        gz_conf["world"]["file"],
    )
    base_cmd = ['gz', 'sim', world]
    
    if "--headless-rendering" in gz_conf["parameters"]:
        # start from caller-provided env (or empty), then force headless EGL
        additional_env = {
            # "EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
            # "EGL_PLATFORM": "surfaceless",
            "QT_QPA_PLATFORM": "offscreen",
            "DISPLAY": "",
            "GZ_PARTITION": gz_partition,
            'GZ_SIM_RESOURCE_PATH': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
        }

        base_cmd = ["gz", "sim", world] + gz_conf["parameters"]

    return ExecuteProcess(
        cmd=base_cmd,
        output='screen',
        additional_env=additional_env
    )


def generate_gz_bridge(bridge_conf: dict, gz_partition: str, n_robots: int) -> List[Node]:
    """Procedure to start the GZ Bridge node."""
    global creations_finished
    # This function is called each time a robot is spawned, but we want to start the bridgge only after all robots are spawned.
    creations_finished += 1
    if creations_finished < n_robots:
        return []
    bridge_yaml = os.path.join(
        get_package_share_directory(bridge_conf["package_name"]),
        bridge_conf["file"],
    )
    additional_env = {
        "GZ_PARTITION": gz_partition
    }
    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=gz_partition,
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_yaml}',
        ],
        output='screen',
        additional_env=additional_env
    )]


def generate_robot_spawn(spawn_conf: dict, gz_partition: str) -> Node:

    """Procedure to spawn the robot in the simulation environment."""
    robot_model = os.path.join(
        get_package_share_directory(spawn_conf["file"]["package_name"]),
        spawn_conf["file"]["path"],
        spawn_conf["file"]["file"],
    )
    additional_env = {
        "GZ_PARTITION": gz_partition,
        'GZ_SIM_RESOURCE_PATH': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    }
    return Node(
        package='ros_gz_sim',
        executable='create',
        namespace=gz_partition,
        arguments=['-file', robot_model] + spawn_conf['fixed_parameters'],
        output='screen',
        additional_env=additional_env
    )

def provide_robot_description(spawn_conf: dict, gz_partition: str) -> Node:
    """Procedure to start the robot_description node."""
    description_field: dict = spawn_conf.get('model')

    robot_model_filename = description_field['file']
    _, robot_model_extension = os.path.splitext(robot_model_filename)
    robot_model_abs_path = os.path.join(
        get_package_share_directory(description_field["package_name"]),
        description_field['path'],
        description_field['file']
    )

    if robot_model_extension == ".urdf":
        with open(robot_model_abs_path, 'r') as infp:
            robot_description: Union[str, ParameterValue] = infp.read()
    elif robot_model_extension == ".xacro":
        robot_description = ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', robot_model_abs_path]),
            value_type=str
        )
    else:
        raise RuntimeError(f"Unknown robot model extension of {robot_model_filename}.")

    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=gz_partition,
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

def generate_additional_nodes(additional_nodes_conf: Optional[list], gz_partition: str) -> List[Node]:
    """Procedure to start the experiment's additional nodes."""
    generated_nodes = []
    if additional_nodes_conf is not None:
        assert isinstance(additional_nodes_conf, list), f"Additional nodes should be provided as a list, found a {type(additional_nodes_conf)}."
        for nc in additional_nodes_conf:
            # Allow both "node_name" (your current JSON) or "executable"
            executable = nc.get('executable', nc.get('node_name'))
            if not executable:
                raise ValueError(f"Missing 'executable'/'node_name' in {nc}")

            # Build parameters list (dicts and/or YAML file paths)
            param_entries = []

            # 1) Inline params from JSON (must be native JSON types: bool/int/float/str/list)
            params = nc.get('parameters')
            if isinstance(params, dict):
                param_entries.append(params)
            elif isinstance(params, list):
                # Support a list of dicts if you want to layer multiple blocks
                param_entries.extend(p for p in params if isinstance(p, dict))

            # 2) YAML files (optional)
            for p in nc.get('parameter_files', []):
                # Expand ~ and $VARS if you like
                param_entries.append(os.path.expandvars(os.path.expanduser(p)))

            # Remap handling: accept dict {"from":"to"...} or list of pairs
            remaps = []
            r = nc.get('remappings', [])
            if isinstance(r, dict):
                remaps = list(r.items())
            elif isinstance(r, list):
                for item in r:
                    if isinstance(item, (list, tuple)) and len(item) == 2:
                        remaps.append((item[0], item[1]))
                    elif isinstance(item, dict) and 'from' in item and 'to' in item:
                        remaps.append((item['from'], item['to']))

            remaps.append(('/clock', 'clock'))

            generated_nodes.append(Node(
                package=nc['package_name'],
                executable=executable,
                name=nc.get('name'),
                namespace=gz_partition,
                parameters=param_entries or None,
                remappings=remaps or None,
                arguments=nc.get('arguments', []),
                output=nc.get('output', 'screen'),
                emulate_tty=nc.get('emulate_tty', True),
                respawn=nc.get('respawn', False),
                respawn_delay=nc.get('respawn_delay', 2.0),
                additional_env=nc.get('env')  # optional dict of env vars
            ))
    return generated_nodes

def start_system(context: LaunchContext):
    """Procedure to start the complete simulation, including robot description, gz_bridge and additional nodes."""
    conf_json_file = context.launch_configurations.get('config')
    with open(conf_json_file) as f:
        config_dict: dict = json.load(f)
    launch_returns = []
    launch_returns.append(generate_gz_launch(config_dict['gazebo'], config_dict['partition']))
    launch_returns.extend(generate_additional_nodes(config_dict.get('additional_nodes'), config_dict['partition']))
    # Make sure all robots are spawned before starting the bridge
    n_robots = len(config_dict['robots'])

    for robot in config_dict['robots']:
        single_spawn = generate_robot_spawn(robot, config_dict['partition'])
        launch_returns.append(single_spawn)

        if 'model' in robot:
            launch_returns.append(provide_robot_description(robot, config_dict['partition']))
        
        launch_returns.append(RegisterEventHandler(
            OnProcessExit(
                target_action=single_spawn,
                on_exit=generate_gz_bridge(config_dict['bridge_configuration'], config_dict['partition'], n_robots),
            )
        ))
    return launch_returns

def generate_launch_description():
    # Set this to True for running STORM in GDB
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='',
            description='Path to JSON config file (overrides defaults).'
        ),
        OpaqueFunction(function=start_system)
    ])
