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

from typing import Union, List
import os
import json

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def _get_storm_exe(conf: dict, open_xterm: bool) -> ExecuteProcess:
    # Model info
    model_conf = conf["computed"]["model"]
    model_path = os.path.join(get_package_share_directory(model_conf["package_name"]), model_conf["path"])
    # Plugins info
    plugins_conf: Union[dict, List[dict]] = conf["computed"]["plugin-paths"]
    plugin_paths_list: List[dict] = []
    if isinstance(plugins_conf, dict):
        plugin_paths_list.append(plugins_conf)
    else:
        assert isinstance(plugins_conf, list), "Unexpected data in storm config: 'plugin-paths' should be a list or a dict."
        plugin_paths_list = plugins_conf
    plugins_path = ""
    for single_plugins_path in plugin_paths_list:
        full_path = os.path.join(get_package_prefix(single_plugins_path["package_name"]), single_plugins_path["path"])
        plugins_path += full_path + ","
    assert len(plugins_path) > 0, "No plugin path provided."
    plugins_path = plugins_path.removesuffix(",")
    # Make the executable
    smc_params = ["--model", model_path, "--plugin-paths", f"{plugins_path}"]
    for k, v in conf["fixed"].items():
        if isinstance(v, bool):
            if v is True:
                smc_params.append(f"--{k}")
        elif isinstance(v, str):
            smc_params.extend([f"--{k}", v])
        else:
            raise RuntimeError("Storm params can be either bool os str, no other type supported")
    storm_prefix = 'xterm -hold -e' if open_xterm else None
    return ExecuteProcess(
        cmd=["smc_storm"] + smc_params,
        output='screen',
        prefix=storm_prefix,
        additional_env={"RUST_LOG": "zenoh=off"}
    )


def _get_gazebo_handler(json_path: str) -> Node:
    return Node(
        package="gz_sim_handler",
        executable="spawn_simulation_ros.py",
        output={'both': 'log'},
        arguments=[json_path],
        additional_env={"RUST_LOG": "zenoh=off"}
    )


def _launch(context: LaunchContext):
    conf_json_path = context.launch_configurations.get('config')
    open_xterm_str = context.launch_configurations.get('storm_in_xterm')
    open_xterm: bool = (open_xterm_str == "true")
    with open(conf_json_path) as f:
        cfg = json.load(f)
    storm_exe = _get_storm_exe(cfg['storm_parameters'], open_xterm)
    spawner_node = _get_gazebo_handler(conf_json_path)
    zenoh_node = Node(
        package = "rmw_zenoh_cpp",
        executable = "rmw_zenohd",
        output = {'both': 'log'},
        additional_env={"RUST_LOG": "zenoh=off"}
    )

    # Shutdown when smc_storm exits
    stop_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=storm_exe,
            on_exit=[EmitEvent(event=Shutdown(reason="smc_storm finished!"))],
        )
    )

    return [zenoh_node, spawner_node, storm_exe, stop_when_done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            description='Path to JSON config file (overrides defaults).',
        ),
        DeclareLaunchArgument(
            'storm_in_xterm',
            description='Path to JSON config file (overrides defaults).',
            default_value="false",
            choices=["true", "false"]
        ),
        OpaqueFunction(function=_launch),
    ])