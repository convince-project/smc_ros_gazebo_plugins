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

from copy import deepcopy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import json

from launch.actions import GroupAction, SetEnvironmentVariable, RegisterEventHandler
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PythonExpression
from launch.actions import TimerAction, LogInfo

from launch.event_handlers import OnExecutionComplete

name_map = {
    "-entity": "entity_name",   # or "entity" if that's what the include uses
    "-name": "name",
    "-allow_renaming": "allow_renaming",
    "-x": "x_pose",
    "-y": "y_pose",
    "-z": "z_pose",
}

def launch_gazebo(parameters, env):
    world = os.path.join(
        get_package_share_directory(parameters["world"]["repository"]),
        parameters["world"]["path"],
        parameters["world"]["file"],
    )
    base_cmd = ['gz', 'sim', world]
    
    if "--headless-rendering" in parameters["parameters"]:
        # start from caller-provided env (or empty), then force headless EGL
        env = dict(env or {})
        # scrub anything that would push GLX/X or software paths
        

        env.update({
            "EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
            "QT_QPA_PLATFORM": "offscreen",
            "EGL_PLATFORM": "surfaceless",
            "DISPLAY": "",
        })

        base_cmd = ["gz", "sim", world, "--render-engine", "ogre2", "-s", "--headless-rendering"]
    
    base_cmd.extend(parameters["parameters"])  # should include: ['-s', '--headless-rendering']

    return [ExecuteProcess(
        cmd=base_cmd,
        output='screen',
        additional_env=env
    )]


def launch_world(parameters):
    world = os.path.join(get_package_share_directory(parameters["world"]["repository"]), parameters["world"]["path"], parameters["world"]["file"])
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    return [gzserver_cmd,gzclient_cmd]

def launch_bridge(env, namespace):
    bridges=[]
    bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="world_control_bridge",
            output='log',
            arguments=[
                "world/default/control"
                "@ros_gz_interfaces/srv/ControlWorld"
                "@gz.msgs.WorldControl"
                "@gz.msgs.Boolean"
            ],
            namespace=namespace,
            additional_env=env
        )
    bridges.append(bridge)
    return bridges




def launch_robot(parameters, env, namespace):
    robot_path = os.path.join(get_package_share_directory(parameters["file"]["repository"]), parameters["file"]["path"], parameters["file"]["file"])
    repetitions = parameters.get("repetitions")
    robot_node_list=[]
    env_actions = [SetEnvironmentVariable(name=k, value=v) for k, v in env.items()]
    for i in range(repetitions):
        fix_parameters = deepcopy(parameters["fixed_parameters"])
        fix_parameters.append('-file')
        fix_parameters.append(robot_path)
        for j in range(len(fix_parameters)):
            term = fix_parameters[j]
            if term == '-name' or term == '-entity':
                if term=='-name':
                    name=fix_parameters[j+1]
                    r_type=fix_parameters[j+1]
                if repetitions > 1:
                    fix_parameters[j+1] += str(i)
                name=fix_parameters[j+1]
                
        child_env = {
            **env,  # your base env: GZ_SIM_RESOURCE_PATH, GZ_PARTITION, etc.
            "TURTLEBOT3_MODEL": r_type,                    # <-- crucial: set here
        }
        env_actions = [SetEnvironmentVariable(name=k, value=v) for k, v in child_env.items()]


        start_gazebo_ros_spawner_cmd = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=fix_parameters,
            output='screen',
            additional_env=child_env
        )
        
        bridge_params = os.path.join(
            get_package_share_directory(parameters["bridge"]["repository"]),
            parameters["bridge"]["configuration_file"]
        )


        start_gazebo_ros_bridge_cmd = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}',
            ],
            output='screen',
            additional_env=child_env
        )

        start_odom_publisher=Node(
            package='roamer_pkg',                 # <-- change to your package name
            executable='pose_publisher.py',          # <-- matches your entry point/executable
            name='pose_to_odom',
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
        
        start_action_node=Node(
            package='roamer_pkg',                 # <-- change to your package name
            executable='roamer_server_node',          # <-- matches your entry point/executable)
            name='roamer_action_node',
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        start_gazebo_ros_image_bridge_cmd = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['camera/image_raw'],
            output='screen',
            additional_env=child_env
        )

        robot_action=GroupAction([
            PushRosNamespace(f"{namespace}/{name}"),
            *env_actions,
            start_gazebo_ros_spawner_cmd,
            start_action_node,
            start_gazebo_ros_bridge_cmd,
            start_odom_publisher,
            start_gazebo_ros_image_bridge_cmd,
        ])
        robot_node_list.append(robot_action)

    return [*env_actions,*robot_node_list]

def _launch(context):
    config_path = LaunchConfiguration('config').perform(context)
    cfg = json.loads(config_path)

    gazebo_parameters=cfg.get("gazebo")
    bridge_parameters=cfg.get("bridge_configuration")
    robots_parameters=cfg.get("robots")
    simulation_partition=cfg.get("partition")
    
    robots_parameters[0]["bridge"]=bridge_parameters

    world_models = os.path.join(get_package_share_directory(gazebo_parameters["world"]["repository"]), gazebo_parameters["world"]["path"])
    ros_models_paths = f"{world_models}"
    for robot in robots_parameters:
        ros_models_paths += ":"+os.path.join(get_package_share_directory(robot["file"]["repository"]), robot["file"]["path"])

    namespace = simulation_partition
    gazebo_environment = {
        'TURTLEBOT3_MODEL': "waffle",
        'GZ_SIM_RESOURCE_PATH': ros_models_paths,
        'GZ_PARTITION': simulation_partition,
    }
    env_actions = [SetEnvironmentVariable(name=k, value=v) for k, v in gazebo_environment.items()]

    
    
    processes = []
    # 4) start gz (your function should return a LIST of actions)
    processes.extend(launch_gazebo(gazebo_parameters, gazebo_environment))


    # processes.extend(launch_world(gazebo_parameters))

    # launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    # # 5) TB3 robot_state_publisher (TB3 launch reads env at import, but we setdefault() already)
    # processes.append(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
    #     ),
    #     launch_arguments={'use_sim_time': 'true'}.items()   # values as strings
    # ))

    use_sim_time = True
    urdf_file_name = 'turtlebot3_' + 'waffle' + '.urdf'
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()


    robot_state_pub=Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
            }],
        )
    processes.append(robot_state_pub)


    # 6) spawn all robots (your helper should return a LIST; use extend)
    for robot in robots_parameters:
        processes.extend(launch_robot(robot, gazebo_environment, namespace))

    processes.extend(launch_bridge(gazebo_environment, namespace))

    group_action = GroupAction([*env_actions, *processes])

    # completion_handler = RegisterEventHandler(
    #     OnExecutionComplete(
    #         target_action=group_action,
    #         on_completion=LogInfo(msg='All actions in the group have been registered.')
    #     )
    # )

    # 8) group so env applies to everything above
    return [group_action]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='',
            description='Path to JSON config file (overrides defaults).'
        ),
        OpaqueFunction(function=_launch),
    ])