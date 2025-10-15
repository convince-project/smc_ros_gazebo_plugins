#!/usr/bin/env python3

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
from random import choice, random
from math import pi
from typing import Dict
import os
from datetime import datetime
import rclpy
from rclpy.node import Node, Client
from sim_handling_interfaces.srv import StartGazebo, StopGazebo
import signal
import psutil
import argparse
import json
from ros_gz_interfaces.srv import ControlWorld

AVAILABLE_COORDINATES = [-1.5, -0.5, 0.5, 1.5]


## CLASS DEFINITION

class SimulationManager(Node):
    def __init__(self, arguments):
        super().__init__('simulation_manager')
        self.max_simulations = arguments.get("max_threads")
        self.command= arguments.get("command", None)
        self.simulation_parameters=arguments.get("simulation_parameters", {})
        curr_datetime = datetime.now()
        configs_folder = f"gz_spawn_node_{curr_datetime.date().isoformat()}_{curr_datetime.time().isoformat()}"
        self._configs_folder_abs = os.path.join("/tmp", configs_folder)
        os.makedirs(self._configs_folder_abs)
        self.get_logger().info(f"Store configs in {self._configs_folder_abs}")
        self._sim_count = 0

        self.__thread_handler: Dict[str, psutil.Popen] = {}
        self.__part_client: Dict[str, Client] = {}

        # Start the services to start and stop simulations
        self.start_srv = self.create_service(StartGazebo, 'start_simulation', self.handle_start)
        self.stop_srv = self.create_service(StopGazebo, 'stop_simulation', self.handle_stop)

    def handle_start(self, request: StartGazebo.Request, response: StartGazebo.Response) -> StartGazebo.Response:
        assert len(self.__thread_handler) < self.max_simulations, f"Spawning too many simulations."
        if request.gz_partition not in self.__thread_handler:
            success = self.__start_simulation(request.gz_partition, request.parameters)
            # Fill response
            response.success = success
        else:
            self.get_logger().error(f"Trying to spawn a simulation on an already used partition {request.gz_partition}.")
            response.success = False
        # Return
        return response

    def handle_stop(self, request: StopGazebo.Request, response: StopGazebo.Response) -> StopGazebo.Response:
        return_value=self.__stop_simulation(request.gz_partition)
        response.success = return_value
        return response

    def __stop_simulation(self, partition):
        """Stops a running gazebo process, together with its children."""
        self.get_logger().info(f"Stopping gz instance at partition {partition}")
        thread_handle: psutil.Popen = self.__thread_handler[partition]
        if thread_handle.poll() is not None:
            self.get_logger().warn(f"The process at partition {partition} already stopped")
        else:
            # Maybe not required, but in case stopping launch processes does not work, kill sub-processes
            try:
                children = thread_handle.children(recursive=True)
            except psutil.Error:
                children = []
            children.append(thread_handle)
            # Send sigkill to all processes and then ensure all got shutdown (faster)
            for child in children:
                child.send_signal(signal.SIGKILL)
            for child in children:
                child.wait(3)    
        self.get_logger().info(f"Process on partition {partition} stopped.")
        _ = self.__thread_handler.pop(partition)
        return True

    def destroy_node(self):
        # Use prints here, since the ROS context should be already gone
        print("Destroying node...")
        while len(self.__thread_handler) > 0:
            gz_part = next(iter(self.__thread_handler.keys()))
            print(f"Kill gazebo process on partition {gz_part}")
            self.__stop_simulation(gz_part)
        return super().destroy_node()

    def __start_simulation(self, partition: str, parameters: str):
        """Start the ignition process."""

        self._sim_count += 1
        # Compute random start pose
        robot_start_x = choice(AVAILABLE_COORDINATES)
        robot_start_y = choice(AVAILABLE_COORDINATES)
        robot_start_theta = 2 * pi * random()
        # Generate and launch simulation
        current_launch_config = deepcopy(self.simulation_parameters)
        current_launch_config["partition"]=partition
        current_launch_config["gazebo"]["parameters"].extend(["--seed", f"{self._sim_count}"])
        spawn_params = current_launch_config["robots"][0]["fixed_parameters"]
        spawn_params['-x'] = str(robot_start_x)
        spawn_params['-y'] = str(robot_start_y)
        spawn_params['-Y'] = str(robot_start_theta)
        gz_spawn_p_list = []
        for k, v in spawn_params.items():
            gz_spawn_p_list.extend([k, v])
        current_launch_config["robots"][0]["fixed_parameters"] = gz_spawn_p_list

        ## TODO: use raw to correct and supplement self.simulation parameters at runtime
        # raw = json.loads(parameters.strip())

        config_file = f"gz_sim_{self._sim_count:06}_{partition}.json"
        json_file_path = os.path.join(self._configs_folder_abs, config_file)
        with open(json_file_path, "w") as json_file:
            json.dump(current_launch_config, json_file)

        # Command to launch the ROS environment and connected topics (the partition becomes the high level namespace)

        ros_start_cmd = f"{self.command} config:={json_file_path} --noninteractive".split()
        self.__thread_handler[partition] = psutil.Popen(ros_start_cmd, shell=False)
        self.get_logger().info(self.__thread_handler[partition].poll())
        ## Use wait for active service instead of sleep
        if partition not in self.__part_client:
            self.__part_client[partition] = self.create_client(ControlWorld, f"/{partition}/world/default/control")
        start_ok = False
        for _ in range(3):
            if self.__part_client[partition].wait_for_service(10.0):
                start_ok = True
                break
        if not start_ok:
            self.get_logger().error(f"Process on partition {partition} failed to start.")
        self.get_logger().info(f"Process on partition {partition} started")
        self.get_logger().info("Simulation started")
        return start_ok

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=str) # the JSON string after it
    cli_args, ros_args = parser.parse_known_args()
    with open(cli_args.config) as f:
        cfg = json.load(f)
    rclpy.init(args=ros_args)  # pass the leftovers to ROS
    node = SimulationManager(cfg['spawner_parameters'])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
