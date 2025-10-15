# Start the single programs by hand

To start the single programs composing the experiments, you need to open three terminal sessions in the running container, one for the ROS 2 communication handler (zenoh), one for the simulation handler and one for smc_storm.

To do so you can, after terminator is open, split it into sub-terminals using `Ctrl + Shift + O`.

The next subsections list the commands to run in the three terminals for each experiment.

## Experiment 1

Terminal 1: start Zenoh (for communication over ROS 2)
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Terminal 2: start the spawn_simulation_ros.py, used for creating and deleting simulation instances on request:
```bash
ros2 run gz_sim_handler spawn_simulation_ros.py $(ros2 pkg prefix --share gz_sim_handler)/config/gazebo_sim_basic_controller.json
```

Terminal 3: start smc_storm, providing the model to verify and the paths where to find the SMC plugins' libraries:
```bash
smc_storm --model $(ros2 pkg prefix --share gz_sim_handler)/jani/basic_controller_model.jani --properties-names driving --plugin-paths $(ros2 pkg prefix gazebo_smc_plugins)/lib/gazebo_smc_plugins --batch-size 1 --max-n-traces 400 --n-threads 4 --disable-explored-states-caching --show-statistics
```

## Experiment 2

Terminal 1: start Zenoh (for communication over ROS 2)
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Terminal 2: start the spawn_simulation_ros.py, used for creating and deleting simulation instances on request:
```bash
ros2 run gz_sim_handler spawn_simulation_ros.py $(ros2 pkg prefix --share gz_sim_handler)/config/gazebo_sim_refined_controller.json
```

Terminal 3: start smc_storm, providing the model to verify and the paths where to find the SMC plugins' libraries:
```bash
smc_storm --model $(ros2 pkg prefix --share gz_sim_handler)/jani/refined_controller_model.jani --properties-names driving --plugin-paths $(ros2 pkg prefix gazebo_smc_plugins)/lib/gazebo_smc_plugins --batch-size 1 --max-n-traces 400 --n-threads 4 --disable-explored-states-caching --show-statistics
```

## Experiment 3

Terminal 1: start Zenoh (for communication over ROS 2)
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Terminal 2: start the spawn_simulation_ros.py, used for creating and deleting simulation instances on request:
```bash
ros2 run gz_sim_handler spawn_simulation_ros.py $(ros2 pkg prefix --share gz_sim_handler)/config/gazebo_sim_roamer.json
```

Terminal 3: start smc_storm, providing the model to verify and the paths where to find the SMC plugins' libraries:
```bash
smc_storm --model $(ros2 pkg prefix --share gz_sim_handler)/jani/roamer_model.jani --properties-names two_to_nine_steps --plugin-paths $(ros2 pkg prefix ros_smc_plugins)/lib/ros_smc_plugins --batch-size 1 --max-n-traces 400 --n-threads 4 --disable-explored-states-caching --show-statistics
```
