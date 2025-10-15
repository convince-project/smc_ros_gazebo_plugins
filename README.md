# SMC_plugins_examples

In this repository, we show how it is possible to use Statistical Model Checking (SMC) plugins to verify a robotic system that is partly implemented using ROS 2 and Gazebo.
This repository is connected to the article "Integrating External Components in JANI Models Using SMC Plugins".
Please find the up-to-date content of this repository at https://github.com/convince-project/smc_ros_gazebo_plugins .

## Repository content

The repository consists of the following packages:

* *gz_sim_handler*: containing the JANI models used for verification, as well and the launch files used to start the experiments and the spawn_simulation_ros script, used to handle creation and deletion of the complete simulation environment (based on requests coming from the SMC plugins) and additional elements describing the simulation environment;
* *sim_handling_interfaces*: containing the ROS interfaces used for the communication between the SMC plugins and the spawn_simulation_ros script;
* *gazebo_smc_plugins*: containing the implementation of the SMC plugins used for experiments 1 and 2;
* *ros_smc_plugins*: containing the implementation of the SMC plugins interfacing with the roamer server, used for experiment 3;
* *roamer_pkg*: containing the roamer server, that drives the simulated robot upon request

## Installation

To ensure reproducibility, we recommend using the provided Docker setup, using the Ubuntu 24.04 image and ROS jazzy.

The docker image that we are going to build will contain:
* ROS 2 Jazzy
* ROS Gazebo
* SMC Storm
* Different SMC Plugins, used to communicate with Gazebo and control a wheeled robot

*Important:* the robot simulation relies on the GPU to work, therefore the Docker container needs to use the nvidia-container-toolkit, that can be installed following [this guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

Once the nvidia-container-toolkit is available, the docker image can be built using docker compose:

```bash
docker compose -f docker_compose.yaml build smc_plugins_container
```

## Running the experiments

### Start the docker container

The docker container can be started using docker compose, using the command:

```bash
docker compose -f docker_compose.yaml run --rm smc_plugins_container
```

This will open a new `terminator` terminal, that can be used to execute the commands in the next sections to run the experiments.

### Run the experiments

The subsections below provide with the exact command to run the experiments. Each experiment launches the SMC tool (smc_storm) together with the `spawn_simulation_ros.py` python node, that will create and destroy the robot simulation outside of the SMC tool based on request (each time the SMC tool starts a trace, a new simulation is created, and once it finishes, it is destroyed).
Optionally, the experiment command can start the SMC tool in a separated XTerm session, to better separate the different output from the different tools.

Additional commands for visualization and plotting are presented in the sections after the experiments.

### Experiment 1: Basic controller (obstacle-avoidance)

For the first and the second experiment, we rely on the statistical model checker to control the simulation completely: this means that the simulation is normally paused, and it steps forward by a specific amount of time only after this is requested by the model checker.

This experiments uses the SMC plugins developed in the [gazebo_smc_plugins package](gazebo_smc_plugins) and the [basic_controller_model.jani model](gz_sim_handler/jani/basic_controller_model.jani).

This image gives an overview of the model.

![Obstacle avoidance model](images/sim_step_model.drawio.svg)

The red arrows are related to edges associated to an SMC Plugin, while the other edges are implemented directly in JANI.
The model does the following operation:
* Update the model status, asking the simulator (Gazebo) to advance by one step and reading the updated information
* Check if the robot has an obstacle on its front, front-left or front-right side:
  * If yes, prepare a rotational-only motion (to spin the robot until no obstacle is in front anymore)
  * If not, compute the new velocity to drive the robot towards the goal position
* Finally, send the velocity to the simulator, and go to the first operation again.

To start the first experiment, run the following:

```bash
ros2 launch gz_sim_handler start_ros_and_smc_storm_launch.py config:=$(ros2 pkg prefix --share gz_sim_handler)/config/gazebo_sim_basic_controller.json storm_in_xterm:=true
```

This command will run the experiment on 4 parallel threads, and will take ~ 20 minutes to generate a result based on 400 traces (limited in the configuration for execution time reasons).

In XTerm, SMC Storm will provide information on the current status of the experiment, reporting on the right side how many traces satisfied the property (*S*uccess), how many did not satisfy the property (*F*ailure) and how many failed to terminate correctly (*U*nknown).
The percentage value relates to how the chosen statistical method is converging to the desired result (by default, we are using the Adaptive Sampling Method, with an 95% Confidence Score and an Epsilon of 0.01).

Below, an exemplary progress bar from SMC Storm, after generating 128 traces:

```
[--------------------------------------------------] 0%  (S: 88 F: 40 U: 0)     
```

Once finished, the XTerm terminal running smc_storm should report a result similar to the following one:
```
============= SMC Results =============
        N. of times target reached:     271
        N. of times no termination:     1
        Tot. n. of tries (samples):     400
        Estimated success prob.:        0.679197995
        Confidence score: 0.95
        Estimated Epsilon: 0.0468555 (Corrected Wilson Method)

        Min trace length:       0
        Max trace length:       982
=========================================
Result: 0.679197995
```

At this point, the XTerm terminal can be closed, and the experiment is finished.

### Experiment 2: Controller with refined recovery (obstacle-avoidance)

This model is a small refinement of the one from the 1st experiment, with a more refined handling of the case with obstacles in front of the robot.

It relies on the same plugins used in the previous experiment.
The only difference lies on the handling of the obstacles in front of the robot: the robot starts spinning only in case the front side detects an obstacle (ignoring the front-left and front-right side), and once it starts, it keeps rotating until all three sides are free.

The JANI model containing the refined model can be found in [refined_controller_model.jani](gz_sim_handler/jani/refined_controller_model.jani). As for the previous model, it makes use of the SMC plugins provided in the [gazebo_smc_plugins package](gazebo_smc_plugins).

To start the second experiment, run the following:

```bash
ros2 launch gz_sim_handler start_ros_and_smc_storm_launch.py config:=$(ros2 pkg prefix --share gz_sim_handler)/config/gazebo_sim_refined_controller.json storm_in_xterm:=true
```

This command will run the experiment on 4 parallel threads, and will take ~ 20 minutes to generate a result based on 400 traces (limited in the configuration for execution time reasons).

Once finished, the XTerm terminal running smc_storm should report a result similar to the following one:
```
============= SMC Results =============
        N. of times target reached:     385
        N. of times no termination:     0
        Tot. n. of tries (samples):     400
        Estimated success prob.:        0.9625
        Confidence score: 0.95
        Estimated Epsilon: 0.02028 (Corrected Wilson Method)

        Min trace length:       89
        Max trace length:       926
=========================================
Result: 0.9625
```

At this point, the XTerm terminal can be closed, and the experiment is finished.

### Experiment 3: Random roamer

This model relies on a completely different philosophy, expecting the simulation to be always running, and reading the state from the simulation when the plugin is executed, without the need of pausing/unpausing the simulation each time.

The experiment itself consists of a robot moving randomly in a room until it bumps into an obstacle. Each time it completes a movement, a rotation accompanied by a linear translation, it increases a counter. The movement portion is executed through the assignment of a goal displacement to a ROS action by the plugin. This module returns the conclusion of the movement and whether the robot bumped during it.

The model itself is shown in the following picture.

![Roamer model](images/roamer_diagram.drawio.svg)

It relies on the [roamer_model.jani model](gz_sim_handler/jani/roamer_model.jani), that in turns makes use of the SMC plugins provided in the [ros_smc_plugins package](ros_smc_plugins).

To start the third experiment, run the following:

```bash
ros2 launch gz_sim_handler start_ros_and_smc_storm_launch.py config:=$(ros2 pkg prefix --share gz_sim_handler)/config/gazebo_sim_roamer.json storm_in_xterm:=true
```

This command will run the experiment on 4 parallel threads, and will take ~ 20 minutes to generate a result based on 400 traces (limited in the configuration for execution time reasons).

Once finished, the XTerm terminal running smc_storm should report a result similar to the following one:

```
============= SMC Results =============
        N. of times target reached:     120
        N. of times no termination:     7
        Tot. n. of tries (samples):     400
        Estimated success prob.:        0.3053435115
        Confidence score: 0.95
        Estimated Epsilon: 0.0466115 (Corrected Wilson Method)

        Min trace length:       2
        Max trace length:       20
=========================================
Result: 0.3053435115
```

At this point, the XTerm terminal can be closed, and the experiment is finished.

### Optional: visualize the running experiment

Optionally, it is possible to visualize the running simulation using the Gazebo GUI. This can be done while the experiment is running, executing the commands below in a new terminal within the container (from the terminator instance, you can use `Ctrl + Shift + O` to split the terminal in two):
```bash
# The partition name is thread dependent: robot0, robot1, ..., robotN
export GZ_PARTITION=robot0
gz sim -g
```

### Optional: visualize the exported traces

Each trace that is generated during the experiments is exported as an own CSV file. It can be found in the container's `/tmp` folder, and can be inspected either with a normal text editor, or by plotting tools as `plotjuggler` (recommended).

To open a trace using plotjuggler, you can use the following command (assuming the traces are generated in the `basic_controller_2025-10-01-07-14-35` folder):

```bash
cd /tmp/basic_controller_2025-10-01-07-14-35
ros2 run plotjuggler plotjuggler -n -d trace_47_000005_not_verified.csv
```

The trace name is structured as follows: `trace_<thread-id>_<trace-number>_<evaluation-result>.csv`.

Once plotjuggler opens, select "Use row number as X axis" in the "CSV Loader" window, confirm by pressing OK, and load the desired entries in the plot.

Note that there might be a prompt about a new version of plotjuggler being available, that hides under the "CSV Loader" window and prevents from using it.
In that case, close the New version window first and then continue with the CSV loading configuration.

An exemplary outcome (from the first experiment) is the following:

![PlotJuggler](images/plotjugler-screenshot.png)

## Optional: Running the single components by hand

Using ROS 2 Launch, allows us to start multiple processes at once with a single command.

In case it is desired to start the single processes composing the experiments by hand, we provide instructions on how to run them in the [Manual-instructions.md file](Manual-instructions.md).

## License

ros_gazebo_smc_plugins comes under the Apache-2.0 license, see [LICENSE](LICENSE).
