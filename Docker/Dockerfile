FROM ros:jazzy

# Set environment variables to non-interactive to prevent prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
# User defined from the existing ros:jazzy image
ENV USER=ubuntu 

# Ensure user and group IDs are defined
ARG UID
ARG GID

# Create a non-root user
# RUN groupadd -g $GID "$USER" && useradd -u $UID -g $GID -m -s /bin/bash "$USER"

# Update apt cache
RUN apt update

# Give sudo privileges to the non-root user
RUN apt install -y sudo \
    && echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER \
    && chmod 0440 /etc/sudoers.d/$USER

# Verify user creation
RUN cat /etc/passwd | grep "$USER"

# Install base necessary packages
RUN apt install --fix-missing -y \
    wget \
    locales \
    lsb-release \
    software-properties-common \
    libglx0 \
    libopengl0 \
    libegl1 \
    libgles2 \
    libglvnd0 \
    mesa-utils-extra   \
    vulkan-tools \
    x11-apps \
    unzip \
    curl \
    terminator \
    tmux \
    locales \
    git \
    nano \
    gnupg2 \
    python3-tk \
    pkg-config \
    libcairo2-dev \
    psmisc \
    gedit \
    xterm \
    gdb \
    && locale-gen en_US.UTF-8

# Set locale
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install Gazebo key for extra packages
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update APT cache to get new GZ packages
RUN apt update

# Install gz-transport library and turtlebot models 
RUN apt install -y \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-plotjuggler-ros \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-bringup \
    ros-jazzy-turtlebot3-teleop \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-gz-sim-vendor \
    ros-jazzy-gz-tools-vendor \
    ros-jazzy-gz-ogre-next-vendor \
    libgz-rendering8-ogre2 

# Install SMC storm
WORKDIR /opt/smc_storm
RUN curl -L https://github.com/convince-project/smc_storm/releases/download/0.1.8/smc_storm_executable.tar.gz \
-o smc_storm_executable.tar.gz && \
tar -xzf smc_storm_executable.tar.gz && \
rm smc_storm_executable.tar.gz
RUN bash install.sh --install-dependencies
RUN ln -s $PWD/bin/smc_storm /usr/local/bin

SHELL ["/bin/bash", "-c"]

# Get ros_gz from source, since APT version has missing features
WORKDIR /opt/ros_gz
RUN git clone https://github.com/gazebosim/ros_gz.git --branch jazzy src
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --packages-up-to ros_gz_bridge --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_cohesion+ && \
    rm -rf src

# Prepare required directories
WORKDIR /home/$USER/
RUN mkdir -p smc_gazebo_ws/src

WORKDIR /home/$USER/smc_gazebo_ws
ADD gazebo_smc_plugins src/gazebo_smc_plugins
ADD gz_sim_handler src/gz_sim_handler
ADD roamer_pkg src/roamer_pkg
ADD ros_smc_plugins src/ros_smc_plugins
ADD sim_handling_interfaces src/sim_handling_interfaces

RUN source /opt/ros/jazzy/setup.bash && \
    source /opt/ros_gz/install/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_cohesion+

# Prepare the bashrc file
RUN echo "alias colcon-clean-ws='rm -rf build install log'" >> /home/$USER/.bashrc
RUN echo "alias colcon-build-ws='colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_cohesion+ --symlink-install'" >> /home/$USER/.bashrc

# Make sure the user owns his home folder (and subfolders)
RUN chown -R $UID:$GID /home/$USER

# Clean APT cache
RUN rm -rf /var/cache/apt/archives /var/lib/apt/lists/*

CMD /bin/bash
COPY Docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
# Default to bash
ENTRYPOINT ["/entrypoint.sh"]
