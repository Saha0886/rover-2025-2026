FROM ros:humble-ros-core

WORKDIR /ros_ws

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    gcc \
    g++ \
    neovim \
    libasio-dev \
    ros-humble-ament-cmake \
    git \
    python3-rosdep \
    python3-pip\
    python3-serial \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY . /ros_ws/src/ranger_peripherals

RUN rosdep init && \
    rosdep update

RUN /usr/bin/python3 -m pip install pyserial

# Install dependencies using rosdep
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# Source the setup file when starting a shell
RUN echo "source /ros_ws/install/setup.bash" >> /root/.bashrc

# Command to run when the container starts
CMD ["bash"]