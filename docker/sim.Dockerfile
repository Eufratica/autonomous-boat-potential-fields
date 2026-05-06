FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
ENV CATKIN_WS=/ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    rsync \
    curl \
    ca-certificates \
    python3-pip \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-rosdep \
    python3-vcstool \
    protobuf-compiler \
    libprotobuf-dev \
    libeigen3-dev \
    libboost-all-dev \
    libignition-math4-dev \
    geographiclib-tools \
    mesa-utils \
    x11-apps \
    ros-noetic-hector-gazebo-plugins \
    ros-noetic-robot-localization \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins \
    ros-noetic-realsense2-description \
    ros-noetic-xacro \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir bluerobotics-ping pymavlink

RUN rosdep init 2>/dev/null || true && rosdep update || true

# Mirror the native install_deps.sh external repositories at image build time.
RUN mkdir -p ${CATKIN_WS}/src && cd ${CATKIN_WS}/src && \
    git clone https://github.com/Livox-SDK/livox_laser_simulation && \
    sed -i 's/-std=c++11/-std=c++17/gi' livox_laser_simulation/CMakeLists.txt && \
    git clone --recurse-submodules https://github.com/ttrindader/ping360_gazebo.git && \
    chmod +x ping360_gazebo/ping360_gazebo_plugin/scripts/numpy_pc2.py \
             ping360_gazebo/ping360_gazebo_plugin/scripts/pcl_gen.py || true

RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh || true

COPY docker/entrypoints/sim_entrypoint.sh /usr/local/bin/sim_entrypoint.sh
RUN chmod 755 /usr/local/bin/sim_entrypoint.sh

WORKDIR ${CATKIN_WS}
ENTRYPOINT ["/usr/local/bin/sim_entrypoint.sh"]
CMD ["bash"]
