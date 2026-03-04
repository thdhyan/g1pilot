FROM registry.gitlab.com/bleurobotics/code/bleu_integration/bleu-base:jazzy-cuda-26-02

SHELL ["/bin/bash", "-c"]

ENV PYTHONUNBUFFERED=1

WORKDIR /home
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# create forest ws and use it to clone and install CONCERT's simulation package
WORKDIR /home/forest_ws
ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https

# create python virtual env
RUN apt-get update && apt-get install -y python3-venv && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv /home/.base
RUN echo "source /home/.base/bin/activate" >> ~/.bashrc
SHELL ["bash", "-ic"]

RUN pip install --upgrade ttictoc "setuptools<81" hhcm-forest && forest init
RUN echo "source $PWD/setup.bash" >> ~/.bashrc
SHELL ["bash", "-ic"]

RUN forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git --tag ros2

# pre-install pybind11 and custom matlogger2
#RUN forest grow pybind11 --verbose --jobs 4 --pwd user && \
RUN forest grow matlogger2 --verbose --jobs 4 --pwd user

# HPP-FCL
WORKDIR /home/forest_ws/src
RUN git clone https://github.com/humanoid-path-planner/hpp-fcl.git && \
    ls -a && \
    cd /home/forest_ws/src/hpp-fcl && \
    git checkout 45e60ca7ba81e5394605f8c1097c016245d221c2 && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/hpp-fcl && \
    cd /home/forest_ws/build/hpp-fcl && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_PYTHON_INTERFACE=OFF ../../src/hpp-fcl && \
    make -j8 && \
    make install

## PINOCCHIO
WORKDIR /home/forest_ws/src
RUN git clone -b devel https://github.com/stack-of-tasks/pinocchio.git && \
    cd /home/forest_ws/src/pinocchio && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/pinocchio && \
    cd /home/forest_ws/build/pinocchio && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=ON -DBUILD_TESTING=FALSE -DBUILD_PYTHON_INTERFACE=OFF ../../src/pinocchio && \
    make -j4 && \
    make install


# xbot2_interface
WORKDIR /home/forest_ws/src
RUN apt-get update && apt-get install -y ros-jazzy-srdfdom ros-jazzy-geometric-shapes && rm -rf /var/lib/apt/lists/*
RUN git clone -b devel https://github.com/ADVRHumanoids/xbot2_interface.git && \
    mkdir -p /home/forest_ws/build/xbot2_interface && \
    cd /home/forest_ws/build/xbot2_interface && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DXBOT2_IFC_BUILD_TESTS=ON -DXBOT2_IFC_BUILD_ROS=OFF -DXBOT2_IFC_BUILD_ROS2=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/xbot2_interface && \
    make -j8 && \
    make install

# osqp
RUN git clone https://github.com/oxfordcontrol/osqp.git && \
    cd /home/forest_ws/src/osqp && \
    git checkout 0b34f2ef5c5eec314e7945762e1c8167e937afbd && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/osqp && \
    cd /home/forest_ws/build/osqp && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DDLONG=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/osqp && \
    make -j8 && \
    make install

# proxQP
RUN git clone https://github.com/Simple-Robotics/proxsuite.git && \
    cd /home/forest_ws/src/proxsuite && \
    git checkout f19f07b51f66268db1f16cbeb538e891bb6d4e21 && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/proxsuite && \
    cd /home/forest_ws/build/proxsuite && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/proxsuite && \
    make -j8 && \
    make install

RUN apt-get update && apt-get upgrade -y && apt-get clean && apt-get install -y libxcb-cursor0 gdb ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui && rm -rf /var/lib/apt/lists/*

# FCL v0.6.0 THIS IS REQUIRED TO RUN TESTS IN OPENSOT! THIS MAY CONFLICT WITH fcl v0.7 installed by default!
WORKDIR /home/forest_ws/src
RUN git clone -b v0.6.0 https://github.com/flexible-collision-library/fcl.git && \
    mkdir -p /home/forest_ws/build/fcl && \
    cd /home/forest_ws/build/fcl && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/fcl  && \
    make -j8 && \
    make install
#MANUALLY ADDING missing fclConfigVersion.cmake to retieve version informations of FCL
RUN FCL_VERSION=0.6.1 && \
    CONFIG_DIR=/home/forest_ws/install/lib/cmake/fcl && \
    mkdir -p "$CONFIG_DIR" && \
    echo "set(PACKAGE_VERSION \"$FCL_VERSION\")" > "$CONFIG_DIR/fclConfigVersion.cmake" && \
    echo "" >> "$CONFIG_DIR/fclConfigVersion.cmake" && \
    echo "if(PACKAGE_FIND_VERSION)" >> "$CONFIG_DIR/fclConfigVersion.cmake" && \
    echo "  if(PACKAGE_VERSION VERSION_LESS PACKAGE_FIND_VERSION)" >> "$CONFIG_DIR/fclConfigVersion.cmake" && \
    echo "    set(PACKAGE_VERSION_COMPATIBLE FALSE)" >> "$CONFIG_DIR/fclConfigVersion.cmake" && \
    echo "  else()" >> "$CONFIG_DIR/fclConfigVersion.cmake" && \
    echo "    set(PACKAGE_VERSION_COMPATIBLE TRUE)" >> "$CONFIG_DIR/fclConfigVersion.cmake" && \
    echo "    if(PACKAGE_VERSION VERSION_EQUAL PACKAGE_FIND_VERSION)" >> "$CONFIG_DIR/fclConfigVersion.cmake" && \
    echo "      set(PACKAGE_VERSION_EXACT TRUE)" >> "$CONFIG_DIR/fclConfigVersion.cmake" && \
    echo "    endif()" >> "$CONFIG_DIR/fclConfigVersion.cmake" && \
    echo "  endif()" >> "$CONFIG_DIR/fclConfigVersion.cmake" && \
    echo "endif()" >> "$CONFIG_DIR/fclConfigVersion.cmake"


# qpSWIFT
RUN git clone https://github.com/qpSWIFT/qpSWIFT.git && \
    mkdir -p /home/forest_ws/build/qpSWIFT && \
    cd /home/forest_ws/build/qpSWIFT && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/qpSWIFT && \
    make -j8 && \
    make install

WORKDIR /home
RUN echo "export PYTHONPATH=${PYTHONPATH}:/root/.local/lib/python3.12/site-packages:/usr/lib/python3/dist-packages/" >> ~/.bashrc

WORKDIR /home/ros2_ws/src
RUN git clone https://github.com/frankarobotics/franka_description.git
RUN git clone -b ros2 https://github.com/EnricoMingo/LittleDog.git
WORKDIR /home/ros2_ws

RUN apt-get update && apt-get install -yy \
    terminator \
    nano \
    neovim \
    gedit \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --break-system-packages onnxruntime

# MuJoCo
RUN git clone https://github.com/deepmind/mujoco.git && \
    cd mujoco && \
    mkdir build && \
    cd build && \
    cmake .. && \
    cmake --build . && \
    cmake --install .

# OpenSoT
RUN apt-get update && apt-get install -y ros-jazzy-moveit-msgs ros-jazzy-moveit-core && rm -rf /var/lib/apt/lists/*
RUN git clone -b 4.0-devel_ros2 https://github.com/ADVRHumanoids/OpenSoT.git && \
    cd OpenSoT && \
    mkdir -p build && \
    cd build && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release .. && \
    make && \
    make install

# Livox lidar
# RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
#     cd Livox-SDK2 && \
#     mkdir build && \
#     cd build && \
#     cmake .. && \
#     make -j && \
#     make install

WORKDIR /opt/unitree_ros2_ws/src
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x

RUN git clone https://github.com/unitreerobotics/unitree_ros2 && \
    mv unitree_ros2/cyclonedds_ws/src/unitree /opt/unitree_ros2_ws/src && \
    rm -rf unitree_ros2

RUN apt-get update && apt-get install -yy ros-${ROS_DISTRO}-tf-transformations && rm -rf /var/lib/apt/lists/*

###### Unitree ROS2 Packages (system-wide in /opt)
WORKDIR /opt/unitree_ros2_ws/src
RUN git clone https://github.com/unitreerobotics/unitree_ros2.git && \
    rm -rf unitree_ros2/cyclonedds_ws/src/unitree

RUN apt-get update && apt-get install -yy \
    ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
    ros-${ROS_DISTRO}-rosidl-default-generators && \
    rm -rf /var/lib/apt/lists/*

RUN echo "export ROS_DOMAIN_ID=1" >> /root/.bashrc

# Also install in the venv, because rosidl_adapter may run with /home/.base/bin/python3
RUN /home/.base/bin/pip install -U pip setuptools wheel && \
    /home/.base/bin/pip install empy lark catkin_pkg mujoco robot_descriptions playground pyqt6 && \
    /home/.base/bin/pip install pygame numpy pin meshcat placo pin-pink pyspacemouse && \
    /home/.base/bin/pip install git+https://github.com/bglopez/python-easyhid.git && \
    /home/.base/bin/pip install hidapi scipy matplotlib

# Fix rosidl Python deps (empy / lark) + make shell env robust
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-empy \
    python3-lark \
    python3-catkin-pkg \
 && rm -rf /var/lib/apt/lists/*

RUN /home/.base/bin/pip install -U colcon-common-extensions

# Make sure interactive shells have ROS + venv ready
RUN echo "source /home/.base/bin/activate" >> /root/.bashrc && \
    echo "if [ -f /home/forest_ws/setup.bash ]; then source /home/forest_ws/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /opt/unitree_ros2_ws/install/setup.bash ]; then source /opt/unitree_ros2_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

# Shell helpers: rosenv + colcon via venv
RUN cat >> /root/.bashrc <<'EOF'

# Load ROS + venv + overlays
rosenv() {
  source /opt/ros/jazzy/setup.bash
  source /home/.base/bin/activate
  [ -f /home/forest_ws/setup.bash ] && source /home/forest_ws/setup.bash
  [ -f /opt/unitree_ros2_ws/install/setup.bash ] && source /opt/unitree_ros2_ws/install/setup.bash
  [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
}

# Always build using colcon from the venv (ensures entrypoints use /home/.base/bin/python3)
colcon_venv() {
  rosenv
  cd /ros2_ws || return
  /home/.base/bin/python3 -m colcon build --symlink-install "$@"
}

# Short alias
cb() { colcon_venv "$@"; }

# Optional: show python used
pywho() { python3 -c "import sys; print(sys.executable)"; }

EOF

RUN apt-get update && apt-get install -y --no-install-recommends \
    libhidapi-hidraw0 \
    libhidapi-libusb0 \
    libusb-1.0-0 \
    libhidapi-dev \
 && rm -rf /var/lib/apt/lists/*

RUN echo "rosenv 2>/dev/null || true" >> /root/.bashrc

RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /root/.bashrc

WORKDIR /opt/unitree_ros2_ws/src
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git /opt/unitree_ros2_ws/src/livox_ros_driver2

WORKDIR /
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    \
    # --- Patch missing <cstdint> includes for GCC 13 / Ubuntu 24.04 ---
    for f in sdk_core/comm/define.h sdk_core/logger_handler/file_manager.h; do \
      if [ -f "$f" ] && ! grep -qE '^[[:space:]]*#include[[:space:]]*<cstdint>' "$f"; then \
        echo "Patching $f (add <cstdint>)"; \
        # insert after #pragma once if present, else at top
        if grep -qE '^[[:space:]]*#pragma[[:space:]]+once' "$f"; then \
          sed -i '/^[[:space:]]*#pragma[[:space:]]\+once/a #include <cstdint>' "$f"; \
        else \
          sed -i '1i #include <cstdint>' "$f"; \
        fi; \
      fi; \
    done && \
    \
    mkdir -p build && cd build && \
    cmake .. && \
    make -j"$(nproc)" && \
    make install

WORKDIR /opt/unitree_ros2_ws/src/livox_ros_driver2
RUN rm -f package.xml && \
    cp package_ROS2.xml package.xml && \
    cp -rf launch_ROS2 launch

WORKDIR /opt/unitree_ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /home/forest_ws/setup.bash && \
    /home/.base/bin/python3 -m colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

# unitree_sdk2_python (needs cyclonedds built above)
WORKDIR /
RUN git clone https://github.com/lnotspotl/unitree_sdk2_python.git \
 && cd unitree_sdk2_python \
 && git checkout 7c661d27f4ae064ffd0dd633fd9d5b518ef0b508 \
 && CYCLONEDDS_HOME=/opt/unitree_ros2_ws/install/cyclonedds /home/.base/bin/pip install -e .

RUN apt-get update && apt-get install -y --no-install-recommends \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-xinput0 \
    libxcb-shape0 \
    python3-gi \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-* \
    gstreamer1.0-libav && \
    rm -rf /var/lib/apt/lists/*

RUN /home/.base/bin/pip install --break-system-packages pyqt6 pin meshcat evdev pyrealsense2 opencv-python

WORKDIR /opt/unitree_ros2_ws/src
RUN git clone https://github.com/CDonosoK/astroviz_interfaces.git

# Build astroviz_interfaces (cloned above) so its message types are generated
WORKDIR /opt/unitree_ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /home/forest_ws/setup.bash && \
    /home/.base/bin/python3 -m colcon build --packages-select astroviz_interfaces

# MOLA packages
RUN apt-get update && apt-get install -y \
 ros-${ROS_DISTRO}-mola \
 ros-${ROS_DISTRO}-mola-state-estimation \
 ros-${ROS_DISTRO}-mola-lidar-odometry && \
 rm -rf /var/lib/apt/lists/*

RUN /home/.base/bin/pip install placo

RUN /home/.base/bin/pip install --no-cache-dir cyclonedds

ENV CRC_SRC=/unitree_sdk2_python/unitree_sdk2py/utils/lib/crc_amd64.so
ENV CRC_DST=/unitree_sdk2py/lib/python3.12/site-packages/unitree_sdk2py/utils/lib

RUN set -eux; \
    mkdir -p "$CRC_DST"; \
    cp "$CRC_SRC" "$CRC_DST/"; \
    chmod +x "$CRC_DST/crc_amd64.so"

RUN rm -f /home/.base/bin/register-python-argcomplete || true

# CycloneDDS network setup (replaces setup_uri.sh)
RUN cat >> /root/.bashrc <<'DDSEOF'

# CycloneDDS configuration via G1_INTERFACE env var
if [ -z "$G1_INTERFACE" ]; then
  echo "[WARNING] G1_INTERFACE is not set. CycloneDDS will not be configured."
  echo "          Set it to your network interface (e.g. eth0) or 'lo' for localhost."
else
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${G1_INTERFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
  if [ "$G1_INTERFACE" = "lo" ]; then
    ip link set lo multicast on 2>/dev/null
  fi
fi
DDSEOF

CMD ["bash"]
