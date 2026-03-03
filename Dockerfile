# ─────────────────────────────────────────────────────────────
#  Lite3 Robot — SDK Deploy Container (ROS2 Humble)
#  Base: Ubuntu 22.04 + CUDA 12.1 (works with NVIDIA GPU)
#  Includes: ROS2 Humble, MuJoCo, sdk_deploy, build tools
# ─────────────────────────────────────────────────────────────
FROM nvidia/cuda:12.1.0-base-ubuntu22.04

# Avoid interactive prompts during apt
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Kolkata
ENV ROS_DISTRO=humble

# ── System packages ──────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    # Build tools
    git \
    cmake \
    build-essential \
    make \
    wget \
    curl \
    unzip \
    gnupg \
    lsb-release \
    # Lite3 specific dep (for backward-cpp crash debug)
    libdw-dev \
    # Input device support (gamepad/keyboard)
    libevdev-dev \
    # Python
    python3 \
    python3-pip \
    python3-dev \
    # MuJoCo display
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libgles2-mesa \
    libglfw3 \
    libglfw3-dev \
    libosmesa6-dev \
    libglew-dev \
    # X11 for GUI window on your host screen
    libx11-6 \
    libxext6 \
    libxrender1 \
    x11-apps \
    # SSH (useful for robot connection later)
    openssh-client \
    # Utilities
    htop \
    nano \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# ── Install ROS2 Humble ──────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ── backward-cpp (crash debug tool) ─────────────────────────
RUN wget https://raw.githubusercontent.com/bombela/backward-cpp/master/backward.hpp \
    && mv backward.hpp /usr/include/

# ── Python packages ──────────────────────────────────────────
RUN pip3 install --upgrade pip && \
    pip3 install \
    "numpy<2.0" \
    mujoco \
    onnx \
    onnxruntime \
    matplotlib \
    scipy

# ── Clone sdk_deploy ─────────────────────────────────────────
WORKDIR /workspace
RUN git clone https://github.com/DeepRoboticsLab/sdk_deploy.git && \
    cd sdk_deploy && \
    find . -name ".gitmodules" | xargs grep -l "eigen" 2>/dev/null | \
    xargs sed -i 's|https://gitlab.com/libeigen/eigen.git|https://github.com/live-clones/eigen.git|g' 2>/dev/null || true && \
    git submodule sync && \
    git submodule update --init --recursive

# ── Build with ROS2 colcon ───────────────────────────────────
WORKDIR /workspace/sdk_deploy
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build \
        --packages-up-to lite3_sdk_deploy \
        --cmake-args -DBUILD_PLATFORM=x86"

# ── Environment variables ────────────────────────────────────
ENV MUJOCO_GL=glfw
ENV DISPLAY=:0
ENV ROS_DOMAIN_ID=1
ENV PYTHONPATH=/workspace/sdk_deploy:$PYTHONPATH

# ── Auto-source ROS2 + workspace in every shell ──────────────
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/sdk_deploy/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc && \
    echo 'alias sim="cd /workspace/sdk_deploy/src/Lite3_sdk_deploy/interface/robot/simulation"' >> ~/.bashrc && \
    echo 'alias deploy="ros2 run lite3_sdk_deploy rl_deploy"' >> ~/.bashrc && \
    echo 'alias ws="cd /workspace/sdk_deploy"' >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "echo ''" >> ~/.bashrc && \
    echo "echo '  🐕  Lite3 SDK Deploy Container Ready (ROS2 Humble)'" >> ~/.bashrc && \
    echo "echo ''" >> ~/.bashrc && \
    echo "echo '  Terminal 1:  ros2 run lite3_sdk_deploy rl_deploy'" >> ~/.bashrc && \
    echo "echo ''" >> ~/.bashrc && \
    echo "echo '  Terminal 2:  python3 src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py'" >> ~/.bashrc && \
    echo "echo ''" >> ~/.bashrc && \
    echo "echo '  Keys: z=stand  c=RL mode  wasd=move  qe=rotate'" >> ~/.bashrc && \
    echo "echo ''"  >> ~/.bashrc

WORKDIR /workspace/sdk_deploy