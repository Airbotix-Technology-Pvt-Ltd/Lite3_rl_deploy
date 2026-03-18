# ─────────────────────────────────────────────────────────────
#  Lite3 RL Deploy Container
#  Base: Ubuntu 22.04 + CUDA 12.1
#  Build: cmake + make  (no ROS2, no sdk_deploy)
#  Deps:  pybullet, mujoco, onnxruntime, libevdev
# ─────────────────────────────────────────────────────────────
FROM nvidia/cuda:12.1.0-base-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Kolkata

# ── System packages ──────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    # Build tools
    git \
    cmake \
    build-essential \
    make \
    wget \
    curl \
    # Lite3 dep — crash debug
    libdw-dev \
    # Input device support (gamepad/keyboard)
    libevdev-dev \
    # Python
    python3 \
    python3-pip \
    python3-dev \
    # MuJoCo / OpenGL display
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libgles2-mesa \
    libglfw3 \
    libglfw3-dev \
    libosmesa6-dev \
    libglew-dev \
    # X11 for GUI window on host screen
    libx11-6 \
    libxext6 \
    libxrender1 \
    x11-apps \
    # Utilities
    htop \
    nano \
    && rm -rf /var/lib/apt/lists/*

# ── backward-cpp (crash debug tool) ─────────────────────────
RUN wget https://raw.githubusercontent.com/bombela/backward-cpp/master/backward.hpp \
    && mv backward.hpp /usr/include/

# ── Python packages ──────────────────────────────────────────
RUN pip3 install --upgrade pip && \
    pip3 install \
    "numpy<2.0" \
    mujoco \
    pybullet \
    onnx \
    onnxruntime \
    matplotlib \
    scipy \
    torch

# ── Copy source into container ───────────────────────────────
# (volume mount in docker-compose will overlay this at runtime)
WORKDIR /workspace
COPY . /workspace/Lite3_rl_deploy/

# ── Build ────────────────────────────────────────────────────
WORKDIR /workspace
RUN mkdir -p /workspace/lite3_build && cd /workspace/lite3_build && \
    cmake /workspace/Lite3_rl_deploy -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DSEND_REMOTE=OFF && \
    make -j$(nproc)

# ── Environment ──────────────────────────────────────────────
ENV MUJOCO_GL=glfw
ENV DISPLAY=:0

# ── Extra pip packages (add here to avoid full rebuild) ─────
RUN pip3 install colorama

# ── Symlink so binary finds policy at /workspace/policy/ ─────
# Binary lives in /workspace/lite3_build/, resolves ../policy/ -> /workspace/policy/
RUN ln -sf /workspace/Lite3_rl_deploy/policy /workspace/policy

# ── Shell config (aliases + welcome message) ─────────────────
COPY .bashrc_lite3 /tmp/.bashrc_lite3
RUN cat /tmp/.bashrc_lite3 >> ~/.bashrc

WORKDIR /workspace/Lite3_rl_deploy