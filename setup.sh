#!/bin/bash
# ─────────────────────────────────────────────────────────────
#  Lite3 Docker Setup Script
#  Run this once on your Ubuntu machine:
#  chmod +x setup.sh && ./setup.sh
# ─────────────────────────────────────────────────────────────

set -e  # stop on any error

echo ""
echo "🐕  Lite3 Robot Sim — Setup Script"
echo "────────────────────────────────────"

# ── Step 1: Check Docker is installed ────────────────────────
echo ""
echo "→ Checking Docker..."
if ! command -v docker &> /dev/null; then
    echo "  Docker not found. Installing..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker $USER
    echo "  ✅ Docker installed. NOTE: Log out and back in for group to apply."
else
    echo "  ✅ Docker found: $(docker --version)"
fi

# ── Step 2: Check Docker Compose ─────────────────────────────
echo ""
echo "→ Checking Docker Compose..."
if ! command -v docker-compose &> /dev/null; then
    echo "  Installing docker-compose..."
    sudo apt-get install -y docker-compose-plugin
fi
echo "  ✅ Docker Compose ready"

# ── Step 3: Check NVIDIA Container Toolkit ───────────────────
echo ""
echo "→ Checking NVIDIA Container Toolkit (for GPU in Docker)..."
if ! dpkg -l | grep -q nvidia-container-toolkit; then
    echo "  Installing NVIDIA Container Toolkit..."
    distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
        sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    echo "  ✅ NVIDIA Container Toolkit installed"
else
    echo "  ✅ NVIDIA Container Toolkit already installed"
fi

# ── Step 4: Allow Docker to show GUI on your screen ──────────
echo ""
echo "→ Setting up X11 display access..."
xhost +local:docker
echo "  ✅ Display access granted"

# ── Step 5: Create your_code folder ──────────────────────────
echo ""
echo "→ Creating your_code folder (put your scripts here)..."
mkdir -p ./your_code
echo "  ✅ ./your_code/ ready"

# ── Step 6: Build the Docker image ───────────────────────────
echo ""
echo "→ Building Docker image (this takes 5-10 minutes first time)..."
echo "  Downloading Ubuntu + CUDA + MuJoCo + Lite3_rl_deploy..."
docker compose build

echo ""
echo "────────────────────────────────────"
echo "✅  Setup complete!"
echo ""
echo "HOW TO RUN:"
echo ""
echo "  Start container:"
echo "    docker compose up -d"
echo ""
echo "  Open Terminal 1 (MuJoCo simulation window):"
echo "    docker exec -it lite3_sim bash"
echo "    cd interface/robot/simulation"
echo "    python3 mujoco_simulation.py"
echo ""
echo "  Open Terminal 2 (robot controller):"
echo "    docker exec -it lite3_sim bash"
echo "    cd build && ./rl_deploy"
echo ""
echo "  Keys inside controller:"
echo "    z     = stand up"
echo "    c     = enter RL control mode"
echo "    w/s   = forward / backward"
echo "    a/d   = strafe left / right"
echo "    q/e   = rotate left / right"
echo ""
echo "  Stop everything:"
echo "    docker compose down"
echo "────────────────────────────────────"
