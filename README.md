# Lite3 RL Deployment Hub (Airbotix Fork)

This repository is optimized for **Sim-to-Real Policy Transfer** on the Lite3 quadruped platform.

---

### 🌐 Project Central Hubs
- [**Master Hub (Root)**](https://github.com/Airbotix-Technology-Pvt-Ltd/Lite3Robot): Mission, specialized workspaces, and organizational identity.
- [**Contributors Hub**](../Contributors.md): Full technical attribution for the Airbotix development team.

---

## 🛠️ Technical Contributions (Airbotix)

The following components have been implemented for this deployment hub:

- **Unified Docker Hub**: Engineered a Docker environment (including MuJoCo, ONNX, and PyBullet) for reproducible testing.
- **Verification & Validation**: Completed the policy integrity verification (March 18th "perfect" baseline) for the Lite3 interface.

---

## 🐳 Unified Docker Hub (Recommended)

To avoid dependency conflicts and ensure high-fidelity simulation performance, use the provided Docker environment.

### **1. Build & Start Container**
```bash
# Allow Docker access to your X11 display (for MuJoCo window)
xhost +local:docker

# Build and start the container (headless build, GUI run)
docker-compose up --build -d
```

### **2. Run Simulation (MuJoCo)**
```bash
# Terminal 1: Start the MuJoCo simulation interface
docker exec -it lite3_sim python3 interface/robot/simulation/mujoco_simulation.py
```

### **3. Run RL Policy**
```bash
# Terminal 2: Execute the compiled C++ policy runner
docker exec -it lite3_sim /workspace/lite3_build/rl_deploy
```

---

## ❤️ Credits & Tribute
We pay tribute to **DeepRobotics** for providing the foundational `Lite3_rl_deploy` framework and robot models.

---
*Airbotix Technology Pvt Ltd - Lite3 P2P Autonomous Navigation Project*
*See our [**Contributors Hub**](../Contributors.md) for full project attribution.*
