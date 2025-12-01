# Quickstart Guide: Physical AI & Humanoid Robotics Course

This guide provides a rapid introduction to setting up your development environment and running your first simulated humanoid robot. It assumes you have an NVIDIA GPU (RTX 4070 Ti or better recommended) and are running Ubuntu 22.04 LTS.

## 1. System Requirements

Ensure your Digital Twin Workstation meets the following minimum specifications:
- **OS**: Ubuntu 22.04 LTS
- **GPU**: NVIDIA RTX (12GB+ VRAM, e.g., RTX 4070 Ti)
- **RAM**: 64 GB minimum
- **CPU**: Intel Core i7 (13th Gen+) or AMD Ryzen 9 equivalent
- **Disk Space**: 200 GB SSD (minimum)

## 2. Install NVIDIA Drivers and Docker

Follow the official NVIDIA documentation to install the latest GPU drivers. Then, install Docker and NVIDIA Container Toolkit:

```bash
# Install Docker
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add current user to the docker group
sudo usermod -aG docker $USER
newgrp docker

# Install NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && sudo curl -s -L https://nvidia.github.io/libnvidia-container/ubuntu22.04/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

## 3. Install ROS 2 Humble (Containerized)

For consistency and ease of setup, we recommend using a Docker container for your ROS 2 Humble environment. This project will provide a `Dockerfile` and `docker-compose.yml`.

```bash
# Clone the course repository (if you haven't already)
# git clone [YOUR_REPO_URL] E:\AISpec-Driven-Hackathon_book
cd E:\AISpec-Driven-Hackathon_book

# Build the ROS 2 Humble Docker image (this may take a while)
docker compose build ros2_dev

# Run the ROS 2 development container
docker compose up -d ros2_dev

# Access the container's shell
docker exec -it ros2_dev bash

# Inside the container, source ROS 2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash # If you have local ROS 2 packages
```

## 4. Install NVIDIA Isaac Sim (in Docker)

Isaac Sim also runs in Docker. Follow the NVIDIA Omniverse Launcher instructions to download and run the Isaac Sim Docker container. This will typically involve:

1.  Download and install Omniverse Launcher.
2.  Install Isaac Sim via the Launcher.
3.  Open the Isaac Sim installation directory and locate the `isaac_sim.sh` script.
4.  Run it from your host machine to launch Isaac Sim:
    ```bash
    cd /path/to/isaac-sim-app
    ./isaac_sim.sh
    ```

Ensure Isaac Sim is running before attempting to connect ROS 2.

## 5. Run Your First Simulated Humanoid (ROS 2 & Isaac Sim)

Once both your ROS 2 development container and Isaac Sim are running, you can launch a basic simulation.

1.  **In Isaac Sim**: Load a sample humanoid robot (e.g., from `isaac_sim_assets/Robots/Humanoid/Franka/Franka.usd` or similar provided course assets) and an environment.

2.  **In your ROS 2 dev container shell**: Navigate to the `physical_ai_robotics_pkg` and launch a basic control node.

    ```bash
    # In the ROS 2 container
    cd src/physical_ai_robotics_pkg
    ros2 launch physical_ai_robotics_pkg basic_humanoid_sim.launch.py
    ```

3.  **Observe**: You should see the simulated humanoid in Isaac Sim respond to basic commands or maintain a stable pose. This confirms your ROS 2 and Isaac Sim integration is working.

## Next Steps

- Explore the provided tutorials and example projects in `src/physical_ai_robotics_pkg`.
- Review the `data-model.md` and `contracts/` for detailed system interfaces.
- Begin working on Chapter 2 modules.
