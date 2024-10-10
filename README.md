# Manipulation and Environmental Robotics Lab (mer_lab)
This project provides a framework for manipulation and robotic vision tasks, focusing on active perception and imitation learning. It is designed to train and test reinforcement learning (RL) models on both simulated and real-world robotic platforms like the Franka Emika Panda.

---

## Project File Structure

- **ros_ws/**
  - **src/**: Contains ROS packages and projects.
    - **general_packages/**: General ROS packages usable across multiple projects.
    - **projects/**: Specific to individual projects, including:
      - **active_vision/**: Implements active vision systems for robotics.
        - **models/ycbAV/**: Contains YCB object dataset files (PCD and SDF).
        - **scripts/RL/**: Imitation learning scripts and tools.
        - **src/**: ROS nodes and services used for active vision.
    - **franka_ros/**: Contains configurations for the Franka Emika Panda.
    - **relaxed_ik_ros1/**: Package for Relaxed IK integration with ROS.

---

## Setup Instructions

### ROS Workspace Setup
1. Navigate to your desired workspace directory:
    ```bash
    cd /path/to/your/workspace
    ```
2. Clone the repository:
    ```bash
    git clone https://www.github.com/berkcalli/mer_lab.git
    cd mer_lab
    ```
3. Install dependencies:
    ```bash
    rosdep install --from-paths --ignore-src ros_ws -y
    ```
4. Enter the ROS workspace:
    ```bash
    cd ros_ws
    ```
5. Clone Relaxed IK repositories:
    ```bash
    git clone https://github.com/uwgraphics/relaxed_ik_ros1/ src/
    git clone https://github.com/uwgraphics/relaxed_ik_core/tree/collision-ik src/relaxed_ik_ros1/relaxed_ik_core
    ```
6. Follow the instructions in the [Relaxed IK Core repository](https://github.com/uwgraphics/relaxed_ik_core/tree/collision-ik) to set up Franka YAML files and install prerequisites.
7. Build the workspace:
    ```bash
    catkin_make -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES="active_vision;active_vision_moveit_config;moveit_planner;franka_pos_grasping_gazebo;relaxed_ik_ros1;motion_planning"
    ```

### Add to Bash Profile
To ensure ROS Noetic and `mer_lab` are set up correctly, add the following to your `~/.bashrc` file:

```bash
# ROS Noetic and mer_lab setup
source /opt/ros/noetic/setup.bash
source /home/uthira/mer_lab/ros_ws/devel/setup.bash
export ROS_PACKAGE_PATH=/home/uthira/mer_lab/ros_ws/src/franka_ros/panda_moveit_config:$ROS_PACKAGE_PATH
```

---

## YCB Dataset Models
1. Ensure that YCB object files (PCD and SDF) are located at `ros_ws/src/projects/active_vision/models/ycbAV`. 
2. Alternatively, download the YCB dataset from the provided link and adjust the paths inside the SDF files accordingly.

---

## Imitation Learning

### Training
1. Set up a Python virtual environment:
    ```bash
    python3 -m venv ~/Envs/AVRL
    source ~/Envs/AVRL/bin/activate
    ```
2. Install dependencies:
    ```bash
    pip install -r requirements.txt
    ```
3. Install CUDA driver and toolkit:
    - Add these lines to your `~/.bashrc`:
      ```bash
      export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda-11.8/lib64
      export CUDA_HOME=/usr/local/cuda-11.8
      export PATH=${CUDA_HOME}/bin:${PATH}
      ```
4. Install PyTorch, TensorFlow, and TensorRT (ensure compatibility with CUDA).
5. Navigate to the RL scripts:
    ```bash
    cd ~/ros_ws/src/projects/active_vision/scripts/RL/
    python3 Imitation.py
    ```
6. Run the simulation in a separate terminal:
    ```bash
    roslaunch active_vision workspace.launch visual:=ON simulationMode:=SIMULATION
    ```
7. Training data will be logged in `ros_ws/src/projects/active_vision/scripts/RL/features_var_log.txt`.
8. Visualize training results by running:
    ```bash
    python3 ~/ros_ws/src/projects/active_vision/scripts/RL/ILSummarizer.py
    ```
9. For multiple feature testing, modify `feature_type` in `kinectService.cpp` and update the observation space in `gazeboenv.py` and `Imitation.py`.

### Testing
1. Choose between Franka simulation or real robot mode.
2. To start the policy evaluator, run:
    ```bash
    rosrun active_vision policyEvaluator.sh /misc/State_Vector/ obj_2_3_g5.csv
    ```
    - Select objects to test, choose the simulation mode, and load the trained model file.
3. In a separate terminal, launch Relaxed IK:
    ```bash
    roslaunch relaxed_ik_ros1 demo.launch
    ```

### Running on Franka Robot (Real Mode)
1. Power on the Franka Emika Panda robot.
2. Set up VNC to connect to the robot PC.
3. Ensure the following settings in `~/.bashrc` on your machine:
    ```bash
    export ROS_MASTER_URI=http://pandarobot:11311
    export ROS_IP=172.16.1.11
    export ROS_HOSTNAME="uthira-IdeaPad-L340-15IRH-Gaming"
    ```
4. Run Franka Control on the robot PC:
    ```bash
    rosrun franka_control franka_control_node
    ```
5. In your remote terminal, start the policy evaluator and launch Relaxed IK as described above.
