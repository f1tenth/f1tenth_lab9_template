# Lab 9: Reinforcement Learning

## I. Learning Goals

- Markov Decision Process (MDP) formulation for autonomous racing
- Proximal Policy Optimization (PPO)
- Sim-to-real transfer of a learned policy
- Deploying a trained neural network policy on the real car via ONNX

## II. Overview

In this lab you'll train a policy network with PPO to drive the F1TENTH car in simulation, then deploy the trained policy on the real car.

### a. Problem Formulation

We treat driving as a Markov Decision Process:
- **State**: downsampled LiDAR scan + current speed + current steering angle
- **Action**: `[steering_cmd, speed_cmd]`, each in `[-1, 1]`, scaled to physical steering/speed limits
- **Reward**: your choice — commonly a combination of progress along the track and a penalty for collisions

### b. Training

Train your PPO agent in simulation using the Colab notebook in [`resources/`](resources/):
- `PPO_from_Scratch_LearnToDrive_Student.ipynb`

This walks through setting up the F1TENTH gym environment, defining the observation/reward, and implementing PPO from scratch in PyTorch. Training saves two files under `models_scratch/`: `ppo_scratch.pt` (the policy weights) and `vecnormalize.pkl` (reward-normalization stats).

### c. Deployment

Once you have a trained policy, deploy it on the car using the `rl_ppo` package:

1. **Download from Colab** — Colab zips your models for you (`models_archive.zip`, containing `models_scratch/`). Download it and unzip locally:
   ```
   unzip models_archive.zip -d models_archive
   # -> models_archive/content/models_scratch/ppo_scratch.pt (+ vecnormalize.pkl)
   ```
2. **Export** — Use a venv to install the export script's Python dependencies, then run it to convert `ppo_scratch.pt` to ONNX and write out `ppo_policy.onnx` + `ppo_deploy_config.json`:
   ```
   python3 -m venv ~/rl_export_venv && source ~/rl_export_venv/bin/activate
   pip install torch onnx onnxruntime
   python3 rl_ppo/scripts/export_ppo_onnx.py \
       --model models_archive/content/models_scratch/ppo_scratch.pt \
       --vecnorm models_archive/content/models_scratch/vecnormalize.pkl \
       --out-dir deploy
   ```
3. **Copy** — `scp` both output files to the car, into `rl_ppo/scripts/`:
   ```
   scp deploy/ppo_policy.onnx deploy/ppo_deploy_config.json \
       nvidia@<car-ip>:~/f1tenth_ws/src/rl_ppo/scripts/
   ```
4. **Install build dependencies on the car** — only if you're using the **C++** deploy node (`rl_ppo/src/ppo_deploy_node.cpp`). **Skip this step if you're using Python** — it only needs `pip install onnxruntime` on the car.
   ```
   sudo apt install nlohmann-json3-dev
   ```
   Install ONNX Runtime's C++ package:
   ```
   cd ~
   wget https://github.com/microsoft/onnxruntime/releases/download/v1.17.1/onnxruntime-linux-x64-1.17.1.tgz
   tar xzf onnxruntime-linux-x64-1.17.1.tgz
   export CMAKE_PREFIX_PATH=~/onnxruntime-linux-x64-1.17.1:$CMAKE_PREFIX_PATH
   ```
   Add the `export CMAKE_PREFIX_PATH=...` line to your `~/.bashrc` to set it for future builds.
5. **Build**:
   ```
   cd ~/f1tenth_ws
   colcon build --packages-select rl_ppo
   source install/setup.bash
   ```
6. **Complete the TODOs** — finish the sensor wiring and observation normalization in `rl_ppo/scripts/ppo_deploy_node.py` (or the C++ scaffold in `rl_ppo/src/ppo_deploy_node.cpp`) so it matches your training setup.
7. **Launch** — `ros2 launch rl_ppo ppo_deploy.launch.py`.


## VII. Deliverables

- **Deliverable 1**: Commit your `rl_ppo` package to GitHub. Your trained policy should complete laps in simulation.
- **Deliverable 2**: Submit a link to a YouTube video showing the car driving with your PPO policy.

## VIII: Grading Rubric
- Compilation: **10** Points
- Correct MDP formulation and working training pipeline: **40** Points
- Working deployment node (completed TODOs): **30** Points
- Videos:
   - In sim: **10** Points
   - On Car: **10** Points
