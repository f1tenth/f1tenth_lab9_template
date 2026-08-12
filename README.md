# Lab 9: Reinforcement Learning

## I. Learning Goals

- Markov Decision Process (MDP) formulation for autonomous racing
- Proximal Policy Optimization (PPO)
- Sim-to-real transfer of a learned policy
- Deploying a trained neural network policy on the real car via ONNX

## II. Overview

In this lab you'll train a PPO agent to drive the F1TENTH car in simulation, then deploy the trained policy on the real car.

### a. Problem Formulation

We treat driving as a Markov Decision Process:
- **State**: downsampled LiDAR scan + current speed + current steering angle
- **Action**: `[steering_cmd, speed_cmd]`, each in `[-1, 1]`, scaled to physical steering/speed limits
- **Reward**: your choice — commonly a combination of progress along the track and a penalty for collisions

### b. Training

Train your PPO agent in simulation using the Colab notebooks in [`resources/`](resources/):
- `rl_roboracer_lec1_part1_student.ipynb`
- `rl_roboracer_lec1_part2_student.ipynb`

These walk through setting up the F1TENTH gym environment, defining the observation/reward, and training with Stable-Baselines3.

### c. Deployment

Once you have a trained policy, deploy it on the car using the `rl_ppo` package:

1. **Export** — run `rl_ppo/scripts/export_ppo_onnx.py` on your training machine to convert the trained model to ONNX and write out `ppo_policy.onnx` + `ppo_deploy_config.json`.
2. **Copy** — `scp` both files to the car, into `rl_ppo/scripts/` (see `rl_ppo/scripts/README.md`).
3. **Complete the TODOs** — finish the sensor wiring and observation normalization in `rl_ppo/scripts/ppo_deploy_node.py` (or the C++ scaffold in `rl_ppo/src/ppo_deploy_node.cpp`) so it matches your training setup.
4. **Launch** — `ros2 launch rl_ppo ppo_deploy.launch.py`.

## VII. Deliverables

- **Deliverable 1**: Commit your `rl_ppo` package to GitHub. Your trained policy should complete laps in simulation.
- **Deliverable 2**: Submit a link to a YouTube video showing the car driving with your PPO policy.

## VIII: Grading Rubric
- Compilation: **10** Points
- Correct MDP formulation and working training pipeline: **50** Points
- Working deployment node (completed TODOs): **20** Points
- Videos:
   - In sim: **10** Points
   - On Car: **10** Points
