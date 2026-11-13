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
- `Module1_LearnToDrive_Student.ipynb`

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


## III. Testing your code for the Autograder

The autograder builds `rl_ppo`, checks your notebook and your deploy node without a simulator, then drives one lap of Spielberg in the [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros/tree/dev-jazzy) simulator with your trained policy.

**Your notebook.** Commit your completed notebook: download it from Colab and save it over `resources/Module1_LearnToDrive_Student.ipynb`. The autograder never trains. It takes `F1TenthRLWrapper`, `ActorCritic`, `compute_gae` and the loss inside your training loop out of the notebook and tests each TODO on made-up inputs, so keep those names, and keep `ratio`, `policy_loss`, `value_loss` and `loss` in the training loop.

**Your policy.** Commit the two files `export_ppo_onnx.py` writes, `ppo_policy.onnx` and `ppo_deploy_config.json`, in `rl_ppo/scripts/`. `CMakeLists.txt` installs that folder and the launch files load the policy from there.

**Your node, without the simulator.** The autograder starts your node with a policy of its own, through the `onnx_path` and `config_path` parameters (keep them), and plays the car. With RB held your node must publish on `/drive`, with RB released it must stop, and the observation it builds from `/scan`, `/odom` and the servo position must be normalised as in training.

**The lap.** To drive it on your laptop, set the map in the simulator's `config/sim.yaml`:

| Track | `map_path` | `sx`, `sy`, `stheta` | Graded run |
| --- | --- | --- | --- |
| Spielberg | `'maps/Spielberg'` | `14.59`, `3.92`, `-2.877` | one lap, in the direction of its centerline |

Start the simulator, then in two more terminals:

```bash
ros2 run rl_ppo sim_car.py               # plays the rest of the car
ros2 launch rl_ppo spielberg_launch.py   # what the autograder runs
```

The simulator has no joystick, VESC or servo. `sim_car.py` plays them: it holds RB on `/joy`, republishes `/ego_racecar/odom` on `/odom` and reports the car's steering angle on `/sensors/servo_position_command`, so your node runs exactly as on the car. The autograder does the same. `spielberg_launch.py` lifts `max_speed_cap`: its 2.0 m/s default is a safety setting for the real car, and in the simulator your policy should drive at the speeds it was trained for. Your lap time is the `completed lap N, last lap X s` line in the simulator's log.

## IV. Deliverables and Submission

As in previous labs, this is a team assignment.

### a. Accepting the Assignment

Each team member can accept the assignment by running:

```bash
gh student accept RoboRacer-Class ese-6150 lab-9-reinforcement-learning
```

The team member that accepts the assignment first creates the team's shared repository, `ese-6150-lab-9-reinforcement-learning-group-<n>`. Make sure you coordinate pushes, branches and commits. One submission is the whole team's submission, so every member gets the same grade for the lab.

### b. Deliverables

- **Deliverable 1**: Your completed notebook, your `rl_ppo` package with the completed deploy node, and your trained policy (`ppo_policy.onnx` and `ppo_deploy_config.json` in `rl_ppo/scripts/`). Your policy should complete a lap of Spielberg in the simulator without touching a wall. The leaderboard tracks your team's lap time.

- **Deliverable 2**: Links to two videos in **`SUBMISSION.md`** (YouTube unlisted, or Google Drive shared as **"Anyone with the link can view"**): your policy driving in simulation, and on the real car.

### c. Submitting

You can commit and push your work as often as you need, as a plain push does **not** count as a submission. When your team is ready to submit, any member can push a tagged commit named `submission` which then counts as a submission for the whole team. You can do so using:

```bash
git push                            # your commits
git tag submission
git push origin submission          # this triggers the autograder
```

### d. Grading

The autograder posts your score as a **Release** on your repo (check the Releases page or the commit's status check a few minutes after you tag). To resubmit, move the tag to a new commit:

```bash
git tag -f submission
git push --force origin submission
```

The `submission` push with the best grade is counted as your team's final submission and grade.

**The autograder finds your work by name.** Package `rl_ppo`, launch files `spielberg_launch.py` and `ppo_deploy.launch.py`, the policy files in `rl_ppo/scripts/`, the notebook's class and function names, and `AckermannDriveStamped` on `/drive`. Otherwise the autograder will not be able to grade your work and your submission may get the wrong grade.

**Only the topics the lab needs.** Your nodes may read `/scan`, `/odom` (or `/ego_racecar/odom`), `/joy` and `/sensors/servo_position_command`, and publish to `/drive` and to topics only your own nodes use. The autograder watches the ROS graph while your code runs: a node that reads the lap counter or the collision flag, or publishes on a topic the simulator listens to (`/initialpose`, which teleports the car, ...) gets a 0 for the deployment node.

## V: Grading Rubric
- Compilation: **10** Points (autograded)
- Correct MDP formulation and working training pipeline: **40** Points (autograded from your notebook, without the simulator: observation **5**, action scaling **5**, reward **10**, actor-critic forward pass **5**, GAE **5**, PPO clipped loss **10**)
- Working deployment node (completed TODOs): **30** Points
  - **20** Points (autograded without the simulator: drives with RB held **5**, stops with RB released **5**, observation normalised as in training **10**)
  - **10** Points (autograded in simulation: one lap of `Spielberg` with your trained policy without touching a wall; a run that ends early earns partial credit for the fraction covered; the lap time goes to the leaderboard)
- Videos (TA-graded from the links in `SUBMISSION.md`):
   - In sim: **10** Points
   - On Car: **10** Points
