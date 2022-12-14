# Lab 9: Model Predictive Control

## I. Learning Goals

- Convex Optimization
- Linearization and Discretization
- Optimal Control

## II. Formulating the MPC problem

Before starting the lab, make sure you understand the formulation of the MPC problem from the lecture. We'll briefly go over it again here. The goal of the MPC is to generate valid input controls for $T$ steps ahead that controls the vehicle and follow the reference trajectory as close as possible.

### a. State and Dynamics Model

We use a kinematic model of the vehicle. The state space is $z=[x, y, \theta, v]$. And the input vector is $u=[a, \delta]$ where $a$ is acceleration and $\delta$ is steering angle. The kinematic model ODEs are:
$$
\dot{x}=v\cos(\theta)
$$

$$
\dot{y}=v\sin(\theta)
$$

$$
\dot{v}=a
$$

$$
\dot{\theta}=\frac{v\tan(\delta)}{L}
$$

Where $L$ is the wheelbase of the vehicle. In summary, we can write the ODEs as:

$$
\dot{z}=\frac{\partial}{z}z=f(z, u)=A'z+B'u
$$

You can write out the matrix representation of $A'$ and $B'$. We highly recommend writing out the system of equation in the matrix form.

### b. Objective Function

First we formulate the objective function of the optimization. We want to minimize three objectives:
1. Deviation of the vehicle from the reference trajectory. Final state deviation weighted by Qf, other state deviations weighted by Q.
2. Influence of the control inputs. Weighted by R.
3. Difference between one control input and the next control input. Weighted by Rd.

<!-- $$\text{minimize}~~~u^TRu + (x-x_{\text{ref}})_{0,\ldots,T-1}^TQ(x-x_{\text{ref}})_{0,\ldots,T-1} + (x-x_{\text{ref}})_{T}^TQ_f(x-x_{\text{ref}})_{T} + (u_{1,\ldots,T}-u_{0,\ldots,T-1})^TR_d(u_{1,\ldots,T}-u_{0,\ldots,T-1})$$ -->

$$
\text{minimize}~~ Q_{f}\left(z_{T, r e f}-z_{T}\right)^{2}+Q \sum_{t=0}^{T-1}\left(z_{t, r e f}-z_{t}\right)^{2}+R \sum_{t=0}^{T} u_{t}^{2}+R_{d} \sum_{t=0}^{T-1}\left(u_{t+1}-u_{t}\right)^{2}
$$

### c. Constraints

We then formulate the constraints for the optimization problem. The constraints should inclue:
1. Future vehicle states must follow the linearized vehicle dynamic model.
   $$z_{t+1}=Az_t+Bu_t+C$$
2. Initial state in the plan for the current horizon must match current vehicle state.
   $$z_{0}=z_{\text{curr}}$$
3. Inputs generated must be within vehicle limits.
   $$a_{\text{min}} \leq a \leq a_{\text{max}}$$
   $$\delta_{\text{min}} \leq \delta \leq \delta_{\text{max}}$$

## III. Linearization and Discretization

In order to formulate the problem into a Quadratic Programming (QP), we need to first discretize the dynamical system, and also linearize it around some point.

### a. Discretization

Here we'll use Forward Euler discretization since it's the easiest. Other methods like RK4/6 should also work. We discretize with sampling time $dt$, which you can pick as a parameter to tune. Thus, we can express the system equation as:

$$z_{t+1} = z_t + f(z_t, u_t)dt$$

### b. Linearization
We'll use first order Taylor expansion of the two variable function around some $\bar{z}$ and $\bar{u}$:

$$
z_{t+1}=z_t + f(z_t, u_t)dt
$$

$$
z_{t+1}=z_t + (f(\bar{z_t}, \bar{u_t}) + f'_z(\bar{z_t}, \bar{u_t})(z_t - \bar{z_t}) + f'_u(\bar{z_t}, \bar{u_t})(u_t - \bar{u_t}))dt
$$

$$
z_{t+1}=z_t + (f(\bar{z_t}, \bar{u_t}) + A'(z_t - \bar{z_t}) + B'(u_t - \bar{u_t}))dt
$$

$$
z_{t+1}=z_t + (f(\bar{z_t}, \bar{u_t}) + A'z_t - A'\bar{z_t} + B'u_t - B'\bar{u_t})dt
$$

$$
z_{t+1}=(I+dtA')z_t + dtB'u_t + (f(\bar{z_t}, \bar{u_t})- A'\bar{z_t} - B'\bar{u_t})dt
$$

$$
z_{t+1} = Az_t + Bu_t + C
$$

You can then derive what are matrices $A$, $B$, and $C$.

## IV. Reference Trajectory

You'll need to create a reference trajectory that has velocity attached to each waypoint that you create. (You can follow instructions from the Pure Pursuit lab to create waypoints). For a smooth velocity profile, you can use the curvature information on the waypoint spline you've created to interpolate between a maximum and a minimum velocity. Make sure the reference has the same states as the vehicle model you've set up in the optimization problem.

## V. Logging Waypoints

There are several methods you can use to create waypoints for a specific map.

1. Recording a trajectory of joystick driven path. You can write a node that subscribe to the pose provided by the particle filter localization, and save the waypoints to a csv file. A similar script is provided [here](https://github.com/f1tenth/f1tenth_labs/blob/main/waypoint_logger/scripts/waypoint_logger.py). Note that this script is in ROS 1 and you'll have to write a ROS 2 node.

2. Find key points in the map (e.g. in the Levine loop, the four corner centers of the loop) and create a interpolated spline that goes through all four corners. You can use functions such as `scipy.interpolate.splprep` and `scipy.interpolate.splev`. You can find more documentaion on these [here](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splprep.html) and [here](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splev.html#scipy.interpolate.splev).

Usually, you'll just save the waypoints as `.csv` files with columns such as `[x, y, theta, velocity, arc_length, curvature]`. With pure pursuit, the bare minimum is `[x, y]` positions of the waypoints. Another trick is that you can also smooth the waypoints if you decided to record it with the car. You can subsample the points you gathered and re-interpolate them with the `scipy` functions mentioned above to find better waypoints.

## VI. Visualizing Waypoints

To visualize the list of waypoints you have, and to visualize the current waypoint you're picking, you'll need to use the `visualization_msgs` messages and RViz. You can find some information [here](http://wiki.ros.org/rviz/DisplayTypes/Marker).

## VII. Deliverables

- **Deliverable 1**: Submit the map files (levine_2nd.pgm and levine_2nd.yaml) that you've made using `slam_toolbox`.
- **Deliverable 2**: Commit your pure pursuit package to GitHub. Your commited code should run smoothly in simulation.
- **Deliverable 3**: Submit a link to a video on YouTube showing the real car following waypoints in Levine hallway. Show a screen recording of rviz. 

## VIII: Grading Rubric
- Compilation: **10** Points
- Running slam_toolbox and producing a map: **30** Points
- Running particle_filter: **20** Points
- Implementing pure pursuit: **30** Points
- Video: **10** Points
