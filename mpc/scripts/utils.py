# MIT License

# Copyright (c) Hongrui Zheng, Johannes Betz

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Utility functions for Kinematic Single Track MPC waypoint tracker

Author: Hongrui Zheng, Johannes Betz, Ahmad Amine
Last Modified: 12/27/22
"""
import math
import numpy as np
from numba import njit

@njit(cache=True)
def nearest_point(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.
    Args:
        point (numpy.ndarray, (2, )): (x, y) of current pose
        trajectory (numpy.ndarray, (N, 2)): array of (x, y) trajectory waypoints
            NOTE: points in trajectory must be unique. If they are not unique, a divide by 0 error will destroy the world
    Returns:
        nearest_point (numpy.ndarray, (2, )): nearest point on the trajectory to the point
        nearest_dist (float): distance to the nearest point
        t (float): nearest point's location as a segment between 0 and 1 on the vector formed by the closest two points on the trajectory. (p_i---*-------p_i+1)
        i (int): index of nearest point in the array of trajectory waypoints
    """
    diffs = trajectory[1:,:] - trajectory[:-1,:]
    l2s   = diffs[:,0]**2 + diffs[:,1]**2
    dots = np.empty((trajectory.shape[0]-1, ))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t<0.0] = 0.0
    t[t>1.0] = 1.0
    projections = trajectory[:-1,:] + (t*diffs.T).T
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp*temp))
    min_dist_segment = np.argmin(dists)
    return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment


@njit(cache=True)
def calc_interpolated_ref_trajectory(x, y, cx, cy, cv, cyaw, dt, N):
    """
    Calculate interpolated reference trajectory using proper linear interpolation.

    This function finds the nearest point on the trajectory, then propagates forward
    using interpolated velocities to find reference points for the MPC horizon.
    All state variables are linearly interpolated between waypoints.

    Args:
        x (float): Current x position
        y (float): Current y position
        cx (numpy.ndarray): x positions of waypoints
        cy (numpy.ndarray): y positions of waypoints
        cv (numpy.ndarray): velocities at waypoints
        cyaw (numpy.ndarray): yaw angles at waypoints
        dt (float): Time step
        N (int): Prediction horizon (returns N+1 points)

    Returns:
        ref_traj (numpy.ndarray): Interpolated reference trajectory (4, N+1)
                                  where rows are [x, y, v, yaw]
    """
    ncourse = len(cx)

    # Calculate distance between waypoints (assume uniform spacing)
    dl = np.sqrt((cx[1] - cx[0])**2 + (cy[1] - cy[0])**2)

    # Find nearest point and interpolation parameter t ∈ [0, 1]
    point = np.array([x, y])
    trajectory = np.column_stack((cx, cy))
    _, _, t_current, ind_current = nearest_point(point, trajectory)

    # Build t_list: accumulated "distance" in units of segments
    # Start from t_current at ind_current, propagate forward using interpolated velocity
    t_list = np.zeros(N + 1)
    t_list[0] = t_current

    # Get initial interpolated speed
    ind_next = (ind_current + 1) % ncourse
    current_speed = (1.0 - t_current) * cv[ind_current] + t_current * cv[ind_next]

    for i in range(1, N + 1):
        # Distance traveled in dt at current_speed, converted to segment units
        t_list[i] = t_list[i - 1] + (current_speed * dt) / dl

        # Update speed using new interpolation parameter
        # (Note: this uses fractional part for interpolation within segment)
        t_frac = t_list[i] % 1.0
        seg_idx = (int(t_list[i]) + ind_current) % ncourse
        seg_next = (seg_idx + 1) % ncourse
        current_speed = (1.0 - t_frac) * cv[seg_idx] + t_frac * cv[seg_next]

    # Convert t_list to segment indices and fractional parts
    ind_list = (np.floor(t_list).astype(np.int64) + ind_current) % ncourse
    t_frac_list = t_list % 1.0

    # Interpolate all state variables
    ref_traj = np.zeros((4, N + 1))

    for i in range(N + 1):
        idx = ind_list[i]
        idx_next = (idx + 1) % ncourse
        t_frac = t_frac_list[i]

        ref_traj[0, i] = (1.0 - t_frac) * cx[idx] + t_frac * cx[idx_next]  # x
        ref_traj[1, i] = (1.0 - t_frac) * cy[idx] + t_frac * cy[idx_next]  # y
        ref_traj[2, i] = (1.0 - t_frac) * cv[idx] + t_frac * cv[idx_next]  # v

        # Handle yaw interpolation (unwrap to avoid discontinuities)
        yaw0 = cyaw[idx]
        yaw1 = cyaw[idx_next]
        # Wrap difference to [-pi, pi] using atan2
        yaw_diff = np.arctan2(np.sin(yaw1 - yaw0), np.cos(yaw1 - yaw0))
        ref_traj[3, i] = yaw0 + t_frac * yaw_diff  # yaw

    return ref_traj
