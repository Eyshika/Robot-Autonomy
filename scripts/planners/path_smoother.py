import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, k, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        k (int): The degree of the spline fit.
            For this assignment, k should equal 3 (see documentation for
            scipy.interpolate.splrep)
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        t_smoothed (np.array [N]): Associated trajectory times
        
        traj_smoothed (np.array [N,7]): Smoothed trajectory
    Hint: Use splrep and splev from scipy.interpolate
    """
    assert(path and k > 2 and k < len(path))
    ########## Code starts here ##########
    # Hint 1 - Determine nominal time for each point in the path using V_des
    # Hint 2 - Use splrep to determine cubic coefficients that best fit given path in x, y
    # Hint 3 - Use splev to determine smoothed paths. The "der" argument may be useful.
    path = np.array(path)
    nominal_time = np.zeros(len(path))
    nominal_time[0] = 0
    for i_node in range(1, len(path)): #euclidean distance
        nominal_time[i_node] = nominal_time[i_node-1] + np.linalg.norm(path[i_node-1] - path[i_node])/V_des
    
    splx = scipy.interpolate.splrep(nominal_time, path[:,0], k=3, s=alpha)
    sply = scipy.interpolate.splrep(nominal_time, path[:,1], k=3, s=alpha)
    # print(splx, sply)
    # print(dt, nominal_time)
    t_smoothed = np.arange(nominal_time[0], int(nominal_time[-1]), dt)
    # print(t_smoothed)
    x_d = scipy.interpolate.splev(t_smoothed, splx, der=0)
    y_d = scipy.interpolate.splev(t_smoothed, sply, der=0)
    
    xd_d = scipy.interpolate.splev(t_smoothed, splx, der=1)
    yd_d = scipy.interpolate.splev(t_smoothed, sply, der=1)
    theta_d = np.arctan2(yd_d, xd_d) #was struggling with this but got help from ed discussion

    xdd_d = scipy.interpolate.splev(t_smoothed, splx, der=2)
    ydd_d = scipy.interpolate.splev(t_smoothed, sply, der=2) #k=3 given
    ########## Code ends here ##########
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    return t_smoothed, traj_smoothed
