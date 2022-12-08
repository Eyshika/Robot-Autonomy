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
    ########## Code starts here ##########

    # fix the path into a 2d array (rather than 1d array of tuples)
    path = np.array(path)

    # start with t=0
    t_nominal = [0]

    # find the rest of the times
    for i in range(1,np.shape(path)[0]):
        # find the distance between points in the path
        dist = np.sqrt((path[i,0]-path[i-1,0])**2 + (path[i,1]-path[i-1,1])**2)

        # find the nominal times to get to each point using V_des
        t_nominal.append(t_nominal[i-1] + dist/V_des)

    # find the smoothed times using the final nominal time and the given dt
    t_smoothed = np.arange(0, t_nominal[-1], dt)

    # find the B-spline representation of the x
    # use the k and alpha provided
    x_tck = scipy.interpolate.splrep(t_nominal, path[:,0], k=k, s=alpha)

    # evaluate the spline for x, xd, and xdd
    x_d = scipy.interpolate.splev(t_smoothed, x_tck, der=0)
    xd_d = scipy.interpolate.splev(t_smoothed, x_tck, der=1)
    xdd_d = scipy.interpolate.splev(t_smoothed, x_tck, der=2)

    # find the B-spline representation of the y
    # use the k and alpha provided
    y_tck = scipy.interpolate.splrep(t_nominal, path[:,1], k=k, s=alpha)

    # evaluate the spline for x, xd, and xdd
    y_d = scipy.interpolate.splev(t_smoothed, y_tck, der=0)
    yd_d = scipy.interpolate.splev(t_smoothed, y_tck, der=1)
    ydd_d = scipy.interpolate.splev(t_smoothed, y_tck, der=2)

    # theta d is how we calculate from hw1, arctan2 
    theta_d = np.arctan2(yd_d, xd_d)

    ########## Code ends here ##########
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    return t_smoothed, traj_smoothed
