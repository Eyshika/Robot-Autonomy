theta = 3 
import numpy as np
x = 4
y = 10

orig_heading = np.array([np.cos(theta), np.sin(theta)])
detect_theta = 3.14
R_mat = np.array([[np.cos(detect_theta), -np.sin(detect_theta)], 
                [np.sin(detect_theta), np.cos(detect_theta)]])
t = np.array([x, y])
low_row = np.array([0, 0, 1])
orig_heading = np.append(orig_heading, 1)
up_row = np.hstack([R_mat, t.reshape((2, 1))])
print(up_row)
rot_and_trans_mat = np.vstack([up_row, low_row])
print(rot_and_trans_mat)
world_coord = rot_and_trans_mat @ orig_heading
