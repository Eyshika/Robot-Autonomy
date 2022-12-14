import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from utils import plot_line_segments

class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=0.05)->None:
        self.statespace_lo = np.array(statespace_lo)         # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = np.array(statespace_hi)         # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are condidate for future expension
        self.est_cost_through = {} #np.inf*np.ones((self.statespace_hi - self.statespace_lo)//self.resolution + 1)  # 2d map of the estimated cost from start to goal passing through state (often called f score)
        self.cost_to_arrive = {} # np.inf*np.ones((self.statespace_hi - self.statespace_lo)//self.resolution + 1)    # 2d map of the cost-to-arrive at state from start (often called g score)
        self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)

        self.path = None        # the final path as a list of states

    def is_free(self, x):
        """
        Checks if a give state x is free, meaning it is inside the bounds of the map and
        is not inside any obstacle.
        Inputs:
            x: state tuple
        Output:
            Boolean True/False
        Hint: self.occupancy is a DetOccupancyGrid2D object, take a look at its methods for what might be
              useful here
        """
        ########## Code starts here ##########
        for pts in x:
            # check if x is within bounds of map (statespace_lo and statespace_hi)
            inMap = True
            for dim in range(len(x)):
                if x[dim] < self.statespace_lo[dim] or x[dim] > self.statespace_hi[dim]:
                    inMap = False  
                    # print("inMap =", inMap)
                    break   # will return False since pt is not in map    
                # check if X is not inside obstacles
                freeObs = self.occupancy.is_free(x) 
                # print("dim = ", dim)

            # check if pt is in map AND free of obs; otherwise, return False
            if inMap and freeObs:
                # print("inMap =", inMap)
                return True
        
        # print("inMap =", inMap)
        return False


        ########## Code ends here ##########

    def distance(self, x1, x2):
        """
        Computes the Euclidean distance between two states.
        Inputs:
            x1: First state tuple
            x2: Second state tuple
        Output:
            Float Euclidean distance

        HINT: This should take one line. Tuples can be converted to numpy arrays using np.array().
        """
        ########## Code starts here ##########
        distance = np.linalg.norm(np.array(x1) - np.array(x2))
        return distance
        ########## Code ends here ##########

    def snap_to_grid(self, x):
        """ Returns the closest point on a discrete state grid
        Input:
            x: tuple state
        Output:
            A tuple that represents the closest point to x on the discrete state grid
        """
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))

    def get_neighbors(self, x):
        """
        Gets the FREE neighbor states of a given state x. Assumes a motion model
        where we can move up, down, left, right, or along the diagonals by an
        amount equal to self.resolution.
        Input:
            x: tuple state
        Ouput:
            List of neighbors that are free, as a list of TUPLES

        HINTS: Use self.is_free to check whether a given state is indeed free.
               Use self.snap_to_grid (see above) to ensure that the neighbors
               you compute are actually on the discrete grid, i.e., if you were
               to compute neighbors by adding/subtracting self.resolution from x,
               numerical errors could creep in over the course of many additions
               and cause grid point equality checks to fail. To remedy this, you
               should make sure that every neighbor is snapped to the grid as it
               is computed.
        """
        neighbors = []
        ########## Code starts here ##########

        directions = np.array([[-1,0],[0,-1],[1,0],[0,1],[1,1],[-1,1],[-1,-1],[1,-1]])*self.resolution
        # directions = np.array([[-1,0],[0,-1],[1,0],[0,1]])*self.resolution
        for d in directions:
            new_cord = np.array(x)+d
            if self.is_free(new_cord):
                neighbors.append(self.snap_to_grid(tuple(new_cord)))
        # print("neighbors include:", neighbors)
        ########## Code ends here ##########
        return neighbors

    def find_best_est_cost_through(self):
        """
        Gets the state in open_set that has the lowest est_cost_through
        Output: A tuple, the state found in open_set that has the lowest est_cost_through
        """
        return min(self.open_set, key=lambda x: self.est_cost_through[x])

    def reconstruct_path(self):
        """
        Use the came_from map to reconstruct a path from the initial location to
        the goal location
        Output:
            A list of tuples, which is a list of the states that go from start to goal
        """
        print("Reconstructing path !!!!!")
        path = [self.x_goal]
        # print("path = ", path)
        current = path[-1]
        # print("current = ", current)
        while current != self.x_init:
            path.append(self.came_from[current])   
            current = path[-1]
            # print("current = ", current)
        return list(reversed(path))

    def plot_path(self, fig_num=0, show_init_label=True):
        """Plots the path found in self.path and the obstacles"""
        if not self.path:
            return

        self.occupancy.plot(fig_num)

        solution_path = np.array(self.path) * self.resolution
        plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label="A* solution path", zorder=10)
        plt.scatter([self.x_init[0]*self.resolution, self.x_goal[0]*self.resolution], [self.x_init[1]*self.resolution, self.x_goal[1]*self.resolution], color="green", s=30, zorder=10)
        if show_init_label:
            plt.annotate(r"$x_{init}$", np.array(self.x_init)*self.resolution + np.array([.2, .2]), fontsize=16)
        plt.annotate(r"$x_{goal}$", np.array(self.x_goal)*self.resolution + np.array([.2, .2]), fontsize=16)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

        plt.axis([0, self.occupancy.width, 0, self.occupancy.height])

    def plot_tree(self, point_size=15): #NOTE: had to cast [x] to string to get dictionary to work
        plot_line_segments([(x, self.came_from[x]) for x in self.open_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
        plot_line_segments([(x, self.came_from[x]) for x in self.closed_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
        px = [x[0] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
        py = [x[1] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
        plt.scatter(px, py, color="blue", s=point_size, zorder=10, alpha=0.2)

        # plot_line_segments([(x, self.came_from[str(x)]) for x in self.closed_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
        # plot_line_segments([(x, self.came_from[str(x)]) for x in self.open_set if x != self.x_init], linewidth=1, linestyle='dashed', color="red", alpha=0.2)
        # px = [x[0] for x in self.closed_set if x != self.x_init and x != self.x_goal]
        # py = [x[1] for x in self.closed_set if x != self.x_init and x != self.x_goal]
        # plt.scatter(px, py, color="blue", s=point_size, zorder=10, alpha=0.2)
        # px = [x[0] for x in self.open_set if x != self.x_init and x != self.x_goal]
        # py = [x[1] for x in self.open_set if x != self.x_init and x != self.x_goal]
        # plt.scatter(px, py, color="red", s=point_size, zorder=10, alpha=0.2)

    def solve(self):
        """
        Solves the planning problem using the A* search algorithm. It places
        the solution as a list of tuples (each representing a state) that go
        from self.x_init to self.x_goal inside the variable self.path
        Input:
            None
        Output:
            Boolean, True if a solution from x_init to x_goal was found

        HINTS:  We're representing the open and closed sets using python's built-in
                set() class. This allows easily adding and removing items using
                .add(item) and .remove(item) respectively, as well as checking for
                set membership efficiently using the syntax "if item in set".
        """
        ########## Code starts here ##########
        # while open set is > 0
        while len(self.open_set) > 0:
            # get the lowest est_cost_through in open set
            xCurr = self.find_best_est_cost_through()
            # print("xCurr = ", xCurr)

            # if xCurr = xG, then return the path (list of waypoints)
            if xCurr == self.x_goal:
                # print("self.path = ", self.path)
                self.path = self.reconstruct_path() 
                
                return True
            
            # remove xCurr from open set and add to closed set
            self.open_set.remove(xCurr)
            self.closed_set.add(xCurr)

            # get neighbors
            neighbors = self.get_neighbors(xCurr)

            # check neighbors
            for xNeigh in neighbors:
                if xNeigh in self.closed_set:
                    continue
                # print("xNeigh datatype = ", type(xNeigh))
                
                # calculate tentative cost to arrive at xNeigh
                tentativeCostArrive = self.cost_to_arrive[xCurr] + self.distance(xCurr, xNeigh)

                # print("self cost to arrive xCurr = ", self.cost_to_arrive[xCurr])
                # print("self cost to arrive xNeigh = ", self.cost_to_arrive[xNeigh])

                # add xNeigh to open set (to be explored)
                if xNeigh not in self.open_set:
                    self.open_set.add(xNeigh)
                elif tentativeCostArrive > self.cost_to_arrive[xNeigh]:     # default/ initial cost to xNeigh is infinity
                    continue
                
                # set "parent" for xNeigh
                # print("came_from dict = ", self.came_from)
                self.came_from[xNeigh]= xCurr        # TODO: check if this is correct in adding to dictionary
                # print("came_from dict = ", self.came_from)

                # set cost to arrive to xNeigh
                self.cost_to_arrive[xNeigh] = tentativeCostArrive

                # set cost to goal ("total cost")
                self.est_cost_through[xNeigh] = tentativeCostArrive + self.distance(xNeigh, self.x_goal)
        
        return False   

        ########## Code ends here ##########

class DetOccupancyGrid2D(object):
    """
    A 2D state space grid with a set of rectangular obstacles. The grid is
    fully deterministic
    """
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_free(self, x):
        """Verifies that point is not inside any obstacles"""
        for obs in self.obstacles:
            inside = True
            for dim in range(len(x)):
                if x[dim] < obs[0][dim] or x[dim] > obs[1][dim]:
                    inside = False
                    break
            if inside:
                return False
        return True

    def plot(self, fig_num=0):
        """Plots the space and its obstacles"""
        fig = plt.figure(fig_num)
        ax = fig.add_subplot(111, aspect='equal')
        for obs in self.obstacles:
            ax.add_patch(
            patches.Rectangle(
            obs[0],
            obs[1][0]-obs[0][0],
            obs[1][1]-obs[0][1],))
        ax.set(xlim=(0,self.width), ylim=(0,self.height))