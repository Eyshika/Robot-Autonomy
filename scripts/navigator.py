#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from std_msgs.msg import String
import tf
import numpy as np
from numpy import linalg
from utils.utils import wrapToPi
from utils.grids import StochOccupancyGrid2D
from planners import AStar, compute_smoothed_traj
import scipy.interpolate
import matplotlib.pyplot as plt
from controllers import PoseController, TrajectoryTracker, HeadingController
from enum import Enum

from dynamic_reconfigure.server import Server
from robot_autonomy.cfg import NavigatorConfig

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 0
    ALIGN = 1
    TRACK = 2
    PARK = 3


class Navigator:
    """
    This node handles point to point turtlebot motion, avoiding obstacles.
    It is the sole node that should publish to cmd_vel
    """

    def __init__(self):
        rospy.init_node("turtlebot_navigator", anonymous=True)
        self.mode = Mode.IDLE

        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # goal state
        self.x_g = None
        self.y_g = None
        self.theta_g = None

        self.th_init = 0.0

        # map parameters
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0, 0]
        self.map_probs = []
        self.occupancy = None
        self.occupancy_updated = False

        # plan parameters
        self.plan_resolution = 0.05
        self.plan_horizon = 15

        # time when we started following the plan
        self.current_plan_start_time = rospy.get_rostime()
        self.current_plan_duration = 0
        self.last_successful_replan_time = rospy.get_rostime()
        self.replan_time = 5
        self.plan_start = [0.0, 0.0]

        # Robot limits
        self.v_max = .15  # maximum velocity
        self.om_max = 0.4 # maximum angular velocity

        self.v_des = 0.1  # desired cruising velocity
        self.theta_start_thresh = 0.04  # threshold in theta to start moving forward when path-following
        self.start_pos_thresh = (
            0.2  # threshold to be far enough into the plan to recompute it
        )

        # threshold at which navigator switches from trajectory to pose control
        self.near_thresh = 0.2
        self.at_thresh = 0.02
        self.at_thresh_theta = 0.6 # THIS NEEDS TO BE A LOT BETTER!

        # # threshold at which we can replan our path
        self.replan_length_thresh = 4

        # trajectory smoothing
        self.spline_alpha = 0.07
        self.spline_deg = 3  # cubic spline
        self.traj_dt = 0.1

        # trajectory tracking controller parameters
        self.kpx = 0.5
        self.kpy = 0.5
        self.kdx = 1.5
        self.kdy = 1.5

        # start in explore mode
        self.rescue_mode = False

        # heading controller parameters
        self.kp_th = 2.0

        self.traj_controller = TrajectoryTracker(
            self.kpx, self.kpy, self.kdx, self.kdy, self.v_max, self.om_max
        )
        self.pose_controller = PoseController(
            0.0, 0.0, 0.0, self.v_max, self.om_max
        )
        self.heading_controller = HeadingController(self.kp_th, self.om_max)

        self.nav_planned_path_pub = rospy.Publisher(
            "/planned_path", Path, queue_size=10
        )
        self.nav_smoothed_path_pub = rospy.Publisher(
            "/cmd_smoothed_path", Path, queue_size=10
        )
        self.nav_smoothed_path_rej_pub = rospy.Publisher(
            "/cmd_smoothed_path_rejected", Path, queue_size=10
        )
        self.nav_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.trans_listener = tf.TransformListener()

        self.cfg_srv = Server(NavigatorConfig, self.dyn_cfg_callback)

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/map_metadata", MapMetaData, self.map_md_callback)
        rospy.Subscriber("/cmd_nav", Pose2D, self.cmd_nav_callback)
        rospy.Subscriber("/rescue_nav", Pose2D, self.rescue_nav_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback) #for checking position ;live
        self.set_next = False
        self.start_time = rospy.get_rostime()
        self.end_time = rospy.get_rostime()

        rospy.loginfo("Navigator: finished init")

    def dyn_cfg_callback(self, config, level):
        rospy.loginfo(
            "Reconfigure Request: k1:{k1}, k2:{k2}, k3:{k3}".format(**config)
        )
        self.pose_controller.k1 = config["k1"]
        self.pose_controller.k2 = config["k2"]
        self.pose_controller.k3 = config["k3"]
        return config


    def rescue_nav_callback(self, data):
        self.rescue_mode = True
        if (data.x != self.x_g or data.y != self.y_g or data.theta != self.theta_g):
            rospy.loginfo("!!!! GOT NEW RESCUE CMD_NAV POSITIONS !!!!")
            rospy.loginfo("New goal: x:%.2f y:%.2f th:%.2f",data.x,data.y,data.theta)
            self.x_g = data.x
            self.y_g = data.y
            self.theta_g = data.theta

            if not self.replan():
                rospy.loginfo("!!!! RESCUE CMD_NAV POSITION CANNOT BE PLANNED !!!!")
                self.switch_mode(Mode.IDLE)

            # # keep replanning until we succeed, failure is not an option
            # offset_pos = 0.25
            # offset_th = 1
            # while not self.replan():
            #     self.x_g = data.x + np.random.rand() * offset_pos - offset_pos/2
            #     self.y_g = data.y + np.random.rand() * offset_pos - offset_pos/2
            #     self.theta_g = data.theta + np.random.rand() * offset_th + offset_th/2
            #     rospy.loginfo("New goal: x:%.2f y:%.2f th:%.2f",self.x_g,self.y_g,self.theta_g)



    def cmd_nav_callback(self, data):
        """
        loads in goal if different from current goal, and replans
        """
        if (data.x != self.x_g or data.y != self.y_g or data.theta != self.theta_g):
            rospy.loginfo("!!!! GOT NEW CMD_NAV POSITIONS !!!!")
            rospy.loginfo("New goal: x:%.2f y:%.2f th:%.2f",data.x,data.y,data.theta)
            self.x_g = data.x
            self.y_g = data.y
            self.theta_g = data.theta
            if not self.replan():
                rospy.loginfo("!!!! CMD_NAV POSITION CANNOT BE PLANNED !!!!")
                self.switch_mode(Mode.IDLE)


    def map_md_callback(self, msg):
        """
        receives maps meta data and stores it
        """
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x, msg.origin.position.y)

    def map_callback(self, msg):
        """
        receives new map info and updates the map
        """
        self.map_probs = msg.data
        # if we've received the map metadata and have a way to update it:
        if (
            self.map_width > 0
            and self.map_height > 0
            and len(self.map_probs) > 0
        ):
            self.occupancy = StochOccupancyGrid2D(
                self.map_resolution,
                self.map_width,
                self.map_height,
                self.map_origin[0],
                self.map_origin[1],
                9,
                self.map_probs,
            )

            # we don't need to replan - if the robot has a goal it's already
            # constantly replanning, this just slows down the main thread
            # if self.x_g is None or self.mode == Mode.IDLE:
            #     # rospy.loginfo("New map info- ignoring, no goal")
            #     pass
            # elif self.near_goal():
            #     rospy.loginfo("New map info- ignoring, near goal")
            # else:                
            #     rospy.loginfo("New map info- replanning")
            #     self.replan()  # new map, need to replan

    def shutdown_callback(self):
        """
        publishes zero velocities upon rospy shutdown
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.nav_vel_pub.publish(cmd_vel)

    def near_goal(self):
        """
        returns whether the robot is close enough in position to the goal to
        start using the pose controller
        """
        return (
            linalg.norm(np.array([self.x - self.x_g, self.y - self.y_g]))
            < self.near_thresh
        )

    def at_goal(self):
        """
        returns whether the robot has reached the goal position with enough
        accuracy to return to idle state
        """
        # dist_diff = linalg.norm(np.array([self.x - self.x_g, self.y - self.y_g]))
        # theta_diff = abs(wrapToPi(self.theta - self.theta_g))
        # rospy.loginfo("DIFF- dist: %.3f/%.2f theta: %.3f/%.2f",dist_diff,self.at_thresh,theta_diff,self.at_thresh_theta)
        return (linalg.norm(np.array([self.x-self.x_g, self.y-self.y_g])) < self.near_thresh and abs(wrapToPi(self.theta - self.theta_g)) < self.at_thresh_theta)

    def cmd_vel_callback(self, data):
        # linear = data.linear 
        # lin_x, lin_y, lin_z = linear.x, linear.y, linear.z
        # angular = data.angular
        # ang_x, ang_y, ang_z = angular.x, angular.y, angular.z 
        if abs(data.linear.x) < 0.00000001 and abs(data.linear.z) < 0.00000001: 
            self.end_time = rospy.get_rostime()
        else: 
            self.start_time = rospy.get_rostime()

        if (self.end_time-self.start_time)>rospy.Duration.from_sec(3): 
            self.set_next = True
            # rospy.loginfo("ROBOT STOPPED")
        else: 
            self.set_next = False
            #self.end_time = rospy.get_rostime()

    def aligned(self):
        """
        returns whether robot is aligned with starting direction of path
        (enough to switch to tracking controller)
        """
        return (
            abs(wrapToPi(self.theta - self.th_init)) < self.theta_start_thresh
        )

    def close_to_plan_start(self):
        return (
            abs(self.x - self.plan_start[0]) < self.start_pos_thresh
            and abs(self.y - self.plan_start[1]) < self.start_pos_thresh
        )

    def snap_to_grid(self, x):
        return (
            self.plan_resolution * round(x[0] / self.plan_resolution),
            self.plan_resolution * round(x[1] / self.plan_resolution),
        )

    def switch_mode(self, new_mode):
        rospy.loginfo("Switching from %s -> %s", self.mode, new_mode)
        self.mode = new_mode

        # adjust the mode switching variables
        self.set_next = False
        if (new_mode == Mode.IDLE):
            self.x_g = None
            self.y_g = None
            self.theta_g = None


    def publish_planned_path(self, path, publisher):
        # publish planned plan for visualization
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for state in path:
            pose_st = PoseStamped()
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            pose_st.pose.orientation.w = 1
            pose_st.header.frame_id = "map"
            path_msg.poses.append(pose_st)
        publisher.publish(path_msg)

    def publish_smoothed_path(self, traj, publisher):
        # publish planned plan for visualization
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for i in range(traj.shape[0]):
            pose_st = PoseStamped()
            pose_st.pose.position.x = traj[i, 0]
            pose_st.pose.position.y = traj[i, 1]
            pose_st.pose.orientation.w = 1
            pose_st.header.frame_id = "map"
            path_msg.poses.append(pose_st)
        publisher.publish(path_msg)

    def publish_control(self):
        """
        Runs appropriate controller depending on the mode. Assumes all controllers
        are all properly set up / with the correct goals loaded
        """
        t = self.get_current_plan_time()

        if self.mode == Mode.PARK:
            V, om = self.pose_controller.compute_control(
                self.x, self.y, self.theta, t
            )
        elif self.mode == Mode.TRACK:
            V, om = self.traj_controller.compute_control(
                self.x, self.y, self.theta, t
            )
        elif self.mode == Mode.ALIGN:
            V, om = self.heading_controller.compute_control(
                self.x, self.y, self.theta, t
            )
        else:
            V = 0.0
            om = 0.0

        # should be unnecessary, but limit to +/- v_max and om_max
        if V > self.v_max: V = self.v_max
        if V < -self.v_max: V = -self.v_max
        if om > self.om_max: om = self.om_max
        if om < -self.om_max: om = -self.om_max

        cmd_vel = Twist()
        cmd_vel.linear.x = V
        cmd_vel.angular.z = om
        self.nav_vel_pub.publish(cmd_vel)

    def get_current_plan_time(self):
        t = (rospy.get_rostime() - self.current_plan_start_time).to_sec()
        return max(0.0, t)  # clip negative time to 0

    def replan(self):
        """
        loads goal into pose controller
        runs planner based on current pose
        if plan long enough to track:
            smooths resulting traj, loads it into traj_controller
            sets self.current_plan_start_time
            sets mode to ALIGN
        else:
            sets mode to PARK
        """
        # Make sure we have a map
        if not self.occupancy:
            rospy.loginfo("Navigator: replan canceled, waiting for occupancy map")
            self.switch_mode(Mode.IDLE)
            return False

        # Attempt to plan a path
        state_min = self.snap_to_grid((-self.plan_horizon, -self.plan_horizon))
        state_max = self.snap_to_grid((self.plan_horizon, self.plan_horizon))
        x_init = self.snap_to_grid((self.x, self.y))
        self.plan_start = x_init
        x_goal = self.snap_to_grid((self.x_g, self.y_g))
        problem = AStar(
            state_min,
            state_max,
            x_init,
            x_goal,
            self.occupancy,
            self.plan_resolution,
        )

        if self.mode != Mode.TRACK:
            rospy.loginfo("Navigator: computing nav plan")
        else:
            rospy.loginfo("Navigator: re-evaluating plan")

        success = problem.solve()
        if not success:
            rospy.loginfo("Navigator: nav planning failed!")
            return False
        planned_path = problem.path

        # Check whether path is too short
        if len(planned_path) <= self.replan_length_thresh:
            rospy.loginfo("Navigator: new path too short, PARKING")
            self.pose_controller.load_goal(self.x_g, self.y_g, self.theta_g)
            self.switch_mode(Mode.PARK)
            self.last_successful_replan_time = rospy.get_rostime()
            return True

        # Smooth and generate a trajectory
        t_new, traj_new = compute_smoothed_traj(
            planned_path, self.v_des, self.spline_deg, self.spline_alpha, self.traj_dt
        )

        # If currently tracking a trajectory, check whether new trajectory will take more time to follow
        if self.mode == Mode.TRACK:
            t_remaining_curr = (
                self.current_plan_duration - self.get_current_plan_time()
            )

            # Estimate duration of new trajectory
            th_init_new = traj_new[0, 2]
            th_err = wrapToPi(th_init_new - self.theta)
            t_init_align = abs(th_err / self.om_max)
            t_remaining_new = t_init_align + t_new[-1]

            if t_remaining_new > t_remaining_curr:
                #rospy.loginfo("Navigator: keeping current plan")
                # self.publish_smoothed_path(
                #     traj_new, self.nav_smoothed_path_rej_pub
                # )
                return False
            else:
                rospy.loginfo("Navigator: changing plan!")


        # Otherwise follow the new plan
        self.publish_planned_path(planned_path, self.nav_planned_path_pub)
        self.publish_smoothed_path(traj_new, self.nav_smoothed_path_pub)

        self.pose_controller.load_goal(self.x_g, self.y_g, self.theta_g)
        self.traj_controller.load_traj(t_new, traj_new)

        self.current_plan_start_time = rospy.get_rostime()
        self.current_plan_duration = t_new[-1]

        self.th_init = traj_new[0, 2]
        self.heading_controller.load_goal(self.th_init)

        if not self.aligned():
            rospy.loginfo("Navigator: not aligned with start direction")
            self.switch_mode(Mode.ALIGN)
            self.last_successful_replan_time = rospy.get_rostime()
            return True

        rospy.loginfo("Navigator: tracking new plan")
        self.switch_mode(Mode.TRACK)
        self.last_successful_replan_time = rospy.get_rostime()
        return True


    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # try to get state information to update self.x, self.y, self.theta
            try:
                (translation, rotation) = self.trans_listener.lookupTransform(
                    "/map", "/base_footprint", rospy.Time(0)
                )
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ) as e:
                self.current_plan = []
                rospy.loginfo("Navigator: waiting for state info")
                self.switch_mode(Mode.IDLE)
                print(e)
                pass

            # STATE MACHINE LOGIC
            # some transitions handled by callbacks
            if self.mode == Mode.IDLE:
                pass
            elif self.mode == Mode.ALIGN:
                if self.aligned():
                    self.current_plan_start_time = rospy.get_rostime()
                    self.switch_mode(Mode.TRACK)
            elif self.mode == Mode.TRACK:
                if self.near_goal():
                    self.switch_mode(Mode.PARK)
                elif not self.rescue_mode and (rospy.get_rostime() - self.last_successful_replan_time).to_sec() > self.replan_time:
                    # self.replan()
                    pass
            elif self.mode == Mode.PARK:
                if self.at_goal():
                    rospy.loginfo("PARKED")
                    self.switch_mode(Mode.IDLE)
                elif self.set_next:
                    rospy.loginfo("PARKED (timed out)")
                    self.set_next = False
                    self.switch_mode(Mode.IDLE)

            self.publish_control()
            rate.sleep()


if __name__ == "__main__":
    nav = Navigator()
    rospy.on_shutdown(nav.shutdown_callback)
    nav.run()
