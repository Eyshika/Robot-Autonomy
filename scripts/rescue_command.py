#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from robot_autonomy.msg import LocObject, LocObjectList
import tf
import time
import numpy as np


# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = True
HOME_X = 4.25
HOME_Y = 3.25
HOME_THETA = 4.71

class RescueCommander:

    def __init__(self):
        rospy.init_node('rescue_pose_commander', anonymous=True)
        # initialize variables
        self.x_g = None
        self.y_g = None
        self.theta_g = None
        self.goal_pose_received = False
        self.trans_listener = tf.TransformListener()
        self.start_time = rospy.get_rostime()
        # command pose for controller
        # self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.nav_goal_publisher = rospy.Publisher('/rescue_nav', Pose2D, queue_size=10)
        # rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        self.cmd_nav_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.detected_position_sub = rospy.Subscriber('/detector/detected_objs_clean', LocObjectList, self.get_obj_pos)
        self.last_list = []
        self.det_objs = []
        self.rescue_flag = False
        self.new_goal = PoseStamped()
        self.set_next = True
        self.stop_count = 0

    def cmd_vel_callback(self, data): 
        # rospy.loginfo("RECEVIED NEW VEL")
        linear = data.linear 
        lin_x, lin_y, lin_z = linear.x, linear.y, linear.z
        angular = data.angular
        ang_x, ang_y, ang_z = angular.x, angular.y, angular.z 
        if abs(lin_x) < 0.01 and abs(lin_y) < 0.01 and abs(lin_z) < 0.01 \
            and abs(ang_x) < 0.01 and abs(ang_y) < 0.01 and abs(ang_z) < 0.01: 
            self.stop_count += 1
        else: 
            self.stop_count = 0

        if self.stop_count == 10:
            self.set_next = True
            self.stop_count = 0
            rospy.loginfo("ROBOT STOPPED")
  
    def run(self): 
        rate = rospy.Rate(10)
        while not rospy.is_shutdown(): 
            #self.com = input("Enter Rescue animals list: ")
            self.process_input()
            #rescuing
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
                pass
            rate.sleep()


    def wrap_pi(self, val): 
        if val > 2 * np.pi: 
            while val > 2 * np.pi: 
                val -= 2 * np.pi 
        elif val < 0: 
            while val < 0: 
                val += 2 * np.pi
        return val 
        
    def process_input(self):
        rate = rospy.Rate(10)
       
        if self.rescue_flag:
            self.com = input("Enter rescue animals [e.g. 1,3,4]: ")
            self.animal_list = self.com.split(",")
            self.animal_list = [int(i) for i in self.animal_list]
            #TODO: distance measurement for ordering
            for j in self.animal_list: 
                _, _, x_g, y_g, name = self.det_objs[j]
                x_g = round(float(x_g), 2)
                y_g = round(float(y_g), 2)
                theta_g = HOME_THETA #4.71
                
                self.publish_goal_pose(x_g, y_g, theta_g)
                self.set_next = False
                rospy.loginfo("!!! RESCUING ANIMAL %s AT %s %s"%(name, x_g, y_g))
                while not(self.set_next): 
                    # rospy.loginfo("WAITING")
                    rate.sleep()
    
                rospy.loginfo("GOING INTO SLEEP")
                rospy.sleep(5)
                rospy.loginfo("WAKE UP")

                rospy.loginfo("DONE WITH ANIMAL")
            self.go_home()

        else: 
            self.com = input("Enter command [RESCUE when ready]: ")
            if self.com.lower() == "rescue":
                self.initialize_rescue()
                rospy.loginfo("Done initializing")
            #recuing

    def initialize_rescue(self): 
        for n, i in enumerate(self.last_list): 
            self.det_objs.append((i.x, i.y, i.rescue_x, i.rescue_y, i.name))
            rospy.loginfo("Detected object %s %s" %(n, i))
        rospy.loginfo("GOING HOME !!!!")
        self.go_home()
        self.rescue_flag = True

    def get_obj_pos(self,data):
        self.last_list = data.objs.copy()


    def go_home(self): 
        # self.x_g = HOME_X
        # self.y_g = HOME_Y 
        # self.theta_g = HOME_THETA
        self.publish_goal_pose(HOME_X, HOME_Y, HOME_THETA)

    def publish_goal_pose(self, x_g, y_g, theta_g):
        """ sends the current desired pose to the navigator """
        if x_g is not None:
            pose_g_msg = Pose2D()
            pose_g_msg.x = x_g
            pose_g_msg.y = y_g 
            pose_g_msg.theta = theta_g
            rospy.loginfo("nav_goal publisher!!!!!!!!")
            self.nav_goal_publisher.publish(pose_g_msg)

if __name__ == "__main__": 
    RC = RescueCommander()
    RC.run()