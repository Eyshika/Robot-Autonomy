#!/usr/bin/env python3

import rospy
from asl_turtlebot.msg import DetectedObject
import tf
import numpy as np 

class DetectedPositions: 
    def __init__(self, verbose = False): 
        rospy.init_node('detected_positions', anonymous=True)
        rospy.Subscriber('/detector/detector.py', DetectedObject, self.detector_callback, queue_size=1)
        self.trans_listener = tf.TransformListener()
        self.detect_obj = {} # coordinates (keys) to object names (values)
        self.tol = 1
    def run(self): 
        rate = rospy.Rate(10)
        while not rospy.is_shutdown(): 
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
                pass
            rate.sleep()

    def detector_callback(self, data):

        orig_heading = np.array([np.cos(self.theta), np.sin(self.theta)])
        detect_theta = data.thetaleft
        R_mat = np.array([[np.cos(detect_theta), -np.sin(detect_theta)], 
                           [np.sin(detect_theta), np.cos(detect_theta)]])
        t = np.array([self.x, self.y])
        low_row = np.array([0, 0, 0, 1])
        orig_heading = np.append(orig_heading, 1)
        up_row = np.hstack([R_mat, t])
        rot_and_trans_mat = np.vstack([up_row, low_row])
        world_coord = rot_and_trans_mat @ orig_heading
        obj_loc = world_coord[:-1]
        if self.check_existing(obj_loc): 
            self.detect_obj[obj_loc] = data.name
        rospy.loginfo("Detected object")
        rospy.loginfo("at " + str(obj_loc))
        rospy.loginfo("of type " + str(data.name))

    def check_existing(self, obj_loc):
        for k in self.detect_obj.keys(): 
            if np.linalg.norm(obj_loc - k, ord=2) < self.tol: return False 
        return True

        

if __name__ == "__main__":
    DP = DetectedPositions()
    DP.run()
