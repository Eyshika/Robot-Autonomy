#!/usr/bin/env python3

import rospy
from robot_autonomy.msg import DetectedObject, DetectedObjectList
from std_msgs.msg import String
import tf
import numpy as np 
from robot_autonomy.msg import LocObject, LocObjectList

class DetectedPositions: 
    def __init__(self, verbose = False): 
        rospy.init_node('detected_positions', anonymous=True)
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.detector_callback, queue_size=1)
        self.pub = rospy.Publisher('/detector/detected_objs', String, queue_size=10)
        self.pub_clean = rospy.Publisher('/detector/detected_objs_clean', LocObjectList, queue_size=10)
        self.pub_list = LocObjectList()
        self.trans_listener = tf.TransformListener()
        self.detect_obj = {} # coordinates (keys) to object names (values)
        self.tol = .2
        self.pub.publish("INIT!")

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
                pass
            rate.sleep()
            for i in self.detect_obj.items(): 
                self.pub.publish(str(i))
            self.pub_list = LocObjectList()
            for i in self.detect_obj.keys(): 
                obj = LocObject()
                obj.x, obj.y = i
                obj.name = self.detect_obj[i][2]
                self.pub_list.objs.append(obj) 
            self.pub_clean.publish(self.pub_list)
                


    def detector_callback(self, data):
        for d in data.ob_msgs: 
            if d.confidence < 0.8: 
                continue

            self.ascii_art(d.name)
            orig_heading = d.distance * np.array([np.cos(self.theta), np.sin(self.theta)])

            # calculate thetamid
            if (d.thetaleft < d.thetaright):
                d.thetaleft += 2*np.pi
            thetamid = (d.thetaleft-d.thetaright)/2 + d.thetaright
            if (thetamid > 2*np.pi):
                thetamid -= 2*np.pi
            #print("**** tl:",d.thetaleft,"tr:",d.thetaright,"tm:",thetamid)

            detect_theta = thetamid
            R_mat = np.array([[np.cos(detect_theta), -np.sin(detect_theta)], 
                            [np.sin(detect_theta), np.cos(detect_theta)]])
            t = np.array([self.x, self.y])

            low_row = np.array([0, 0, 1])
            orig_heading = np.append(orig_heading, 1)
            up_row = np.hstack([R_mat, t.reshape((2, 1))])
            rot_and_trans_mat = np.vstack([up_row, low_row])
            world_coord = rot_and_trans_mat @ (orig_heading)
            world_coord = world_coord[:-1]
            obj_loc = tuple(world_coord)
            if obj_loc_exist := (self.check_existing(obj_loc)):
                obj_names, count, _ = self.detect_obj[obj_loc_exist]
                if d.name not in obj_names: 
                    obj_names[d.name] = 0
                obj_names[d.name] += 1
                new_loc = (np.array(obj_loc_exist) * count + np.array(obj_loc)) / (count + 1)
                new_loc = tuple(new_loc)
                count += 1
                self.detect_obj.pop(obj_loc_exist)
                best_name_count = max(obj_names.values())
                for i in obj_names.keys(): 
                    if obj_names[i] == best_name_count:
                        best_name = i
                        break 
                self.detect_obj[new_loc] = (obj_names, count, best_name)
                
            else:
                self.detect_obj[obj_loc] = ({d.name: 1}, 1, d.name)
                # rospy.loginfo("Detected Objects: %s"%self.detect_obj)


    def check_existing(self, obj_loc):
        for k in self.detect_obj.keys(): 
            if np.linalg.norm(np.array(obj_loc) - np.array(k), ord=2) < self.tol: 
                return k
        return None

    def ascii_art(self, name): 
        if name == "cat":
            catArt = """
                    |\__/,|   (\\
                    |_ _  |.--.) )   MEOW!
                    ( T   )     /
                    (((^_(((/(((_/
                    """
            rospy.loginfo(catArt)
        elif name == "dog":
            dogArt = """
                    	 __
                    (___()'`;  WOOF!
                    /,    /`
                    \\"--\\
                    
                    """
            rospy.loginfo(dogArt)
        

if __name__ == "__main__":
    DP = DetectedPositions()
    DP.run()
