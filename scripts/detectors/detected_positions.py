#!/usr/bin/env python3

import rospy
from robot_autonomy.msg import DetectedObject, DetectedObjectList
from std_msgs.msg import String
import tf
import numpy as np 
from robot_autonomy.msg import LocObject, LocObjectList

# this is the offset applied to the position of detected objects, in
# the direction of the viewer, applied to object_rescue_position
TARGET_OFFSET = .5

# this is the tolerance on the detected object position, used for
# checking if detected objects have already been detected
DETECTED_OBJECT_POSITION_TOLERANCE = 0.2


class DetectedPositions: 
    def __init__(self, verbose = False): 
        rospy.init_node('detected_positions', anonymous=True)
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.detector_callback, queue_size=1)
        self.pub = rospy.Publisher('/detector/detected_objs', String, queue_size=10)
        self.pub_clean = rospy.Publisher('/detector/detected_objs_clean', LocObjectList, queue_size=10)
        self.pub_list = LocObjectList()
        self.trans_listener = tf.TransformListener()
        self.detect_obj = {} # coordinates (keys) to object names (values)
        self.tol = DETECTED_OBJECT_POSITION_TOLERANCE
        self.pub.publish("INIT!")
        self.last_goal_done = True 
    

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

            # build the object list of detected objects
            self.pub_list = LocObjectList()
            for i in self.detect_obj.keys(): 
                obj = LocObject()
                obj.x, obj.y = str(i[0]), str(i[1])
                obj.rescue_x, obj.rescue_y = self.detect_obj[i][0]
                obj.rescue_x = str(obj.rescue_x)
                obj.rescue_y = str(obj.rescue_y)
                obj.name = self.detect_obj[i][3]
                self.pub_list.objs.append(obj) 
                
            self.pub_clean.publish(self.pub_list)
                


    def detector_callback(self, data):
        for d in data.ob_msgs: 
            if d.confidence < 0.8: 
                continue

            # dog/cat art! +10%
            self.ascii_art(d.name)

            # get object position and object rescue position in the robot frame
            object_pos_robot_frame = d.distance * np.array([np.cos(self.theta), np.sin(self.theta)])
            dist_with_offset = max(d.distance - TARGET_OFFSET, 0)
            object_rescue_pos_robot_frame = dist_with_offset * np.array([np.cos(self.theta), np.sin(self.theta)])
            # print("obj pos:",object_pos_robot_frame,"rescue:",object_rescue_pos_robot_frame)

            # calculate thetamid from the angles to the outside of the bounding box
            if (d.thetaleft < d.thetaright):
                d.thetaleft += 2*np.pi
            thetamid = (d.thetaleft-d.thetaright)/2 + d.thetaright
            if (thetamid > 2*np.pi):
                thetamid -= 2*np.pi
            #print("**** tl:",d.thetaleft,"tr:",d.thetaright,"tm:",thetamid)
            # get rotation matrix
            detect_theta = thetamid
            R_mat = np.array([[np.cos(detect_theta), -np.sin(detect_theta)], 
                            [np.sin(detect_theta), np.cos(detect_theta)]])
            t = np.array([self.x, self.y])

            # transform object positions from robot frame to world frame
            low_row = np.array([0, 0, 1])
            object_pos_robot_frame = np.append(object_pos_robot_frame, 1)
            object_rescue_pos_robot_frame = np.append(object_rescue_pos_robot_frame, 1)
            up_row = np.hstack([R_mat, t.reshape((2, 1))])
            rot_and_trans_mat = np.vstack([up_row, low_row])
            obj_pos_world_frame = rot_and_trans_mat @ (object_pos_robot_frame)
            obj_pos_world_frame = obj_pos_world_frame[:-1]
            obj_rescue_pos_world_frame = rot_and_trans_mat @ (object_rescue_pos_robot_frame)
            obj_rescue_pos_world_frame = obj_rescue_pos_world_frame[:-1]
            
            # get transformed locations
            obj_loc = tuple(obj_pos_world_frame)
            obj_rescue_loc = tuple(obj_rescue_pos_world_frame)
            # print("obj loc:",obj_loc,"rescue:",obj_rescue_loc)

            # check if object is already being tracked
            if obj_loc_exist := (self.check_existing(obj_loc)):
                # if already tracked (within tolerance), adjust with this observation
                obj_rescue_loc_exist, obj_names, count, _ = self.detect_obj[obj_loc_exist]
                if d.name not in obj_names: 
                    obj_names[d.name] = 0
                obj_names[d.name] += 1
                new_loc = (np.array(obj_loc_exist) * count + np.array(obj_loc)) / (count + 1)
                new_loc = tuple(new_loc)
                new_rescue_loc = (np.array(obj_rescue_loc_exist) * count + np.array(obj_rescue_loc)) / (count + 1)
                new_rescue_loc = tuple(new_rescue_loc)
                count += 1
                self.detect_obj.pop(obj_loc_exist)
                best_name_count = max(obj_names.values())
                for i in obj_names.keys(): 
                    if obj_names[i] == best_name_count:
                        best_name = i
                        break 
                self.detect_obj[new_loc] = (new_rescue_loc, obj_names, count, best_name)
                # print("detect_obj:",self.detect_obj[new_loc])
                
            else:
                self.detect_obj[obj_loc] = (obj_rescue_loc, {d.name: 1}, 1, d.name)
                # print("detect_obj:",self.detect_obj[obj_loc])



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
