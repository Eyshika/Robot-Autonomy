#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from robot_autonomy.msg import LocObjectList


class SpeedControl:

    def __init__(self):
        rospy.init_node('speed_control', anonymous=True)

        # subscriber
        self.detected_objs_clean = rospy.Subscriber('/detector/detected_objs_clean', LocObjectList, self.speed_update)
        self.velocity = rospy.Subscriber('/cmd_nav', Twist, self.get_velocity)
        # publishers
        self.vel_pub = rospy.Publisher('/control_nav', Twist, queue_size=10)
        self.labelsforlowvelocity = ["cow", "bear", "giraffe", "zebra", "cat", "dog", "elephant", "sheep"]
        
        self.labelsforstop = ["person", "stop_sign"]
        self.current_linear = 0
        self.current_angular = 0
        self.locations = {}
        # self.init_timer()

    def init_timer(self):
        self.time_ = rospy.Time.now()

    def run(self): 
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # time_difference = rospy.Time.now() - self.time_ 
            # print("time difference: ", time_difference)
            # if time_difference > 40:
            #     self.clear_location()
            #     self.time_ = rospy.Time.now()
            rate.sleep()
            

    def get_velocity(self, vel):
        self.current_linear = vel.linear.x
        self.current_angular = vel.angular.z

    def stop_robot(self, event):
        rospy.loginfo("Stopped for 3 sec")

    def clear_location(self):
        if self.locations:
            del self.locations
        rospy.loginfo("Cleared Locations !!")

    def speed_update(self, data):
        cmd_vel = Twist()
        ## slower
        # print(data.objs)
        for n, det in enumerate(data.objs):
            flag = False
            checking_location = (round(float(det.x), 2), round(float(det.y), 2))
            key_index = list(self.locations).index(checking_location) if checking_location in self.locations else None

            if key_index is None or list(self.locations.values()[key_index]) != det.name:

                if det.name in self.labelsforlowvelocity:
                    flag = True
                    rospy.loginfo("Detected %s Slowering the Robot for 3 sec" %det.name)
                    cmd_vel.linear.x = self.current_linear * 0.30
                    cmd_vel.angular.z = self.current_angular * 0.30 

                elif det.name in self.labelsforstop:
                    flag = True
                    rospy.loginfo("Detected %s Stopping the Robot for 3 sec" %det.name)
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                # rospy.Timer(rospy.Duration(3), self.stop_robot)
                 # Save current time and set publish rate at 10 Hz
                now = rospy.Time.now()

                # For the next 6 seconds publish cmd_vel move commands to Turtlesim
                while rospy.Time.now() < now + rospy.Duration.from_sec(5.5):
                    self.vel_pub.publish(cmd_vel)
                    rospy.loginfo("controlled velocity for 6 secs")
                self.locations[(round(float(det.x), 2), round(float(det.y), 2))] =  det.name
        rospy.loginfo(self.locations)
            # if flag:
            #     rospy.Timer(rospy.Duration(20), self.stop_robot)
            #     rospy.loginfo("Starting the robot back")
    
if __name__ == "__main__": 
    SC = SpeedControl()
    SC.run()