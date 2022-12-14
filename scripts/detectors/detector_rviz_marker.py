#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from robot_autonomy.msg import LocObject, LocObjectList

ID_OFFSET = 100 # offset to apply to the id of various markers
DEBUG_RESCUE = True # put marker at rescue location

class DetectorRvizMarker:

    def __init__(self):
        rospy.init_node('detector_rviz_marker', anonymous=True)

        # subscriber
        self.detected_objs = rospy.Subscriber('/detector/detected_objs_clean', LocObjectList, self.update_markers)

        # publisher
        self.pub = rospy.Publisher('marker_topic', Marker, queue_size=10)

    def run(self): 
        rate = rospy.Rate(10)
        while not rospy.is_shutdown(): 
            rate.sleep()

    def update_markers(self,data):
        for n, det in enumerate(data.objs): 
            # create markers
            markerObj = Marker()            
            markerObj.header.stamp = rospy.Time()
            markerObj.header.frame_id = "map"
            markerObj.id = n
            markerObj.type = 2 #sphere
            markerObj.pose.position.x = float(det.x)
            markerObj.pose.position.y = float(det.y)
            markerObj.pose.position.z = 0.2
            markerObj.scale.x = 0.2
            markerObj.scale.y = 0.2
            markerObj.scale.z = 0.2
            markerObj.color.a = 1.0 # alpha
            markerObj.color.r = 1.0
            markerObj.color.g = 0.0
            markerObj.color.b = 0.0

            if DEBUG_RESCUE:
                markerRescue = Marker()            
                markerRescue.header.stamp = rospy.Time()
                markerRescue.header.frame_id = "map"
                markerRescue.id = n + ID_OFFSET
                markerRescue.type = 2 #sphere
                markerRescue.pose.position.x = float(det.rescue_x)
                markerRescue.pose.position.y = float(det.rescue_y)
                markerRescue.pose.position.z = 0.2
                markerRescue.scale.x = 0.1
                markerRescue.scale.y = 0.1
                markerRescue.scale.z = 0.1
                markerRescue.color.a = 1.0 # alpha
                markerRescue.color.r = 0.0
                markerRescue.color.g = 0.0
                markerRescue.color.b = 1.0

            markerTxt = Marker()
            markerTxt.header.stamp = rospy.Time()
            markerTxt.header.frame_id = "map"
            markerTxt.id = n + 2*ID_OFFSET
            markerTxt.type = 9 #text
            markerTxt.text = det.name
            markerTxt.pose.position.x = float(det.x)
            markerTxt.pose.position.y = float(det.y)
            markerTxt.pose.position.z = 0.5
            markerTxt.scale.z = 0.2 # z used for height of uppercase A
            markerTxt.color.a = 1.0 # alpha
            markerTxt.color.r = 1.0
            markerTxt.color.g = 1.0
            markerTxt.color.b = 1.0

            # publish!
            self.pub.publish(markerObj)
            if DEBUG_RESCUE: self.pub.publish(markerRescue)
            self.pub.publish(markerTxt)


if __name__ == "__main__": 
    DRM = DetectorRvizMarker()
    DRM.run()