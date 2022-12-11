#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from robot_autonomy.msg import LocObject, LocObjectList


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
            marker = Marker()            
            marker.header.stamp = rospy.Time()
            marker.header.frame_id = "map"
            marker.id = n
            marker.type = 2 #sphere
            marker.pose.position.x = det.x
            marker.pose.position.y = det.y
            marker.pose.position.z = 0.2
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0 # alpha
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            markerTxt = Marker()
            markerTxt.header.stamp = rospy.Time()
            markerTxt.header.frame_id = "map"
            markerTxt.id = n+100
            markerTxt.type = 9 #text
            markerTxt.text = det.name
            markerTxt.pose.position.x = det.x
            markerTxt.pose.position.y = det.y
            markerTxt.pose.position.z = 0.5
            markerTxt.scale.z = 0.2 # z used for height of uppercase A
            markerTxt.color.a = 1.0 # alpha
            markerTxt.color.r = 1.0
            markerTxt.color.g = 1.0
            markerTxt.color.b = 1.0

            # publish!
            self.pub.publish(marker)
            self.pub.publish(markerTxt)


if __name__ == "__main__": 
    DRM = DetectorRvizMarker()
    DRM.run()