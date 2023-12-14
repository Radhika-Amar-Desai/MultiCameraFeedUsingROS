#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow('Camera 2 Feed', frame)
        cv2.waitKey(1)

def camera_subscriber():
    rospy.init_node('dual_camera_subscriber_2', anonymous=True)

    subscriber = CameraSubscriber(topic ='/camera2_topic')
    rospy.Subscriber(subscriber.topic, Image, subscriber.callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        camera_subscriber()
    except rospy.ROSInterruptException:
        pass

