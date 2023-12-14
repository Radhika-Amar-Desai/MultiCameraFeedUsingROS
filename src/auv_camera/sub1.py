#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DualCameraSubscriber:
    def __init__(self, topic1, topic2):
        self.topic1 = topic1
        self.topic2 = topic2
        self.bridge = CvBridge()

        self.frame1 = None
        self.frame2 = None

        # Create a timer to display frames at a regular interval
        self.timer = rospy.Timer(rospy.Duration(0.1), self.display_side_by_side)

    def callback1(self, msg):
        self.frame1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def callback2(self, msg):
        self.frame2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def display_side_by_side(self, event):
        # Check if both frames are available
        if self.frame1 is not None and self.frame2 is not None:
            # Resize the frames to have the same height (assuming both frames have the same height)
            height, width, _ = self.frame1.shape
            frame1_resized = cv2.resize(self.frame1, (int(width / 2), int(height / 2)))
            frame2_resized = cv2.resize(self.frame2, (int(width / 2), int(height / 2)))

            # Display frames side by side
            combined_frame = np.hstack([frame1_resized, frame2_resized])

            # Use rospy.rostime to execute GUI operations in the main thread
            rospy.rostime.wallsleep(0.1)
            cv2.imshow('Dual Camera Feed', combined_frame)
            cv2.waitKey(1)

def dual_camera_subscriber():
    rospy.init_node('dual_camera_subscriber', anonymous=True)

    subscriber = DualCameraSubscriber(topic1='/camera_topic1', topic2='/camera_topic2')
    rospy.Subscriber(subscriber.topic1, Image, subscriber.callback1)
    rospy.Subscriber(subscriber.topic2, Image, subscriber.callback2)

    rospy.spin()

if __name__ == '__main__':
    try:
        dual_camera_subscriber()
    except rospy.ROSInterruptException:
        pass

