#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class CameraPublisher:
    def __init__(self, camera_id, topic):
        self.camera_id = camera_id
        self.topic = topic
        self.cap = cv2.VideoCapture(self.camera_id)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.topic, Image, queue_size=10)
        self.thread = threading.Thread(target=self.publish_frames)
        self.thread.daemon = True

    def start(self):
        self.thread.start()

    def publish_frames(self):
        rate = rospy.Rate(20)  # 10Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.pub.publish(img_msg)
                rate.sleep()

    def release(self):
        self.cap.release()

def multithreaded_camera_publisher():
    rospy.init_node('multithreaded_camera_publisher', anonymous=True)
    
    cameras = [
        CameraPublisher(camera_id=0, topic='camera_topic1'),
        CameraPublisher(camera_id=2, topic='camera_topic2')
    ]

    for camera in cameras:
        camera.start()

    rospy.on_shutdown(lambda: [camera.release() for camera in cameras])

    rospy.spin()

if __name__ == '__main__':
    try:
        multithreaded_camera_publisher()
    except rospy.ROSInterruptException:
        pass

