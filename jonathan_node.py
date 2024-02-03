#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False

    def image_callback(self, data):
        if not self.image_received:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                cv2.imwrite("/home/qtrobot/catkin_ws/src/jonathan/src/image.jpg", cv_image)
                self.image_received = True
                print("Image saved.")
            except CvBridgeError as e:
                print("CvBridgeError:", e)

def main():
    rospy.init_node('jonathan_node')
    rospy.loginfo("jonathan_node started!")

    speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
    rospy.sleep(5.0)  # Increased delay for speech setup
    speechSay_pub.publish("Hello! I am going to capture an image now.")

    ic = image_converter()
    rospy.Subscriber("/camera/color/image_raw", Image, ic.image_callback)

    while not ic.image_received and not rospy.is_shutdown():
        rospy.sleep(0.1)

    rospy.sleep(2.0)  # Delay to ensure speech message is processed
    rospy.loginfo("Image capture complete. Node will shutdown.")
    rospy.signal_shutdown("Image Captured")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")


