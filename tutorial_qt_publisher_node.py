#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.captured_image = None
        self.image_captured_event = False
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.captured_image = cv_image
            self.image_captured_event = True
        except CvBridgeError as e:
            print("Error:", e)

def main():
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
    rospy.sleep(3.0)
    speechSay_pub.publish("Hello! I am going to capture an image now.")

    ic = image_converter()
    rospy.Subscriber("/camera/color/image_raw", Image, ic.image_callback)  # Assuming the image topic is /camera/color/image_raw

    print("Waiting for the image to be captured...")

    while not ic.image_captured_event and not rospy.is_shutdown():
        rospy.sleep(0.1)  # Wait for a short duration before checking again

    if ic.image_captured_event:
        # Image captured, save it to a file
        cv2.imwrite("captured_image.jpg", ic.captured_image)
        print("Image saved as captured_image.jpg")
    else:
        print("Image capture failed.")

    rospy.loginfo("Finished!")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")


