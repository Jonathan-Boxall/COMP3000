#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
    def __init__(self, permission_granted_callback):
        self.bridge = CvBridge()
        self.image_sub = None
        self.permission_granted_callback = permission_granted_callback

    def start_camera(self):
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite("/home/qtrobot/catkin_ws/src/jonathan/src/image.jpg", cv_image)
            rospy.loginfo("Image saved.")
            rospy.signal_shutdown("Image Captured")
        except CvBridgeError as e:
            rospy.loginfo("CvBridgeError: {}".format(e))

def speech_recognition_callback(msg):
    # Implement the logic to handle the speech recognition here
    # For example, if msg.data is 'yes', call the permission_granted_callback
    if msg.data.lower() == 'yes':
        image_converter.start_camera()

def main():
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    global image_converter
    image_converter = ImageConverter(permission_granted_callback=speech_recognition_callback)

    speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=1)
    speech_recog_sub = rospy.Subscriber('/qt_robot/speech/recognition', String, speech_recognition_callback)

    rospy.sleep(1.0)  # Wait for the publisher to set up
    speech_pub.publish("Can I take a picture? Please say yes or no.")

    rospy.spin()  # Keep the node running until shutdown is signaled

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")

