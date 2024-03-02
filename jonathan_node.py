#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import face_recognition as fr
from datetime import datetime
import firebase_admin
from firebase_admin import db, credentials, storage

# Initialize Firebase
if not firebase_admin._apps:
    cred = credentials.Certificate("Certificate.json")
    firebase_admin.initialize_app(cred, {
        'databaseURL': 'https://dissertation-fd159-default-rtdb.europe-west1.firebasedatabase.app/',
        'storageBucket': 'dissertation-fd159.appspot.com'
    })

class FaceRecognition:
    def __init__(self):
        rospy.init_node('face_recognition', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.ref = db.reference('/recognised_faces_log')

    def download_image(self, remote_path, local_path):
        bucket = storage.bucket('dissertation-fd159.appspot.com')
        blob = bucket.blob(remote_path)
        blob.download_to_filename(local_path)
        rospy.loginfo(f"Image downloaded to {local_path}")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite("test.jpg", cv_image)  # Save the image
            
            # Load known images
            self.download_image("Faces/jonathan.jpg", "downloaded_image.jpg")  # Path to the known image
            self.download_image("Faces/Phil.jpg", "phil_image.jpg")  # Path to the second known image
            
            trainedImage = fr.load_image_file("downloaded_image.jpg")
            trainedImage2 = fr.load_image_file("phil_image.jpg")
            trainedEnc = fr.face_encodings(trainedImage)[0]
            trainedEnc2 = fr.face_encodings(trainedImage2)[0]
            
            # Load and encode unknown image
            unknownImage = fr.load_image_file("test.jpg")  # Path to the saved image
            unknownEnc = fr.face_encodings(unknownImage)[0]
            
            # Compare faces
            recognised_names = []
            if fr.compare_faces([trainedEnc], unknownEnc)[0]:
                recognised_names.append("Jonathan")
            if fr.compare_faces([trainedEnc2], unknownEnc)[0]:
                recognised_names.append("Phil")
            
            # Update the log for who was recognised and when
            if recognised_names:
                current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                for name in recognised_names:
                    self.ref.push({
                        "name": name,
                        "date_time": current_time
                    })
                    rospy.loginfo(f"Hello {name}, your face has been recognised successfully")
            else:
                rospy.loginfo("No recognised faces.")
                
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
        except FileNotFoundError:
            rospy.logerr("Image file not found.")
        except IndexError:
            rospy.logerr("No faces found in the image.")

if __name__ == '__main__':
    face_recognition = FaceRecognition()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

