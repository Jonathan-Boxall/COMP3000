#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import face_recognition as fr
from datetime import datetime
import firebase_admin
from firebase_admin import db, credentials, storage
from std_msgs.msg import String
import time
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
import random

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
        self.ref_emails = db.reference('/emails')  # Reference to the emails node
        self.ref_faces_log = db.reference('/recognised_faces_log')  # Reference to the recognised faces log node
        self.talk = rospy.Publisher('/qt_robot/behavior/talkText', String, queue_size=10)
        self.authorised = False
        
    def download_image(self, remote_path, local_path):
        bucket = storage.bucket('dissertation-fd159.appspot.com')
        blob = bucket.blob(remote_path)
        blob.download_to_filename(local_path)
        rospy.loginfo(f"Image downloaded to {local_path}")
        
    def send_email(self, recipient, subject, body):
        #Email details
        sender_email = "qtrobottest@outlook.com"
        sender_password = "QTrobot24!"
        
        #Creates message
        msg = MIMEMultipart()
        msg['From'] = sender_email
        msg['To'] = recipient
        msg['Subject'] = subject
        msg.attach(MIMEText(body, 'plain'))

        server = smtplib.SMTP('smtp-mail.outlook.com', 587) #Email server details
        #Starts server connection, sends message and then kills the connection
        server.starttls()
        server.login(sender_email, sender_password)
        server.send_message(msg)
        server.quit()
        
    def get_user_email(self, name):
        # Retrieve email from Firebase based on the name
        return self.ref_emails.child(name).get()

    def get_user_input(self):
        # Prompt the user to enter input and return it
        return input("Enter input: ")
    
    def image_callback(self, data):
        if not self.authorised:
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
                        self.ref_faces_log.push({
                            "name": name,
                            "date_time": current_time
                        })
                        # In-line message to say they have been authorised, should be spoken by the robot too with the next code but this is a backup for debugging.
                        rospy.loginfo(f"Hello {name}, your face has been recognised successfully")
                        
                        # Get robot to speak that the user has been authorised.
                        self.talk.publish(f"Welcome {name}, A two-factor authentication code has been sent to your email. Please enter the code.")
                        
                        # Get user's email from Firebase
                        user_email = self.get_user_email(name.lower())
                        
                        # Send 2FA code to the user's email
                        two_factor_code = str(random.randint(1000, 9999))
                        email_subject = "Two-factor Authentication Code"
                        email_body = f"Your two-factor authentication code is: {two_factor_code}"
                        self.send_email(user_email, email_subject, email_body)
                        
                        #User input
                        user_code = self.get_user_input()
                        
                        # Validate the entered code
                        if user_code == two_factor_code:
                            self.authorised = True
                            rospy.loginfo("User successfully authenticated.")
                            self.talk.publish(f"Welcome {name}, you have been successfully authenticated.")
                        else:
                            self.talk.publish("Invalid authentication code. Access denied.")
                            self.talk.publish(f"Sorry {name}, the code you have entered is invalid. Please try again or seek admin support.")
                    
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

