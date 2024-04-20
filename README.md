# COMP3000 

COMP3000 Dissertation project  - Security and Authentication for Social Robots Through Facial Recognition

Project Supervisor: Hai-Van Dang  

This project is the implementation of facial recognition and two-factor email authentication through the use of randomly generated codes into a pre-existing social robot (QTRobot by luxai). This project will be achieved using Python to add extra levels of security to these devices. This is primarily going to be used by elderly, special needs children and any guardians or carers they have, as this is the primary use of the QTRobot.
  

The following libraries have been used: 

    rospy 

    sensor_msgs.msg (part of ROS) 

    cv_bridge (part of ROS) 

    cv2 (OpenCV) 

    face_recognition 

    datetime 

    firebase_admin 

    std_msgs.msg (part of ROS) 

    time 

    smtplib (Python standard library for sending emails) 

    email.mime.multipart (Python standard library for creating MIME multipart messages) 

    email.mime.text (Python standard library for creating MIME text messages) 

    random 

 To use this program, you must have a rosnode setup for it to run from. Once this is done, simply run it in the terminal where it will take a picture of your face to compare with the known faces stored in the database.  

If your face has been setup as a known face, you will be recognised and sent a 2FA code to your registered email (also held in the database). Upon recognition, an access log held in the database will also be updated with your name and time of access. 

After the 2FA code is sent, the user will be prompted to enter it to gain full authentication.  
