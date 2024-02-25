#!/usr/bin/env python
# coding: utf-8

# Importing modules
import firebase_admin
from firebase_admin import db,credentials,storage,initialize_app
import face_recognition as fr
from datetime import datetime

# Authentication for database
if not firebase_admin._apps:
    # Authentication for database
    cred = credentials.Certificate("Certificate.json")
    firebase_admin.initialize_app(cred, {
        'databaseURL': 'https://dissertation-fd159-default-rtdb.europe-west1.firebasedatabase.app/',
        "storageBucket": "dissertation-fd159.appspot.com"
    })

# Reference to firebase Storage Bucket
bucket = storage.bucket('dissertation-fd159.appspot.com')

# Reference to firebase database
ref = db.reference('/recognised_faces_log')

# Downloads accepted face image from database when called
def download_image(remote_path, local_path):
    blob = bucket.blob(remote_path)
    blob.download_to_filename(local_path)
    print(f"Image downloaded to {local_path}")

remote_image_path = "Faces/jonathan.jpg"  # Path to the image in Firebase Storage
local_image_path = "downloaded_image.jpg"  # Local path where the image will be downloaded
download_image(remote_image_path, local_image_path)

remote_image_path = "Faces/Phil.jpg"  # Path to the image in Firebase Storage
local_image_path = "phil_image.jpg"  # Local path where the image will be downloaded
download_image(remote_image_path, local_image_path)


# In[144]:


# Load known images
trainedImage = fr.load_image_file("downloaded_image.jpg")
trainedImage2 = fr.load_image_file("phil_image.jpg")

# Encode known images
try:
    trainedEnc = fr.face_encodings(trainedImage)[0]
    trainedEnc2 = fr.face_encodings(trainedImage2)[0]
except IndexError:
    print("I wasn't able to locate any faces in at least one of the images. Check the image files. Aborting...")
    quit()

# List of recognised faces and their names
known_faces = {
    "Jonathan": trainedEnc,
    "Phil": trainedEnc2
    }

# Load and encode unknown images - Try/Except to handle any errors where the image hasn't been found or no faces are found
try:
    unknownImage = fr.load_image_file("unknown.jpg")
except (FileNotFoundError, IOError):
    print("Wrong file or file path")
try:
    unknownEnc = fr.face_encodings(unknownImage)[0]
except IndexError:
    print("I wasn't able to locate any faces in at least one of the images. Check the image files. Aborting...")
    quit()

# Defining a list to store recognised names
recognised_names = []

# Check for recognised faces
for name, encoding in known_faces.items():
    if fr.compare_faces([encoding], unknownEnc)[0]:
        recognised_names.append(name)

# Update the log on the database for who was recognised and when - Useful for access logs when fully implemented 
if recognised_names:
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    for name in recognised_names:
        ref.push({
            "name": name,
            "date_time": current_time
        })
    print("Hello", name + ",", "your face has been recognised successfully")
else:
    print("No recognised faces.")


