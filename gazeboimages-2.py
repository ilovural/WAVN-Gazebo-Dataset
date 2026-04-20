#!/usr/bin/env python3

#
# Script to automate collection of images from Gazebo
# designed to work with the world/launchfiles WAVNpXXXX
#
# * Setup the list "Locations" with the (x,y,z,yaw) angles
# where you want the the image taken.
# * Set the random displacements to the range that you want
# from the locations
# * Set the total number of images you want (it should be a
# multiple of the number of location for a balanced dataset)
#
# Execute as: python3 gazeboimages.py
# it will take a while if there are a lot of images
# for N images total it will take N/5 seconds, eg 1000 images 
# will take under 4 minutes.
#
# It will generate a folder called saved_image_files in the same folder
# you run the python program. This will contain all the files named
# uniquely for location at which they were taken, and a CSV file with
# the filenames and exact locations and angles they were taken at.
#

import rospy
import csv
import os
import time
import uuid
import random

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import cv2
import numpy as np


Locations = [ # x    y   z   yaw (radians)
            (0.0, 0.0, 0.1, 0.0),
            (0.0, 0.0, 0.1, np.radians(90)),
            (0.0, 0.0, 0.1, np.radians(-90)),
            (0.0, 0.0, 0.1, np.radians(180))
            ]

# camera positions will be a Location offset randomly 
# (unform distrib) in the x,y and yaw based on +/- ranges below
           
random_X_range = 2.0
random_Y_range = 2.0
random_Yaw_range = 10 

number_of_images = 250  # forces a balanced dataset

class CameraDataCollector:
    def __init__(self):
        rospy.init_node("camera_data_collector")

        # Parameters (edit these)
        self.camera_model_name = "Pioneer3at1"
        self.image_topic = "/P1/camera/color/image_raw"
        self.output_dir = "saved_camera_images"
        self.csv_file = os.path.join(self.output_dir, "image_log.csv")

        # Camera poses: (x, y, z, qx, qy, qz, qw)
        self.camera_poses = Locations
        
        self.rx = random_X_range
        self.ry = random_Y_range
        self.ryaw = random_Yaw_range
        
        self.N = number_of_images
        self.count = 0

        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_image = None

        rospy.Subscriber(self.image_topic, Image, self.image_callback)

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState
        )

        self.init_csv()

    def init_csv(self):
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    ["filename", "x", "y", "z", "yaw"]
                )

    def image_callback(self, msg):
        self.latest_image = msg

    def move_camera(self, pose):
        x, y, z, yaw = pose
        # Convert yaw-only to quaternion (roll=0, pitch=0)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

        state = ModelState()
        state.model_name = self.camera_model_name
        state.pose.position.x = pose[0]
        state.pose.position.y = pose[1]
        state.pose.position.z = pose[2]
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw

        self.set_model_state(state)

    def save_image(self, i, pose):
        if self.latest_image is None:
            rospy.logwarn("No image received yet")
            return

        cv_image = self.bridge.imgmsg_to_cv2(
            self.latest_image, desired_encoding="bgr8"
        )
        # unique filename with location index and unique string
        filename = f"Loc{i}-{uuid.uuid4().hex}.png"
        filepath = os.path.join(self.output_dir, filename)
        cv2.imwrite(filepath, cv_image)

        # log this image name and actual location it was taken
        with open(self.csv_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([filename] + list(pose[0:3]) + [np.degrees(pose[3])])

        #rospy.loginfo(f"Saved {filename}")

    def runOnce(self):
        for i,loc in enumerate(self.camera_poses):
            pose=list(loc) # make it mutable
            rx = random.uniform(-self.rx,self.rx)
            ry = random.uniform(-self.ry,self.ry)
            ryaw = random.uniform(-self.ryaw,self.ryaw)
            pose[0] += rx
            pose[1] += ry
            pose[3] += ryaw
            self.move_camera(pose)
            rospy.sleep(0.2) # Let Gazebo settle
            self.save_image(i,pose)
            
    def run(self):
        rospy.sleep(1.0)  # Allow subscriptions to settle
        for self.count in range(1,self.N):
            self.runOnce()
            if rospy.is_shutdown():
                break # allow this to be killed by ^C from terminal
            if (self.count*len(self.camera_poses) % 100) == 0:
                print(f"Completed {self.count*len(self.camera_poses)} images....")
        print(f"Completed {self.N*len(self.camera_poses)} images.")

if __name__ == "__main__":
    try:
        collector = CameraDataCollector()
        collector.run()
    except rospy.ROSInterruptException:
        pass
