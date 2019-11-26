#!/usr/bin/env python
import sys, time
# numpy and scipy
import numpy as np
# OpenCV
import cv2
# Ros libraries
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String as rosString
# Other libraries
import argparse
from os import mkdir
from os.path import join, exists
import ntplib
from std_msgs.msg import Int32
import random
import argparse
import json
import socket


# EXAMPLE RUN: python collect_data.py -s S1 -t T2

class DataCapture:
    def __init__(self, subject, trial, path):
        # camera topic names
        left_camera = "/modified_display_left/compressed"
        right_camera = "/modified_display_right/compressed"

        #### Subscribed topics ####
        self.camera_pedal_sub = rospy.Subscriber("/dvrk/footpedals/coag", Joy, self.pedal_callback)
        #PSM1 topics
        self.psm1_cartesian_subs = rospy.Subscriber("/dvrk/PSM1/position_cartesian_current", PoseStamped, self.psm1_cartesian_callback)
        self.psm1_joints_subs = rospy.Subscriber("/dvrk/PSM1/state_joint_current", JointState, self.psm1_joints_callback)
        self.psm1_gripper_subs = rospy.Subscriber("/dvrk/PSM1/state_jaw_current", JointState, self.psm1_gripper_callback)
        #PSM2 topics
        self.psm2_cartesian_subs = rospy.Subscriber("/dvrk/PSM2/position_cartesian_current", PoseStamped, self.psm2_cartesian_callback)
        self.psm2_joints_subs = rospy.Subscriber("/dvrk/PSM2/state_joint_current", JointState, self.psm2_joints_callback)
        self.psm2_gripper_subs = rospy.Subscriber("/dvrk/PSM2/state_jaw_current", JointState, self.psm2_gripper_callback)
        # Camera Topics
        self.subscriber_left = rospy.Subscriber(left_camera,
            CompressedImage, self.callback_left,  queue_size = 1)
        self.subscriber_right = rospy.Subscriber(right_camera,
            CompressedImage, self.callback_right,  queue_size = 1)

        # Initialize files for data collection
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out_left = cv2.VideoWriter(join(path, subject +"_"+trial+"_left_color.avi"),fourcc, 30.0, (720,480))
        self.out_right = cv2.VideoWriter(join(path, subject +"_"+trial+"_right_color.avi"),fourcc, 30.0, (720,480))
        self.out_left_ts = open(join(path, subject +"_"+trial+"_left_color_ts.txt"),'w')
        self.out_right_ts = open(join(path, subject +"_"+trial+"_right_color_ts.txt"),'w')
        self.out_robot_left = open(join(path, subject +"_"+trial+"_left_kinematics.txt"),'w')
        self.out_robot_right = open(join(path, subject +"_"+trial+"_right_kinematics.txt"),'w')

        # Other variables
        self.coag_count = 0
        self.record = False

    def callback_left(self, ros_data):
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.out_left.write(image_np)
        self.out_left_ts.write('%f\n'% time.time())
        # cv2.imshow('cv_img', image_np)
        # cv2.waitKey(2)

    def callback_right(self, ros_data):
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.out_right.write(image_np)
        self.out_right_ts.write('%f\n'% time.time())
        # cv2.imshow('cv_img', image_np)
        # cv2.waitKey(2)

    def pedal_callback(self,data):
        # if the button is pressed, record data
        self.coag_count +=1
        print("coag pressed count: %i, Recording..." % self.coag_count)
        if data.buttons[0]:
            self.record = True
        else:
            self.record = False


    #PSM1 callbacks
    def psm1_cartesian_callback(self, data):
        self.x_right = data.pose.position.x
        self.y_right = data.pose.position.y
        self.z_right = data.pose.position.z

        self.rx_right = data.pose.orientation.x
        self.ry_right = data.pose.orientation.y
        self.rz_right = data.pose.orientation.z
        self.rw_right= data.pose.orientation.w

        if self.record:
            message = "%f, " % time.time()
            # add the joint angles
            angles = "".join(["{: 0.8f}, ".format(joint_i) for joint_i in self.joints_right])
            message += angles
            # add the rotations and positon
            message += "{: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, "\
                    .format(self.rx_right, self.ry_right,self.rz_right,self.rw_right,
                            self.x_right,self.y_right,self.z_right)
            # add the gripper state
            message += "{: 0.8f}\n".format(self.gripper_right)
            self.out_robot_right.write(message)

    def psm1_joints_callback(self, data):
        self.joints_right = data.position

    def psm1_gripper_callback(self, data):
        self.gripper_right = data.position[0]

    #PSM2 callbacks
    def psm2_cartesian_callback(self, data):
        self.x_left = data.pose.position.x
        self.y_left = data.pose.position.y
        self.z_left = data.pose.position.z

        self.rx_left = data.pose.orientation.x
        self.ry_left= data.pose.orientation.y
        self.rz_left = data.pose.orientation.z
        self.rw_left = data.pose.orientation.w

        if self.record:
            message = "%f, " % time.time()
            # add the joint angles
            angles = "".join(["{: 0.8f}, ".format(joint_i) for joint_i in self.joints_left])
            message += angles
            # add the rotations and positon
            message += "{: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, "\
                    .format(self.rx_left, self.ry_left,self.rz_left,self.rw_left,
                            self.x_left,self.y_left,self.z_left)
            # add the gripper state
            message += "{: 0.8f}\n".format(self.gripper_left)
            self.out_robot_left.write(message)

    def psm2_joints_callback(self, data):
        self.joints_left = data.position

    def psm2_gripper_callback(self, data):
        self.gripper_left= data.position[0]

    def close_recordings(self):
        self.out_left.release()
        self.out_right.release()
        self.out_left_ts.close()
        self.out_right_ts.close()
        self.out_robot_left.flush()
        self.out_robot_right.flush()
        self.out_robot_left.close()
        self.out_robot_right.close()

def main(args):
    ##############################
    ###     PARSE ARGS         ###
    ##############################
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', action="store", dest="subject",
            help="Subject number, example: S3")
    parser.add_argument('-t', action="store", dest="trial",
            help="Trial number, example: T2")
    parser.add_argument('-o', action="store", dest="out_path",
            default="./data_collection/",
            help="Path location of the data collection output")
    args = parser.parse_args()
    subject = args.subject
    trial = args.trial
    out_path = args.out_path
    out_path  = join(out_path, subject)
    # check if the path exists
    if exists(join(out_path, subject +"_"+trial+"_left_color.avi")) \
            or exists(join(out_path, subject +"_"+trial+"_right_color.avi")):
        print("Subject and trial already exist")
        exit()
    else:
        # if the subject doesn't have a folder, create it
        # The out folder will look as follows:
        # out_path/SubjectNumber/
        if not exists(out_path):
            mkdir(out_path)
        print("Recording...\n\
               Press CTR+c to finish recording")
        # Initializes and cleanup ros node
        data_capture = DataCapture(subject,trial,out_path)
        rospy.init_node('video_joints_capture', anonymous=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down ROS Image feature detector module"
        print("Close system")
        cv2.destroyAllWindows()
        data_capture.close_recordings()

if __name__ == '__main__':
    main(sys.argv)
