#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import cv2 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import ntplib
import time
import datetime
import numpy as np 
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String as rosString
import random 
import argparse
import json
import socket


class data_collection_module:
    def __init__(self,totalTime =1, secondaryTime = 5, userId=None, trialId = None):

        #Create files
        #PSM1 files
        self.psm1FilesDict = {}
        self.psm2FilesDict = {}
        self.initialTimestamp = self.createTimeStamp()
        self.userId = userId
        self.trialId = trialId
        self.initFiles()

        #Important variables
        self.initTime = time.time()
        self.secondaryTime = secondaryTime
        self.turnSecondaryTask = True
        self.lastActivation = -1
        self.startProcedure = False
        self.stopProcedure = False
        self.totalTime = totalTime*60 #In seconds
        self.client = None
        self.maxPossibleScore = (secondaryTime % 10)*2 * totalTime/2
        self.score = 0
        self.correct = 0
        self.incorrect = 0

        self.bridge = CvBridge()

        #Subscribed topics
        self.image_sub_left  = rospy.Subscriber("/pu_dvrk_cam/left/inverted", Image, self.left_callback)
        self.image_sub_right = rospy.Subscriber("/pu_dvrk_cam/right/inverted", Image, self.right_callback)
        self.camera_pedal_sub = rospy.Subscriber("/dvrk/footpedals/bicoag", Joy, self.pedal_callback)

        #PSM1 topics
        self.psm1_cartesian_subs = rospy.Subscriber("/dvrk/PSM1/position_cartesian_current", PoseStamped, self.psm1_cartesian_callback)
        self.psm1_joints_subs = rospy.Subscriber("/dvrk/PSM1/state_joint_current", JointState, self.psm1_joints_callback)
        self.psm1_gripper_subs = rospy.Subscriber("/dvrk/PSM1/state_jaw_current", JointState, self.psm1_gripper_callback)

        #PSM2 topics
        self.psm1_cartesian_subs = rospy.Subscriber("/dvrk/PSM2/position_cartesian_current", PoseStamped, self.psm2_cartesian_callback)
        self.psm1_joints_subs = rospy.Subscriber("/dvrk/PSM2/state_joint_current", JointState, self.psm2_joints_callback)
        self.psm1_gripper_subs = rospy.Subscriber("/dvrk/PSM2/state_jaw_current", JointState, self.psm2_gripper_callback)	


        #Additional variables
        self.misalignment = 75
        self.fontSize = 1.2
        self.message =  ""
        self.timerStr = ""
        self.scoreStr = ""
        self.alpha = 0.45
        self.numberOfTargets = 2
        self.target = random.sample(range(min(secondaryTime,10)), self.numberOfTargets)

        #Blink a green/red rectangle on screen to indicate the user the secondary task is starting
        self.notifyUser= False
        self.notificationColor = (0,0,0)

        #Kernel used to blurr images when secondary task is active
        self.smoothingKernel = np.ones((5,5),np.float32)/25


    def initFiles(self):
        timeStamp = self.initialTimestamp
        timeStamp = ""

        fileName1 = "forward_S{:s}_T{:s}_PSM1_cartesian.txt".format(str(self.userId), str(self.trialId))
        self.psm1FilesDict['cartesian_file'] = open("/home/juan/ForwardData/" + timeStamp + '_' + fileName1,'w')
        self.psm1FilesDict['cartesian_file'].write("timestamp x y z rx ry rz rw\n")

        fileName2 = "forward_S{:s}_T{:s}_PSM1_joints.txt".format(str(self.userId), str(self.trialId))
        self.psm1FilesDict['joints_file'] = open("/home/juan/ForwardData/" + timeStamp + '_' + fileName2,'w')		
        self.psm1FilesDict['joints_file'].write("timestamp j1 j2 j3 j4 j5 j6\n")		

        fileName3 = "forward_S{:s}_T{:s}_PSM1_gripper.txt".format(str(self.userId), str(self.trialId))
        self.psm1FilesDict['gripper_file'] = open("/home/juan/ForwardData/" + timeStamp + '_' + fileName3,'w') 
        self.psm1FilesDict['gripper_file'].write("timestamp j7\n")

        fileName1 = "forward_S{:s}_T{:s}_PSM2_cartesian.txt".format(str(self.userId), str(self.trialId))
        self.psm2FilesDict['cartesian_file'] = open("/home/juan/ForwardData/" + timeStamp + '_' + fileName1,'w')
        self.psm2FilesDict['cartesian_file'].write("timestamp x y z rx ry rz rw\n")

        fileName2 = "forward_S{:s}_T{:s}_PSM2_joints.txt".format(str(self.userId), str(self.trialId))
        self.psm2FilesDict['joints_file'] = open("/home/juan/ForwardData/" + timeStamp + '_' + fileName2,'w')		
        self.psm2FilesDict['joints_file'].write("timestamp j1 j2 j3 j4 j5 j6\n")		

        fileName3 = "forward_S{:s}_T{:s}_PSM2_gripper.txt".format(str(self.userId), str(self.trialId))
        self.psm2FilesDict['gripper_file'] = open("/home/juan/ForwardData/" + timeStamp + '_' + fileName3,'w')
        self.psm2FilesDict['gripper_file'].write("timestamp j7\n")

    def update(self):
        if self.startProcedure and not self.stopProcedure:
            currentTime = time.time()
            secondsCounter = int((currentTime - self.initTime))
            seconds = secondsCounter % 60
            minutes = int(secondsCounter / 60)
            self.timerStr = "Timer: {:02d}:{:02d}".format(minutes,seconds)

            #Check if procedure is already over.
            if time.time() - self.initTime > self.totalTime:
                secondaryTaskStatus = "finished"

                #Writing to file
                self.file.write("{:.9f} {}\n".format(time.time(), secondaryTaskStatus))
                self.file.write("##DATA##\n")
                self.file.write("{} {}\n".format("Score", self.score))
                self.file.write("{} {:.3f}\n".format("Max possible Score", float(self.maxPossibleScore) ))
                #self.file.write("{} {:.3f}\n".format("Accuracy", float(self.score/self.maxPossibleScore) ))
                self.file.write("{} {}\n".format("Incorrect", self.incorrect))
                self.file.write("{} {}\n".format("Correct", self.correct))
                self.file.flush()

                self.stopProcedure = True
                self.message = "Procedure finished"
            #If the procedure have not finished, check if the status of the secondary task have to change
            elif secondsCounter % self.secondaryTime == 0:
                if secondsCounter != self.lastActivation:
                    self.turnSecondaryTask = not self.turnSecondaryTask
                    self.target = random.sample(range(1,min(self.secondaryTime,10)), self.numberOfTargets)
                    self.lastActivation = secondsCounter
                    secondaryTaskStatus = "active" if self.turnSecondaryTask else "not_active"

                    #Writing to file
                    self.file.write("{:.9f} {}\n".format(time.time(), secondaryTaskStatus))
                    self.file.flush()

                    if self.turnSecondaryTask:
                        temp = " ".join(map(str,self.target))
                        self.message  = "Do secondary, Targets: {:s}".format(temp)
                        self.scoreStr = "Score: {:3d}".format(self.score)
                        self.alpha = 0.45
                    else:
                        self.message  = "Do only primary task"
                        self.scoreStr = "Score: {:3d}".format(self.score)
                        self.alpha = 0.30

    def modifyImageAndPublish(self,cv_image, misalignment=0, publisherId=1):
            publisher = self.image_pub1 if publisherId == 1 else self.image_pub2
            compressedPublisher = self.image_pub1_compressed if publisherId == 1 else self.image_pub2_compressed
            overlay = cv_image.copy()
            cv2.putText(overlay, self.message,(10+misalignment, 30), cv2.FONT_HERSHEY_SIMPLEX, self.fontSize, (0, 0, 255), 3)
            cv2.putText(overlay, self.timerStr+"  "+self.scoreStr,(10+misalignment, 75), cv2.FONT_HERSHEY_SIMPLEX, self.fontSize, (0, 0, 255), 3)

            if self.notifyUser:
                    color = self.notificationColor
                    cv2.rectangle(overlay, (660, 0), (720,60), color, -1)

            cv2.addWeighted(overlay, self.alpha, cv_image, 1 - self.alpha, 0, cv_image)

            try:
                    publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                    print(e)

            #### Create and Publish Compressed Image ####
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
            compressedPublisher.publish(msg)

    def left_callback(self,data):

            try:
                    cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            except CvBridgeError as e:
                    print(e)

            self.modifyImageAndPublish(cv_image, misalignment=0, publisherId=1)

    def right_callback(self,data):

            try:
                    cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            except CvBridgeError as e:
                    print(e)

            self.modifyImageAndPublish(cv_image, misalignment=self.misalignment, publisherId=2)

    def pedal_callback(self,data):
        if not self.startProcedure and data.buttons[0]:
            self.message  = "Configuring procedure ..."
            self.message  = "Starting procedure in 3..."
            time.sleep(1)
            self.message  = "Starting procedure in 2..."
            time.sleep(1)
            self.message  = "Starting procedure in 1..."
            time.sleep(1)
            self.initTime = time.time()
            self.message  = "Recording"
            self.startProcedure = True
        elif self.startProcedure and data.buttons[0]:
            self.message  = "Stopped recording"
            self.startProcedure = False

    #PSM1 callbacks
    def psm1_cartesian_callback(self, data):
        if self.startProcedure:
            x = data.pose.position.x
            y = data.pose.position.y
            z = data.pose.position.z

            rx = data.pose.orientation.x
            ry = data.pose.orientation.y
            rz = data.pose.orientation.z
            rw = data.pose.orientation.w

            timestamp = data.header.stamp.secs + data.header.stamp.nsecs*(10**(-9))
            message = "{:0.7f} {: 0.7f} {: 0.7f} {: 0.7f} {: 0.7f} {: 0.7f} {: 0.7f} {: 0.7f}\n".format(timestamp,x,y,z,rx,ry,rz,rw)

            self.psm1FilesDict['cartesian_file'].write(message)

    def psm1_joints_callback(self, data):
        if self.startProcedure:
            joints = data.position
            temp = " ".join(["{: 0.8f}".format(joint_i) for joint_i in joints])
            timestamp = data.header.stamp.secs + data.header.stamp.nsecs*(10**(-9))
            message = "{:0.7f} {:s}\n".format(timestamp, temp)
            self.psm1FilesDict['joints_file'].write(message)

    def psm1_gripper_callback(self, data):
        if self.startProcedure:
            joints = data.position
            temp = " ".join(["{: 0.8f}".format(joint_i) for joint_i in joints])
            timestamp = data.header.stamp.secs + data.header.stamp.nsecs*(10**(-9))

            message = "{:0.7f} {:s}\n".format(timestamp, temp)
            self.psm1FilesDict['gripper_file'].write(message)

    #PSM2 callbacks
    def psm2_cartesian_callback(self, data):
        if self.startProcedure:
            x = data.pose.position.x
            y = data.pose.position.y
            z = data.pose.position.z

            rx = data.pose.orientation.x
            ry = data.pose.orientation.y
            rz = data.pose.orientation.z
            rw = data.pose.orientation.w

            timestamp = data.header.stamp.secs + data.header.stamp.nsecs*(10**(-9))
            message = "{:0.7f} {: 0.7f} {: 0.7f} {: 0.7f} {: 0.7f} {: 0.7f} {: 0.7f} {: 0.7f}\n".format(timestamp,x,y,z,rx,ry,rz,rw)

            self.psm2FilesDict['cartesian_file'].write(message)

    def psm2_joints_callback(self, data):
        if self.startProcedure:
            joints = data.position
            temp = " ".join(["{: 0.8f}".format(joint_i) for joint_i in joints])

            timestamp = data.header.stamp.secs + data.header.stamp.nsecs*(10**(-9))
            message = "{:0.7f} {:s}\n".format(timestamp, temp)
            self.psm2FilesDict['joints_file'].write(message)

    def psm2_gripper_callback(self, data):
        if self.startProcedure:
            joints = data.position
            temp = " ".join(["{: 0.8f}".format(joint_i) for joint_i in joints])
            timestamp = data.header.stamp.secs + data.header.stamp.nsecs*(10**(-9))

            message = "{:0.7f} {:s}\n".format(timestamp, temp)
            self.psm2FilesDict['gripper_file'].write(message)


    #Close files
    def closeFiles(self):
        for key, item in self.psm1FilesDict.items():
            item.flush()
            item.close()

        for key, item in self.psm2FilesDict.items():
            item.flush()
            item.close()

def main(userId, trialId):
    print("Starting Da vinci Operation...")
    ic = data_collection_module(userId = userId, trialId = trialId,
                                                        secondaryTime = 60,
                                                        totalTime = 6)
    #Sleep until the subscribers are ready.
    time.sleep(0.200)
    try:
        while not rospy.core.is_shutdown():
            #ic.update()
            rospy.rostime.wallsleep(0.25)

    except KeyboardInterrupt:
        print("Shutting down")

    finally:
        ic.closeFiles()
        cv2.destroyAllWindows()
        print("Shutting down")



ntpClient = ntplib.NTPClient()

if __name__ == '__main__':
    rospy.init_node('data_recording_node')
    if rospy.has_param('/data_recording_launcher/subject_id') and  rospy.has_param('/data_recording_launcher/trial'):
            subject_id = rospy.get_param('/data_recording_launcher/subject_id')
            trial = rospy.get_param('/data_recording_launcher/trial')
    else:
            print("Subject Id or trial was not specified")
            raise KeyError

    print("Initializing node")
    main(subject_id, trial)
