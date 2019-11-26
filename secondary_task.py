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
from std_msgs.msg import String as rosString
import random 
import argparse
import json
import socket

class secondary_task_module:

	def __init__(self,totalTime =1, secondaryTime = 5, file= None, videoFileName = None):

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

		#Initialize recording variables
		self.is_recording_time = False
		self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
		self.out = cv2.VideoWriter(videoFileName, self.fourcc, 30.0, (720,480))
		self.frame = None
		
		#Published topics
		self.image_pub1 = rospy.Publisher("modified_display_left",Image, queue_size=5)
		self.image_pub2 = rospy.Publisher("modified_display_right",Image, queue_size=5)
		self.score_pub = rospy.Publisher("score_correctly", Joy, queue_size=5)
		# self.socket_client_pub = rospy.Publisher("socket_client", rosString, queue_size=5)
		
		self.image_pub1_compressed = rospy.Publisher("modified_display_left/compressed" ,CompressedImage, queue_size=5)
		self.image_pub2_compressed = rospy.Publisher("modified_display_right/compressed",CompressedImage, queue_size=5)
		self.bridge = CvBridge()
		
		#Subscribed topics
		#Topic to Record Video
		self.camera_stream_subs = rospy.Subscriber("/modified_display_left/", Image, self.video_recording_callback,  queue_size = 1)
		self.image_sub_left  = rospy.Subscriber("/pu_dvrk_cam/left/inverted", Image, self.left_callback)
		self.image_sub_right = rospy.Subscriber("/pu_dvrk_cam/right/inverted", Image, self.right_callback)
		self.camera_pedal_sub = rospy.Subscriber("/dvrk/footpedals/bicoag", Joy, self.pedal_callback)
		self.score_sub = rospy.Subscriber("score_correctly",Joy, self.score_callback)
		# self.socket_client_sub = rospy.Subscriber("socket_client", rosString, self.socket_callback)

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

		#File to write timestamps
		self.file = file


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

				#Stop recording
				self.is_recording_time = False
				self.out.release()

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
		
		#Blur the image if the secondary task is on
		# if  not self.turnSecondaryTask:
		# 	#cv_image = cv2.filter2D(cv_image,-1,self.smoothingKernel)
		# 	self.timerStr = ""

		#Modify Image
		if not self.startProcedure or self.stopProcedure:
			overlay = cv_image.copy()
			
			cv2.putText(overlay, self.message,(10+misalignment, 30), cv2.FONT_HERSHEY_SIMPLEX, self.fontSize, (0, 0, 255), 3)
			cv2.putText(overlay, self.timerStr+"  "+self.scoreStr,(10+misalignment, 75), cv2.FONT_HERSHEY_SIMPLEX, self.fontSize, (0, 0, 255), 3)
			
			if self.notifyUser:
				color = self.notificationColor
				cv2.rectangle(overlay, (660, 0), (720,60), color, -1)

			cv2.addWeighted(overlay, self.alpha, cv_image, 1 - self.alpha, 0, cv_image)

		#Publish modified Image
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
		if self.startProcedure and data.buttons[0] and self.turnSecondaryTask:
			
			pedalTimestamp = data.header.stamp.secs + data.header.stamp.nsecs *10**(-9)
			secondsCounter = int((pedalTimestamp - self.initTime))
			secondsFirstDigit = secondsCounter % 10 
			
			if any([secondsFirstDigit == x for x in self.target]):
				self.score += 2
				self.scoreStr = "Score: {:3d}".format(self.score)
				data.header.frame_id = "right"
				self.score_pub.publish(data)
				self.correct += 1
				
			else:
				self.score -= 2
				self.scoreStr = "Score: {:3d}".format(self.score)
				data.header.frame_id = "wrong"
				self.score_pub.publish(data)
				self.incorrect += 1

		elif not self.startProcedure and data.buttons[0]:
			self.is_recording_time = True
			self.message  = "Configuring procedure ..."

			#Communicate to NTP server and write header to timestamp file.
			ntp,localTime  = ntpClient.request('europe.pool.ntp.org', version=3),time.time()
			self.file.write("Initial computer time: {:.9f}\n".format(localTime))
			self.file.write("Initial NTP time:      {:.9f}\n".format(ntp.tx_time))
			self.file.write("NTP offset:            {:.9f}\n".format(ntp.offset))
			self.file.write("##DATA##\n")
			self.file.write("timeStamp secondary_task_status\n")

			self.message  = "Starting procedure in 3..."
			time.sleep(1)
			self.message  = "Starting procedure in 2..."
			time.sleep(1)
			self.message  = "Starting procedure in 1..."
			time.sleep(1)
			self.initTime = time.time()
			
			#Writing to file
			secondaryTaskStatus = "started"
			self.file.write("{:.9f} {}\n".format(self.initTime, secondaryTaskStatus))
			self.file.flush()

			self.startProcedure = True
							

	def score_callback(self,data):
		
		self.notificationColor = (0,255,0) if data.header.frame_id == "right" else (0,0,255)
		
		if data.buttons[0]:
			self.notifyUser = True
			time.sleep(0.4)
			self.notifyUser = False

	def video_recording_callback(self, videoFrame):
		tempFrame = None
		cv_image = self.bridge.imgmsg_to_cv2(videoFrame,"bgr8")
		if self.is_recording_time:	
			print(self.is_recording_time)
			self.out.write(cv_image)
			
			# tempFrame = np.fromstring(videoFrame.data, np.uint8)
	  #       image_np = cv2.imdecode(tempFrame, cv2.IMREAD_COLOR)
	  #       # image_np=np.zeros((600,600,3))
	  # #       cv2.imshow('cv_img', image_np)
	  #       cv2.waitKey(2)

	# def init_socket_connection(self,ip, port):
	# 	publisher = self.socket_client_pub
	# 	message = {"command":"connect","arg":{"ip":ip,"port":port} }
	# 	message = json.dumps(message)
	# 	publisher.publish(message)

	# def close_socket_connection(self,ip, port):
	# 	publisher = self.socket_client_pub
	# 	message = {"command":"close","arg":{}}
	# 	message = json.dumps(message)
	# 	publisher.publish(message)


	# def socket_callback(self,data):
	# 	instruction =  json.loads(data.data)
	# 	if instruction['command'] == "connect":
	# 		ip = instruction['arg']['ip']
	# 		port = int(instruction['arg']['port'])

	# 		print("connecting to socket ...")
	# 		self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	# 		self.client.connect((ip, port))
	# 		print("successful connection")
	# 		self.client.send(b"I am CLIENT\n")
	# 		print("sent and waiting for response")
	# 		from_server = self.client.recv(4096)
	# 		print(from_server)

	# 	elif instruction['command'] == "close":
	# 		print("close socket")
	# 		self.client.close()
	# 	else:
	# 		pass

def main(userId, trialId):
	#Press enter to init procedure
	#raw_input("Press any key to start the data collection")
	print("Starting Da vinci Operation...")

	#Create File to save Timestamps
	timeStamp = createTimeStamp()
	fileName = "dvrk_collection_{:s}_{:s}.txt".format(str(userId), str(trialId))
	videoFileName = "dvrk_collection_{:s}_{:s}.avi".format(str(userId), str(trialId))
	file = open("/home/juan/DVRK_secondary_task_data/" + timeStamp + '_' + fileName,'w')
	completeVideoFileName  = "/home/juan/DVRK_secondary_task_data/" + timeStamp + '_' + videoFileName

	#Communicate to NTP server and write header to timestamp file.
	# ntp,localTime  = ntpClient.request('europe.pool.ntp.org', version=3),time.time()
	# file.write("Initial computer time: {:.9f}\n".format(localTime))
	# file.write("Initial NTP time:      {:.9f}\n".format(ntp.tx_time))
	# file.write("NTP offset:            {:.9f}\n".format(ntp.offset))
	# file.write("##DATA##\n")
	# file.write("timeStamp secondary_task_status\n")
	
	ic = secondary_task_module(file=file, secondaryTime = 60, totalTime = 5, videoFileName =completeVideoFileName)
	#Sleep until the subscribers are ready.
	time.sleep(0.050)
	# ic.init_socket_connection('127.0.0.1', '8080')
	
	try:
		while not rospy.core.is_shutdown():
			ic.update()
			rospy.rostime.wallsleep(0.25)
			

	except KeyboardInterrupt:
		print("Shutting down")

	finally:
		file.close()
		cv2.destroyAllWindows()
		print("Shutting down")


def createTimeStamp():
	ts = time.time()
	return datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H.%M.%S_')

#Global Variables
ntpClient = ntplib.NTPClient()

if __name__ == '__main__':
	
	# import sys
	# sys.path.insert('/home/juan/DVRK_secondary_task_data/')

	rospy.init_node('secondary_task')
	
	if rospy.has_param('/secondary_task_launcher/subject_id') and  rospy.has_param('/secondary_task_launcher/trial'):
		subject_id = rospy.get_param('/secondary_task_launcher/subject_id')
		trial = rospy.get_param('/secondary_task_launcher/trial')
	else:
		print("Subject Id or trial was not specified")
		raise KeyError

	print("Initializing node")
	main(subject_id, trial)