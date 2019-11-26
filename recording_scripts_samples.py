#!/usr/bin/env python



##If you use compressed topic use the following
# def callback(self, ros_data):

#         #### direct conversion to CV2 ####
#         np_arr = np.fromstring(ros_data.data, np.uint8)
#         image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
#         out.write(image_np)
#         cv2.imshow('cv_img', image_np)
#         cv2.waitKey(2)


##If you use raw topic use the following callback
# def video_recording_callback(self, videoFrame):
# 		tempFrame = None
# 		cv_image = self.bridge.imgmsg_to_cv2(videoFrame,"bgr8")
# 		if self.is_recording_time:	
# 			print(self.is_recording_time)
# 			self.out.write(cv_image)


import sys, time

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage

class image_capture:

    def __init__(self):
       
        # subscribed Topic
        topicName = "/modified_display_left/compressed"
        # topicName = "/juan_cam/right/image_raw/compressed"
        self.subscriber = rospy.Subscriber(topicName,
            CompressedImage, self.callback,  queue_size = 1)
    
    def callback(self, ros_data):

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        out.write(image_np)
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)

       

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_capture()
    rospy.init_node('image_capture', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    
    print("Close system")
    cv2.destroyAllWindows()
    out.release()

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('./testVideo.avi',fourcc, 30.0, (720,480))

if __name__ == '__main__':
    main(sys.argv)