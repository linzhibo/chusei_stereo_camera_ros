#! /usr/bin/env python

import numpy as np 
import cv2
import os
import rospy
import shlex
import subprocess
import yaml
from std_msgs.msg import String 
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import thread
# from pub_info import *
class LittleStereoCam():
    def __init__(self):
        rospy.init_node("little_stereo_camera", anonymous = True)
        self.bridge = CvBridge() 

        self.left_image_pub = rospy.Publisher("stereo/left/image_raw", Image, queue_size=1)
        self.left_image_info_pub = rospy.Publisher("stereo/left/camera_info", CameraInfo, queue_size=1)
        self.right_image_pub = rospy.Publisher("stereo/right/image_raw", Image, queue_size=1)
        self.right_image_info_pub = rospy.Publisher("stereo/right/camera_info", CameraInfo, queue_size=1)

        self.camera_info = CameraInfo()
        self.msg_header = Header()
        self.ros_image = Image()
        self.ros_image.height = 240
        self.ros_image.width = 320
        
        cam_id= rospy.get_param("cam_id", 1)
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.left_yaml_file = dir_path+"/../calibration/left.yaml"
        self.right_yaml_file = dir_path+"/../calibration/right.yaml"
        self.cam=cv2.VideoCapture(cam_id)
        cam_mode_dict={
            'LEFT_EYE_MODE':1,
            'RIGHT_EYE_MODE':2,
            'RED_BLUE_MODE':3,
            'BINOCULAR_MODE':4,
            }
        cam_mode=cam_mode_dict['BINOCULAR_MODE']
        command_list=[
        "uvcdynctrl -d /dev/video{cam_id} -S 6:8 '(LE)0x50ff'",
        "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x00f6'",
        "uvcdynctrl -d /dev/video{cam_id} -S 6:8 '(LE)0x2500'",
        "uvcdynctrl -d /dev/video{cam_id} -S 6:8 '(LE)0x5ffe'",
        "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0003'",
        "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0002'",
        "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0012'",
        "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0004'",
        "uvcdynctrl -d /dev/video{cam_id} -S 6:8 '(LE)0x76c3'",
        "uvcdynctrl -d /dev/video{cam_id} -S 6:10 '(LE)0x0{cam_mode}00'",
        ]
        for command in command_list:
            subprocess.Popen(shlex.split(command.format(cam_id=cam_id,cam_mode=cam_mode)))

    def pub_image(self, publisher, image, header):
        try:
            self.ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.ros_image.header = header
            publisher.publish(self.ros_image)
        
        except CvBridgeError as e:
            print(e)

    def yaml_to_camera_info(self,yaml_file):
        with open(yaml_file, "r") as f :
            calib_data = yaml.load(f)
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg


    def run(self):
        # left_cam_info = self.yaml_to_camera_info(self.left_yaml_file)
        # right_cam_info = self.yaml_to_camera_info(self.right_yaml_file)
        left_cam_info = CameraInfo()
        left_cam_info.width = 320
        left_cam_info.height = 240
        left_cam_info.K = [428.742281, 0.000000, 156.350849, 0.000000, 428.925552, 128.154488, 0.000000, 0.000000, 1.000000]
        left_cam_info.D = [0.145034, -0.494188, 0.011364, -0.015166, 0.000000]
        left_cam_info.R = [0.964297, 0.000775, 0.264822, -0.002139, 0.999986, 0.004861, -0.264815, -0.005254, 0.964285]
        left_cam_info.P = [486.886495, 0.000000, 2.254309, 0.000000, 0.000000, 486.886495, 121.254906, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
        left_cam_info.distortion_model = 'plumb_bob'

        right_cam_info = CameraInfo()
        right_cam_info.width = 320
        right_cam_info.height = 240
        right_cam_info.K = [416.679011, 0.000000, 164.113939, 0.000000, 416.632961, 113.014961, 0.000000, 0.000000, 1.000000]
        right_cam_info.D = [0.101117, -0.318121, -0.004195, 0.003498, 0.000000]
        right_cam_info.R = [0.936555, 0.004652, 0.350489, -0.002834, 0.999980, -0.005699, -0.350508, 0.004344, 0.936550]
        right_cam_info.P = [486.886495, 0.000000, 2.254309, -88.273173, 0.000000, 486.886495, 121.254906, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
        right_cam_info.distortion_model = 'plumb_bob'

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ret,frame=self.cam.read()
            if not ret:
                print('[ERROR]: frame error')
                break
            expand_frame=cv2.resize(frame,None,fx=1,fy=0.5)
            left_image = expand_frame[0:240,0:320]
            right_image = expand_frame[0:240,320:640]
            

            self.msg_header.frame_id = 'stereo_image'
            self.msg_header.stamp = rospy.Time.now()
            left_cam_info.header = self.msg_header
            right_cam_info.header = self.msg_header
            self.left_image_info_pub.publish(left_cam_info)
            self.right_image_info_pub.publish(right_cam_info)
            # self.pub_image(self.left_image_pub, left_image, self.msg_header )
            # self.pub_image(self.right_image_pub, right_image, self.msg_header )

            try:
                thread.start_new_thread( self.pub_image, (self.left_image_pub, left_image, self.msg_header, ))
                thread.start_new_thread( self.pub_image, (self.right_image_pub, right_image, self.msg_header, ))
            except:
                print("[ERROR]: unable to start thread")
            rate.sleep()

if __name__ == '__main__':
	lsc = LittleStereoCam()
	try:
		lsc.run()
	except rospy.ROSInterruptException: 
		pass

    