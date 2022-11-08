#!/usr/bin/env python3

# Import the necessary libraries
import rospy  # Python library for ROS
from sensor_msgs.msg import Image  # Image is the message type
from sensor_msgs.msg import CompressedImage  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import copy
import time

def mouse_event(event, x, y, flags, param):
    global gx, gy
    global display_img
    global current_depth

    if event == cv2.EVENT_FLAG_LBUTTON:
        Xc = x
        Yc = y
        Zc=current_depth[y,x]
        print("2D points", Xc, Yc)
        print ("depth",Zc)
        # point_x=(Xc-ppx)/fx*Zc
        # point_y=(Yc-ppx)/fy*Zc
        # point_z=Zc
        # print ("3D point ",point_x,point_y,point_z)

def colorImagCallback(data):
    global current_frame
    
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    
    rospy.loginfo("receiving video frame")
    
    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data, data.encoding)
    
def depthImagCallback(data):
    global current_depth
    
    # Used to convert between ROS and OpenCV images
    # br = CvBridge()
    
    # Convert ROS Image message to OpenCV image
    current_depth = data

if __name__ == '__main__':
    global current_frame
    
    rospy.init_node('video_py', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, colorImagCallback)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw/compressedDepth', CompressedImage, depthImagCallback)
    cv2.namedWindow('result')
    cv2.setMouseCallback('result', mouse_event)

    while not rospy.is_shutdown():
        time.sleep(3)
        show_frame = copy.deepcopy(current_frame)
        # Display image
        cv2.imshow('result', show_frame)

        cv2.waitKey(1)
