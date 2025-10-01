import cv2
import numpy as np
import copy
import json
import math


if __name__=='__main__':
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    for index in range(0,8):
        mark=cv2.aruco.drawMarker(arucoDict,index,80*10)
        # write the generated ArUCo tag to disk and then display it to our
        # screen
        mark=cv2.cvtColor(mark,cv2.COLOR_GRAY2BGR)
        cv2.imwrite("aruco_DICT_6X6_50_"+str(index)+".png",mark)
        cv2.imshow("ArUCo Tag", mark)
        cv2.waitKey(0)
