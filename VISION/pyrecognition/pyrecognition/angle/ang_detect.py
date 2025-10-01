import cv2
import numpy as np
from pyrecognition.angle.polar_signal import detect_polar,calculate_dif_angle
class angle_detector():
    def run(self,**kwargs):
        print(calculate_dif_angle(**kwargs))
        ret,X,Y,differ_angle,reference_r,ret_img=calculate_dif_angle(**kwargs)
        ret_dict={"ret":ret,
                  "reference_r":reference_r,
                  "differ_angle":differ_angle,
                  "X":X,
                  "Y":Y,
                  "ret_img":ret_img}
        return ret_dict