import sys
import numpy as np
import json
try:
    import cv2
except:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    import cv2

def print_grips(grips):
    print("X,Y,Z", grips[0:3], end=" / ")
    print("box size", grips[3:5], end=" / ")
    print("angle", grips[5], end=" / ")
    print("left point", grips[6:8], end=" / ")
    print("right point", grips[8:10], end=" / ")
    print("score", grips[10])

if __name__=='__main__':
    print("Demo GraspPoint")
    print("====================")
    rgb_data = cv2.imread("../data/Grasp_RGB.png")
    depth_data = cv2.imread("../data/Grasp_depth.png", -1)
    ppx, ppy, fx, fy = np.load("intr_param.npy")
    print("====================")
    filename="../data/Result_Data.json"
    with open(filename) as json_file:
        json_data = json.load(json_file)
        grips=np.array(json_data["grip"])
        result_image=np.array(json_data['im']).astype(np.uint8)
        best_index=json_data['best_ind']
        best_n_inds=np.array(json_data['best_n_inds'])
        best_grip=np.array(json_data['best'])[0]
    print("best_index", best_index)
    print_grips(best_grip)
    print("====================")
    print("best_n_inds", best_n_inds.size)
    best_n_grips=grips[best_n_inds]
    print("best top n grips")
    index=0
    for cur_grip in best_n_grips:
        print("#",str(index)," : ", end=" / ")
        print_grips(cur_grip)
        index+=1
    print("====================")
    cv2.imshow("result_image",result_image)
    cv2.waitKey(0)

    # Mission
    # 상위 5개의 grips을 원본 rgb_data에 표시하기
    # cv2.line 함수 이용