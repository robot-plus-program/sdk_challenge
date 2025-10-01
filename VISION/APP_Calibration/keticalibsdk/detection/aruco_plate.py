import cv2
import numpy as np
import copy
import os
import json
class aruco_pattern:

    def __init__(self,path_intrinsic="../../data/cameraMatrix.npy"):
        self.cameraMatrix=np.load(path_intrinsic)
        self.distCoeffs=np.array([0,0,0,0,0]).astype(float)

    def Aruco_to_3dpoint(self,corn, markersize, cameraMatrix, distCoeffs):
        pose1 = cv2.aruco.estimatePoseSingleMarkers(corn, markersize, cameraMatrix, distCoeffs)
        rvec = pose1[0]
        tvec = pose1[1]
        mrv, jacobian = cv2.Rodrigues(rvec)
        markercorners = np.matmul(mrv, pose1[2].reshape(4, 3).T).T + np.tile(tvec.flatten(), [4, 1])
        markercorners = markercorners.reshape([4, 3])
        return markercorners

    def calibration_board(self,rgb):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        markerLength = 40  # Here, our measurement unit is centimetre.
        markerSeparation = 20  # Here, our measurement unit is centimetre.
        board = cv2.aruco.GridBoard((4, 2), markerLength, markerSeparation, aruco_dict)
        arucoParams = cv2.aruco.DetectorParameters()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(rgb, aruco_dict,
                                                              parameters=arucoParams)  # First, detect markers
        cv2.aruco.refineDetectedMarkers(rgb, board, corners, ids, rejectedImgPoints)

        if np.array([ids != None]).all():  # if there is at least one marker detected
            im_with_aruco_board = cv2.aruco.drawDetectedMarkers(rgb, corners, ids, (0, 255, 0))
            retval,rvec,tvec= cv2.aruco.estimatePoseBoard(corners, ids, board, self.cameraMatrix,self.distCoeffs,None,None)  # posture estimation from a diamond
            if retval != 0:
                cv2.drawFrameAxes(im_with_aruco_board, self.cameraMatrix, self.distCoeffs, rvec, tvec,100)  # axis length 100 can be changed according to your requirement
        else:
            im_with_aruco_board = rgb

        cv2.imshow("arucoboard", im_with_aruco_board)
        cv2.waitKey(100)
        print("  ")
        matrix=np.eye(4)
        matrix[0:3,0:3]=cv2.Rodrigues(rvec)[0]
        matrix[0:3,3]=tvec.T/1000

        if tvec[2]<0:
            print(matrix)
        return matrix,im_with_aruco_board

    def calibration_estimate3d(self, rgb):
            flag_show = False
            display_img = copy.deepcopy(rgb)

            arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

            arucoParams = cv2.aruco.DetectorParameters()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(display_img, arucoDict,
                                                               parameters=arucoParams)
            sortcorners = np.array(corners)[np.argsort(ids.flatten())]
            for index in range(0, ids.__len__()):
                point1 = corners[index][0][0]
                cv2.putText(display_img, "[ " + str(ids[index]) + " ]", point1.astype(int).tolist(), 1, 1, (0, 0, 255))
            if flag_show:
                cv2.imshow("test", display_img)
                cv2.waitKey(0)
            src = [[[-11, -12, 0], [-7, -12, 0], [-7, -8, 0], [-11, -8, 0]],  # 0
                   [[-5, -12, 0], [-1, -12, 0], [-1, -8, 0], [-5, -8, 0]],  # 1
                   [[1, -12, 0], [5, -12, 0], [5, -8, 0], [1, -8, 0]],  # 2
                   [[7, -12, 0], [11, -12, 0], [11, -8, 0], [7, -8, 0]],  # 3
                   [[-11, -6, 0], [-7, -6, 0], [-7, -2, 0], [-11, -2, 0]],  # 4
                   [[-5, -6, 0], [-1, -6, 0], [-1, -2, 0], [-5, -2, 0]],  # 5
                   [[1, -6, 0], [5, -6, 0], [5, -2, 0], [1, -2, 0]],  # 6
                   [[7, -6, 0], [11, -6, 0], [11, -2, 0], [7, -2, 0]]  # 7
                   ]

            src = np.array(src)
            src = np.array(src)[np.sort(ids.flatten())]/100
            distCoeffs = np.array([0, 0, 0, 0, 0])
            dst = []
            for cor in sortcorners:
                cor3d = self.Aruco_to_3dpoint(cor, 40, self.cameraMatrix, distCoeffs)
                dst.append(cor3d)

            pnp_dst = []
            for cor in sortcorners:
                pnp_dst = pnp_dst + cor[0].tolist()

            dst = np.array(dst).reshape(4 * corners.__len__(), 3)
            src = np.array(src).reshape(4 * corners.__len__(), 3)
            # _, rvec, tvec = cv2.solvePnP(object_points, corners, intrinsic_matrix, None)
            RT = cv2.estimateAffine3D(src, dst)
            R = RT[1][::, 0:3]
            T = RT[1][::, 3]

            anglevector = np.rad2deg(cv2.Rodrigues(R)[0])
            # show_log('      @R :', R)
            print('      @angle :' + " x: " + str(anglevector[0]) + " y: " + str(anglevector[1]) + " z: " + str(
                anglevector[2]))
            print('      @calibration ' + " x: " + str(T[0]) + " y: " + str(T[1]) + " z: " + str(T[2]))

            if flag_show:
                cv2.imshow("display_img",display_img)
                cv2.waitKey(100)
            matrix=np.eye(4)
            matrix[0:3, 0:3] = R
            matrix[0:3, 3]=T/1000
            im_with_aruco_board=copy.deepcopy(rgb)
            cv2.drawFrameAxes(im_with_aruco_board, self.cameraMatrix, self.distCoeffs, R, T,100)  # axis length 100 can be changed according to your requirement
            return matrix,im_with_aruco_board

    def calibration_pnp(self, rgb):
            flag_show = False
            display_img = copy.deepcopy(rgb)

            arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
            arucoParams = cv2.aruco.DetectorParameters()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(display_img, arucoDict,
                                                               parameters=arucoParams)
            sortcorners = np.array(corners)[np.argsort(ids.flatten())]
            for index in range(0, ids.__len__()):
                point1 = corners[index][0][0]
                cv2.putText(display_img, "[ " + str(ids[index]) + " ]", point1.astype(int).tolist(), 1, 1, (0, 0, 255))
            if flag_show:
                cv2.imshow("test", display_img)
                cv2.waitKey(0)
            object_points = [[[-11, -12, 0], [-7, -12, 0], [-7, -8, 0], [-11, -8, 0]],  # 0
                             [[-5, -12, 0], [-1, -12, 0], [-1, -8, 0], [-5, -8, 0]],  # 1
                             [[1, -12, 0], [5, -12, 0], [5, -8, 0], [1, -8, 0]],  # 2
                             [[7, -12, 0], [11, -12, 0], [11, -8, 0], [7, -8, 0]],  # 3
                             [[-11, -6, 0], [-7, -6, 0], [-7, -2, 0], [-11, -2, 0]],  # 4
                             [[-5, -6, 0], [-1, -6, 0], [-1, -2, 0], [-5, -2, 0]],  # 5
                             [[1, -6, 0], [5, -6, 0], [5, -2, 0], [1, -2, 0]],  # 6
                             [[7, -6, 0], [11, -6, 0], [11, -2, 0], [7, -2, 0]]  # 7
                             ]

            object_points = np.array(object_points)
            object_points = np.array(object_points)[np.sort(ids.flatten())] * 10 / 1000
            distCoeffs = np.array([0, 0, 0, 0, 0])
            dst = []
            for cor in sortcorners:
                cor3d = self.Aruco_to_3dpoint(cor, 40, self.cameraMatrix, distCoeffs)
                dst.append(cor3d)

            pnp_dst = []
            for cor in sortcorners:
                pnp_dst = pnp_dst + cor[0].tolist()

            dst = np.array(dst).reshape(4 * corners.__len__(), 3)
            object_points = np.array(object_points).reshape(4 * corners.__len__(), 3)
            _, rvec, tvec = cv2.solvePnP(object_points, np.array(pnp_dst), self.cameraMatrix, None)
            # _, rvec, tvec = cv2.solvePnP(object_points, corners, intrinsic_matrix, None)

            if flag_show:
                cv2.imshow("display_img",display_img)
                cv2.waitKey(100)
            matrix=np.eye(4)
            matrix[0:3, 0:3] = cv2.Rodrigues(rvec)[0]
            matrix[0:3, 3]=tvec.T/1000
            im_with_aruco_board=copy.deepcopy(rgb)
            cv2.drawFrameAxes(im_with_aruco_board, self.cameraMatrix, self.distCoeffs, rvec, tvec,100)  # axis length 100 can be changed according to your requirement
            return rvec, tvec,im_with_aruco_board
if __name__ == '__main__':
    aruco=aruco_pattern()
    print(aruco)

    datapath = "/mnt/83296afe-1ebf-459c-999e-755847e2077a/calibration/APP_Calibration-master/data/set1/2024_01_15_17_43_55/"
    files = os.listdir(datapath)
    list_filename = []
    for f in files:
        if "rgb" in f:
            list_filename.append(f[0:-7])
    list_filename = np.sort(list_filename)

    c2w=[]
    b2t=[]
    for filename in list_filename:
        path_rgb = datapath + filename+"rgb.png"
        path_depth = datapath + filename+"depth.png"
        path_json = datapath + filename+"UR10_Re.json"


        rgb=cv2.imread(path_rgb)
        depth = cv2.imread(path_depth,-1)
        st_json=open(path_json, "r")
        robot = json.load(st_json)
        # board_RT,im_with_aruco_board=aruco.calibration_board(rgb)
        RT,im_with_aruco_board=aruco.calibration_estimate3d(rgb)
        path_result = datapath + filename+"result.png"
        cv2.imwrite(path_result,im_with_aruco_board)
        c2w.append(RT.tolist())
        b2t.append(robot['matrix'])

    print(c2w.__len__())
    print(b2t.__len__())


