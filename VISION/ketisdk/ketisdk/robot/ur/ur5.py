import urx
import time
import math
import numpy as np
# from labjack_u3 import hU3_labjack
# suction 
import cv2

class hUR_robot():

    def __init__(self):

        self.l = 0.2
        self.v = 0.3
        self.a = 0.4

        self.r = []
        self.homeRot = [0.0, -90.0, -90.0, -90.0, 90.0, 90.0]
        self.initRot = [0.0, -75.0, -92.8, -72.19, 90.0, 91.73]
        self.placeRot = [29.0, -96.0, -81.0, -59.0, 88.0, 81.0]

        # self.placePos = [0.447, -0.360, 0.450, 25.0, 120.0, -17.0]
        self.placePos = [0.600, -0.550, 0.380, 0.0, 150.0, 0.0]
        self.placePos2 = [0.415, -0.550, 0.620, 0.0, 140.0, 0.0]
        self.placePos3 = [0.415, -0.550, 0.620, 0.0, 140.0, 0.0]

        self.initPos = [0.535, -0.111, 0.640, 0.0, 180.0, 0.0]
        self.initPos2 = [0.350, -0.0991, 0.750, 0.0, 150.0, 0.0]

        self.Tool = [0, 0, 245.0]
        # self.camToTcp = [-55.00, 40.00, 20.00]
        # self.camToTcp = [75.00, 43.00, -30.00]
        self.camToTcp = [85.00, 28.00, -10.00]

        self.ppx = 642.252
        self.ppy = 359.912
        self.focallength = 922.0

        self.open()

        self.suction = hU3_labjack()
        self.affine = []
        #self.setAffine()

        print('hUR :: init')

    def open(self, ipAddress="192.168.0.25"):
        robot = urx.Robot('192.168.0.25')

        self.r = robot

    def close(self):
        self.r.close()

    def move_to_point_wait(self, point):
        self.movel(point)

        while True:
            p = self.r.getl(wait=True)
            if abs(p[2] - point[2]) < 0.001:
                break

    def move_in_jointspace_wait(self, joints):
        self.moveJ(joints)

        while True:
            j = self.r.getj(wait=True)
            time.sleep(0.05)
            if max(abs(np.array(joints) - np.array(j))) < 0.001:
                break

    def moveHome(self):
        print('hUR :: moveHome')
        self.moveJ(self.homeRot)

    def moveInit(self):
        print('hUR :: moveInit')
        self.moveJ(self.initRot)

    def moveInitL(self):
        print('hUR :: moveInitL')
        self.move_to_point_wait(self.initPos)

    def moveInit2L(self):
        print('hUR :: moveInit2L')
        self.move_to_point_wait(self.initPos2)

    def movePlace(self):
        print('hUR :: movePlace')
        self.moveJ(self.placeRot)

    def movePlaceL(self):
        print('hUR :: movePlaceL')

        self.move_to_point_wait(self.placePos2)
        self.move_to_point_wait(self.placePos)

    def movePlace3L(self):
        self.move_to_point_wait(self.placePos3)

    def movePrePlaceL(self):
        print('hUR :: movePrePlaceL')
        temp = self.placePos.copy()
        temp[3] += 100
        temp[4] = 150
        self.move_to_point_wait(temp)


    # degree to radian
    def moveJ(self, joints):
        hjoints = joints.copy()

        for i in range(0, joints.__len__()):
            hjoints[i] = math.radians(joints[i])

        print('hUR :: movej ::' + str(hjoints))
        self.r.movej(hjoints, vel=self.v, acc=self.a, wait=False)

    # degree to radian
    def movel(self, pose):
        hPose = pose.copy()

        for i in range(0, 3):
            hPose[3 + i] = math.radians(pose[3 + i])

        print('hUR :: movel ::' + str(hPose))
        self.r.movel(hPose, vel=self.v, acc=self.a, wait=False)

    def get_point(self):
        point = self.r.getl()
        return point

    def get_joint(self):
        joint = self.r.getj(wait=True)
        return joint

    def stop_l(self, acc=0.3):
        self.r.stopl(acc=acc)

    def grip(self):
        try:
            self.suction.IoOn()
        except:
            print('GripIO Error ')

    def unGrip(self):
        try:
            self.suction.IoOff()
        except:
            print('UnGripIO Error ')


    '''
    def setAffine(self):
        points = [[163.0, 139.0, 478.0],
                  [1098.0, 146.0, 476.0],
                  [1141.0, 526.0, 447.0],
                  [147.0, 521.0, 447.0]
                  #[259.0, 174.0, 545.0],
                  #[929.0, 185.0, 544.0],
                  #[931.0, 523.0, 516.0],
                  #[231.0, 508.0, 518.0]
                  ]

        points_r = [[630.0,  184.0, 545.0],
                    [620.0, -281.0, 540.0],
                    [445.0, -294.0, 453.0],
                    [455.0,  177.0, 456.0]
                    #[656.0, 165.0, 486.0],
                    #[640.0, -227.0, 467.0],
                    #[458.0, -220.0, 395.0],
                    #[474.0, 177.0, 406.0]
                    ]

        points_c = []

        for i in range(0, 4):
            points_c.append([(points[i][0] - self.ppx) / self.focallength * points[i][2],
                             (points[i][1] - self.ppy) / self.focallength * points[i][2],
                             points[i][2]])

        src_r = np.array(points_c)
        dst_r = np.array(points_r)

        retval, self.affine, inliers = cv2.estimateAffine3D(src_r, dst_r)  # , rigid=True

        print(self.affine)
    '''

    ## 범용 X
    def movePixel(self, pixelXYZ, norm):
        print('pixelXYZ::', pixelXYZ)
        print('pixelNorm::', norm)

        pz = -pixelXYZ[2]
        px = float((float(pixelXYZ[0]) - self.ppx)) / self.focallength * float(pz)
        py = float((float(pixelXYZ[1]) - self.ppy)) / self.focallength * float(pz)

        rx = self.initPos[0] * 1000 + py + self.camToTcp[0] - self.Tool[2] * math.cos(math.radians(60))
        ry = self.initPos[1] * 1000 + px + self.camToTcp[1]
        rz = self.initPos[2] * 1000 + pz + self.camToTcp[2] + self.Tool[2] * math.sin(math.radians(60))

        #rx, ry, rz = Convert_Camera2Robot3d(px, py, float(pixelXYZ[2]), self.affine)

        # 수직 접근
        pPose = self.initPos.copy()
        pPose[0] = rx / 1000.0 / 1.2
        pPose[1] = ry / 1000.0
        pPose[2] = rz / 1000.0 * 1.2
        pPose[4] = 160
        self.move_to_point_wait(pPose)

        pPose[0] *= 1.28
        pPose[2] /= 1.25
        self.move_to_point_wait(pPose)

        pPose[0] /= 1.20
        pPose[2] *= 1.45

        #pPose[0] /= 1.4
        #pPose[2] *= 1.6

        if pPose[2] > 0.75:
            pPose[2] = 0.75

        time.sleep(0.2)

        try:
            self.grip()
        except:
            print('SuctionGripError')

        time.sleep(0.1)
        self.move_to_point_wait(pPose)

        pPose[0] /= 1.10
        pPose[2] *= 1.10

        if pPose[2] > 0.75:
            pPose[2] = 0.75

        self.move_to_point_wait(pPose)

        print('rx::', rx)
        print('ry::', ry)
        print('rz::', rz)


if __name__ == "__main__":
    r = hUR_robot()
    while True:
        r.moveHome()
        r.grip()
        time.sleep(10)
        r.moveInit()
        r.unGrip()
        time.sleep(10)






