#!/usr/bin/env python3
import sys
sys.path.append("../ketirobotsdk")
from sdk import *
from time import *
import threading
import math
sys.path.append("../gripper")
from zimmergripper import KetiZimmer
from math3d import Orientation
import numpy as np
import copy

if __name__ == '__main__':
	setLibPath(f'{os.getcwd()}/../ketirobotsdk/librobotsdk.so')
	rob = Robot()
	rob.SetRobotConf(RB10, '127.0.0.1', 5000)
	robot_connected = rob.RobotConnect()
	
	init_joint = [-math.pi/2.0, 0.0, math.pi/2.0, 0.0, math.pi/2.0, 37.0*math.pi/180.0]

	rob.SetVelocity(10)
 
	# example 1
	rob.movej(init_joint)
	sleep(5)
	
	robotInfor = rob.RobotInfo()

	cur_pose = np.array(robotInfor.Mat).reshape(4,4)
	print('cur_pose : ', cur_pose)

	Orient1 = Orientation()
	Orient1.rotate_yt(np.deg2rad(90))
	romat = Orient1.matrix
	
	ro = copy.deepcopy(cur_pose[0:3, 0:3])
	point_pose = copy.deepcopy(cur_pose)
	result_ro = np.matmul(ro, romat)

	point_pose[0:3, 0:3] = result_ro
	point_pose = point_pose.flatten() # 2d array -> to 1d array
	print('point_pose : ', point_pose)

	rob.movel(0, point_pose)
	sleep(5)

	# example 2
	rob.movej(init_joint)
	sleep(5)
	
	robotInfor = rob.RobotInfo()

	cur_pose = np.array(robotInfor.Mat).reshape(4,4)
	print('cur_pose : ', cur_pose)

	Orient2 = Orientation()
	Orient2.rotate_yt(np.deg2rad(-37))
	Orient2.rotate_xt(np.deg2rad(-90))
	Orient2.rotate_yt(np.deg2rad(37))
	romat = Orient2.matrix
	
	ro = copy.deepcopy(cur_pose[0:3, 0:3])
	point_pose = copy.deepcopy(cur_pose)
	result_ro = np.matmul(ro, romat)

	point_pose[0:3, 0:3] = result_ro
	point_pose = point_pose.flatten() # 2d array -> to 1d array
	print('point_pose : ', point_pose)

	rob.movel(0, point_pose)
	sleep(5)
	
	# example 2
	rob.movej(init_joint)
	sleep(5)
	
	robotInfor = rob.RobotInfo()

	cur_pose = np.array(robotInfor.Mat).reshape(4,4)
	print('cur_pose : ', cur_pose)

	Orient3 = Orientation()
	Orient3.rotate_yt(np.deg2rad(-37))
	Orient3.rotate_zt(np.deg2rad(-90))
	Orient3.rotate_yt(np.deg2rad(37))
	romat = Orient3.matrix
	
	ro = copy.deepcopy(cur_pose[0:3, 0:3])
	point_pose = copy.deepcopy(cur_pose)
	result_ro = np.matmul(ro, romat)

	point_pose[0:3, 0:3] = result_ro
	point_pose = point_pose.flatten()
	print('point_pose : ', point_pose)

	rob.movel(0, point_pose)
	sleep(5)
 
	rob.RobotDisconnect()

	print("Good bye!!")