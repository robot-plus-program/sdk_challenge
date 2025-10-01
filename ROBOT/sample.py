#!/usr/bin/env python3
import sys
sys.path.append("ROBOT_SDK/ketirobotsdk")
from sdk import *
from time import *
import threading
import math
sys.path.append("GRIPPER_SDK/include")
from zimmergripper import *

rob = Robot()

class State:
	Wait = 1
	Moving = 2

class Cmd:
	RecvRobotState = 1
	RecvGripperWidth = 2
	RobotMoveJ = 3
	RobotMoveL = 4
	RobotMoveB = 5
	GripperMoveGrip = 6
	GripperMoveRelease = 7

robot_connected = False
state = 0
cmd = 0
current_joint = []
current_T_matrix = []


gripper = KetiZimmer(f'{os.getcwd()}/GRIPPER_SDK/lib/libzimmergripper.so')

def key_input_func():
	global cmd
	key_value = 0

	while robot_connected is True:
		print("\n Enter character and press \"Enter\"")
		print(" 1 : Receive robot current state")
		print(" 2 : Receive gripper current width")
		print(" 3 : Robot move joint motion")
		print(" 4 : Robot move Cartesian motion")
		print(" 5 : Robot move Cartesian motion with blend")
		print(" 6 : Gripper move(grip)")
		print(" 7 : Gripper move(release)")

		key_value = input()

		if key_value == '1':
			cmd = Cmd.RecvRobotState
		elif key_value == '2':
			cmd = Cmd.RecvGripperWidth
		elif key_value == '3':
			cmd = Cmd.RobotMoveJ
		elif key_value == '4':
			cmd = Cmd.RobotMoveL
		elif key_value == '5':
			cmd = Cmd.RobotMoveB
		elif key_value == '6':
			cmd = Cmd.GripperMoveGrip
		elif key_value == '7':
			cmd = Cmd.GripperMoveRelease

		while cmd != 0:
			sleep(0.001)

def data_update_func():
	global robot_connected, state, cmd, current_joint, current_T_matrix
	while robot_connected is True:
		robotInfor = rob.RobotInfo()

		current_joint = [robotInfor.Jnt[0], robotInfor.Jnt[1], robotInfor.Jnt[2], robotInfor.Jnt[3], robotInfor.Jnt[4], robotInfor.Jnt[5]]

		current_T_matrix = [robotInfor.Mat[0], robotInfor.Mat[1], robotInfor.Mat[2], robotInfor.Mat[3],
                      robotInfor.Mat[4], robotInfor.Mat[5], robotInfor.Mat[6], robotInfor.Mat[7],
                      robotInfor.Mat[8], robotInfor.Mat[9], robotInfor.Mat[10], robotInfor.Mat[11],
                      robotInfor.Mat[12], robotInfor.Mat[13], robotInfor.Mat[14], robotInfor.Mat[15]]

		if robotInfor.State == 2:
			state = State.Moving
			cmd = 0
		elif robotInfor.State == 1:
			state = State.Wait

		sleep(0.01)

if __name__ == '__main__':
	setLibPath(f'{os.getcwd()}/ROBOT_SDK/ketirobotsdk/librobotsdk.so')
 
	rob.SetRobotConf(M1013, "192.168.137.50", 12345)
	robot_connected = rob.RobotConnect()
	
	gripper.Connect("192.168.137.254", 502)
	gripper_connected = gripper.IsConnected()
	print("wait...")
	if gripper_connected is True:
		gripper.Init()
  
	key_input_thraed = threading.Thread(target=key_input_func, daemon=True)
	key_input_thraed.start()
	data_update_thread = threading.Thread(target=data_update_func, daemon=True)
	data_update_thread.start()

	cmd_joint = [[-math.pi/2.0, 0.0, math.pi/2.0, 0.0, math.pi/2.0, 37.0*math.pi/180.0],
              [-math.pi/2.0, -30.0*math.pi/180.0, 120.0*math.pi/180.0, -math.pi/2.0, math.pi/2.0, -math.pi + 37.0*math.pi/180.0]]
 
	cmd_rot = [0.799254, 0.00104721, 0.600992,
				0.600992, -1.09254e-06, -0.799255,
				-0.000836329, 0.999999, -0.000630237]
	cmd_pos = [[-0.15761, -0.688601, 0.694795],
				[0.39412, -0.96386, 0.579953],
				[-0.346809, -0.96386, 0.579953], 
				[-0.346809, -0.492616, 0.901642],
				[0.333264, -0.492616, 0.901642]]
 
	cnt_joint = 1
	cnt_pose = 1

	rob.SetVelocity(10)
 
	try:
		while robot_connected is True:
			if state == State.Wait | True:
				if cmd == Cmd.RecvRobotState:
					robotInfor = rob.RobotInfo()
					print("current_state : {0}".format(robotInfor.State))
					print("current_joint : {0}".format(current_joint))
					print("current_T_matrix : ")
					print(current_T_matrix[0:4])
					print(current_T_matrix[4:8])
					print(current_T_matrix[8:12])
					print(current_T_matrix[12:16])
					cmd = 0
				elif cmd == Cmd.RecvGripperWidth:
					print("current width : {0}".format(gripper.CurPos()))
					cmd = 0
				elif cmd == Cmd.RobotMoveJ:
					print(cmd_joint[cnt_joint%2])
					rob.movej(cmd_joint[cnt_joint%2])
					cnt_joint = cnt_joint + 1
					cmd = 0
				elif cmd == Cmd.RobotMoveL:
					cmd_mat = [0]*16
					for i in range(0, 3):
						for j in range(0, 3):
							cmd_mat[i*4 + j] = cmd_rot[i*3 + j]
					cmd_mat[3] = cmd_pos[cnt_pose%5][0]
					cmd_mat[7] = cmd_pos[cnt_pose%5][1]
					cmd_mat[11] = cmd_pos[cnt_pose%5][2]
					cmd_mat[15] = 1
		
					print("cmd_mat : ")
					print(cmd_mat[0:4])
					print(cmd_mat[4:8])
					print(cmd_mat[8:12])
					print(cmd_mat[12:16])

					rob.movel(0, cmd_mat)
					cnt_pose = cnt_pose + 1
					cmd = 0
				elif cmd == Cmd.RobotMoveB:
					cmd_mat = [[0 for col in range(16)] for row in range(5)]
					for num in range(0, 5):
						for i in range(0, 3):
							for j in range(0, 3):
								cmd_mat[num][i*4 + j] = cmd_rot[i*3 + j]
						cmd_mat[num][3] = cmd_pos[num][0]
						cmd_mat[num][7] = cmd_pos[num][1]
						cmd_mat[num][11] = cmd_pos[num][2]
						cmd_mat[num][15] = 1

						print("cmd_mat {0}: ".format(num))
						print(cmd_mat[num][0:4])
						print(cmd_mat[num][4:8])
						print(cmd_mat[num][8:12])
						print(cmd_mat[num][12:16])
    
					rob.moveb(0, 0.02, 5, cmd_mat[0], cmd_mat[1], cmd_mat[2], cmd_mat[3], cmd_mat[4])
					cmd = 0
				elif cmd == Cmd.GripperMoveGrip:
					gripper.Grip()
					cmd = 0
				elif cmd == Cmd.GripperMoveRelease:
					gripper.Release()
					cmd = 0

			sleep(0.001)
	except KeyboardInterrupt:
		rob.RobotDisconnect()
		gripper.Disconnect()

	print("finish")