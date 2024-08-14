#!/usr/bin/env python3
import sys, os
sys.path.append("../gripper")
from zimmergripper import KetiZimmer

if __name__ == '__main__':
	gripper = KetiZimmer(f'{os.getcwd()}/../gripper/libzimmergripper.so')
	gripper.Connect('127.0.0.1', 5002)
	gripper_connected = gripper.IsAlive()
	
	if gripper_connected is True:
		gripper.Init()
  
	gripper.Release()
    
	gripper.Move(300)
	print(gripper.CurPos())

	gripper.Move(1000)
	print(gripper.CurPos())

	gripper.Move(1500)
	print(gripper.CurPos())

	gripper.Move(2000)
	print(gripper.CurPos())

	gripper.Move(2500)
	print(gripper.CurPos())

	gripper.Move(3825)
	print(gripper.CurPos())

	gripper.Grip()
	print(gripper.CurPos())

	print("good bye !!!")