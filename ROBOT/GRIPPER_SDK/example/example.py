#!/usr/bin/env python3

import os, sys
sys.path.append(f"{sys.argv[1]}")
from zimmergripper import *

gripper = KetiZimmer(f'{sys.argv[2]}')
gripper.Connect('192.168.137.254', 502)
gripper.Init()

gripper.Move(1500)
gripper.Grip()
print(gripper.CurPos())


gripper.Release()
print(gripper.CurPos())



