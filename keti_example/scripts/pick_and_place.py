#! /usr/bin/env python3

import rospy
from dataclasses import dataclass
from keti_msgs.msg import RobotState
from keti_msgs.msg import VisionState
import actionlib
import keti_robot.msg
import keti_gripper.msg
import time
from keti_msgs.srv import *

robot_state = 0
current_joint = []
current_position = []
current_rotation = []
current_T_matrix = []
print_enable = False
robot_move_complete = False
robot_moving = False
gripper_move_complete = False
gripper_moving = False
state = 0
step = 0

@dataclass
class State:
    Idle = 0
    MoveWait = 1
    RobotMoveJ = 2
    RobotMoveL = 3
    RobotMoveB = 4
    GripperMoveGrip = 5
    GripperMoveRelease = 6
    Stop = 7

def RobotStateCallback(msg):
    global robot_state, current_joint, current_position, current_rotation, current_T_matrix, print_enable
    robot_state = msg.robot_state
    current_joint = msg.current_joint
    current_position = msg.current_position
    current_rotation = msg.current_rotation
    current_T_matrix = msg.current_T_matrix

    if print_enable is True:
        rospy.loginfo('robot_state : %d', robot_state)
        rospy.loginfo('current_joint : %f, %f, %f, %f, %f, %f', current_joint[0], current_joint[1], current_joint[2], current_joint[3], current_joint[4], current_joint[5])
        rospy.loginfo('current_position : %f, %f, %f', current_position[0], current_position[1], current_position[2])
        rospy.loginfo('current_rotation : %f, %f, %f', current_rotation[0], current_rotation[1], current_rotation[2])
        rospy.loginfo('current_T_matrix 1 : %f, %f, %f, %f', current_T_matrix[0], current_T_matrix[1], current_T_matrix[2], current_T_matrix[3])
        rospy.loginfo('current_T_matrix 2 : %f, %f, %f, %f', current_T_matrix[4], current_T_matrix[5], current_T_matrix[6], current_T_matrix[7])
        rospy.loginfo('current_T_matrix 3 : %f, %f, %f, %f', current_T_matrix[8], current_T_matrix[9], current_T_matrix[10], current_T_matrix[11])
        rospy.loginfo('current_T_matrix 4 : %f, %f, %f, %f', current_T_matrix[12], current_T_matrix[13], current_T_matrix[14], current_T_matrix[15])

def RobotMoveCompleteCallback(arg1, arg2):
    global robot_move_complete, state, State
    rospy.loginfo('Robot move complete')
    robot_move_complete = True
    state = State.Idle

def RobotMovingFeedbackCallback(feedback):
    global robot_moving, robot_move_complete
    state = feedback.sequence[-1]
    if state == 1:
        robot_moving = False
        rospy.loginfo("Robot don't move")
    elif state == 2:
        robot_moving = True
        robot_move_complete = False
        rospy.loginfo("Robot moving")

def RobotMovingActiveCallback():
    global robot_moving, robot_move_complete, state, State
    rospy.loginfo("Goal just went active")
    robot_moving = False
    robot_move_complete = False

    state = State.MoveWait

def GripperMoveCompleteCallback(arg1, arg2):
    global gripper_move_complete, state, State
    rospy.loginfo('Gripper move complete')
    gripper_move_complete = True
    state = State.Idle

def GripperMovingFeedbackCallback(feedback):
    global gripper_moving, gripper_move_complete
    rospy.loginfo("gripper status : %d", feedback.status[-1])
    rospy.loginfo("gripper width : %f", feedback.width[-1])
    gripper_move_complete = False
    gripper_moving = True

def GripperMovingActiveCallback():
    global gripper_moving, gripper_move_complete, state, State
    rospy.loginfo("Goal just went active")
    gripper_moving = False
    gripper_move_complete = False
    state = State.MoveWait

def VisionStateCallback(msg):
    global print_enable
    if print_enable:
        rospy.loginfo('vision_state : %d', msg.connected)

if __name__ == '__main__':
    rospy.init_node('keti_pick_and_place_sample_py_node', anonymous=True)
    rospy.Subscriber('/keti_robot_state', RobotState, RobotStateCallback, buff_size=1)
    rospy.Subscriber('/keti_vision_state', VisionState, VisionStateCallback, buff_size=1)
    rospy.wait_for_service('keti_vision_comm')
    srvVisionComm = rospy.ServiceProxy('keti_vision_comm', VisionComm)

    acRobot = actionlib.SimpleActionClient('robot_move_action',keti_robot.msg.RobotMoveAction)
    goalRobot = keti_robot.msg.RobotMoveActionGoal().goal

    acGripper = actionlib.SimpleActionClient('keti_gripper_node', keti_gripper.msg.GripperMoveAction)
    goalGripper = keti_gripper.msg.GripperMoveActionGoal().goal

    r = rospy.Rate(5)

    robot_move_complete = False
    robot_moving = False

    gripper_move_complete = False
    gripper_moving = False

    acRobot.wait_for_server()
    acGripper.wait_for_server()

    state = State.Idle

    # Move to init joint position
    init_joint = [0, 0, 1.57079, 0, 1.57079, 0]
    goalRobot.cmd = 1
    goalRobot.num = 1
    goalRobot.value = []
    for i in range(6):
        goalRobot.value.append(init_joint[i])
    acRobot.send_goal(goalRobot, active_cb=RobotMovingActiveCallback, feedback_cb=RobotMovingFeedbackCallback, done_cb=RobotMoveCompleteCallback)
    while state != State.MoveWait:
        time.sleep(0.001)
    while state != State.Idle:
        time.sleep(0.001)

    # Move to camera shooting position
    camera_pose = [0.850, -0.245, 0.700, 0, 3.14159, 0]
    goalRobot.cmd = 2
    goalRobot.num = 1
    goalRobot.value = []
    for i in range(6):
        goalRobot.value.append(camera_pose[i])
    acRobot.send_goal(goalRobot, active_cb=RobotMovingActiveCallback, feedback_cb=RobotMovingFeedbackCallback, done_cb=RobotMoveCompleteCallback)
    while state != State.MoveWait:
        time.sleep(0.001)
    while state != State.Idle:
        time.sleep(0.001)

    # Get camera data(communication with vision application)
    rospy.loginfo("Wait return vision module")
    visionComm = srvVisionComm.call()
    rospy.loginfo("vision state : %d", visionComm.connected)
    if len(visionComm.value) >= 3:
        rospy.loginfo("vision data : %f, %f, %f", visionComm.value[0], visionComm.value[1], visionComm.value[2])

    if visionComm.connected is False:
        rospy.logerr("Disconnected vision socket server")
        visionComm.value = []
        visionComm.value.append(0)
        visionComm.value.append(0)
        visionComm.value.append(0)

    # Move to pick position
    pick_pose = [0, 0, 0, 0, 0, 0]
    offset = [0.0818452352279836, -0.023331711769110135, 0.019284411871427095]
    
    pick_pose_wp1 = [0.85, -0.245, 0.35, 0, 3.14159, 0]
    pick_pose_wp2 = [0.931071, -0.267948, 0.35, 0, 3.14159, 0]
    pick_pose_wp3 = [0.931071, -0.267948, 0.22, 0, 3.14159, 0]

    goalRobot.cmd = 2
    goalRobot.num = 1
    goalRobot.value = []
    for i in range(6):
        goalRobot.value.append(pick_pose_wp1[i])
    acRobot.send_goal(goalRobot, active_cb=RobotMovingActiveCallback, feedback_cb=RobotMovingFeedbackCallback, done_cb=RobotMoveCompleteCallback)
    while state != State.MoveWait:
        time.sleep(0.001)
    while state != State.Idle:
        time.sleep(0.001)

    goalRobot.cmd = 2
    goalRobot.num = 1
    goalRobot.value = []
    for i in range(6):
        goalRobot.value.append(pick_pose_wp2[i])
    acRobot.send_goal(goalRobot, active_cb=RobotMovingActiveCallback, feedback_cb=RobotMovingFeedbackCallback, done_cb=RobotMoveCompleteCallback)
    while state != State.MoveWait:
        time.sleep(0.001)
    while state != State.Idle:
        time.sleep(0.001)

    goalRobot.cmd = 2
    goalRobot.num = 1
    goalRobot.value = []
    for i in range(6):
        goalRobot.value.append(pick_pose_wp3[i])
    acRobot.send_goal(goalRobot, active_cb=RobotMovingActiveCallback, feedback_cb=RobotMovingFeedbackCallback, done_cb=RobotMoveCompleteCallback)
    while state != State.MoveWait:
        time.sleep(0.001)
    while state != State.Idle:
        time.sleep(0.001)

    # Gripper grip
    goalGripper.cmd = 1
    goalGripper.width = 5
    goalGripper.force = 20
    goalGripper.speed = 10
    acGripper.send_goal(goalGripper, active_cb=GripperMovingActiveCallback, feedback_cb=GripperMovingFeedbackCallback, done_cb=GripperMoveCompleteCallback)
    while state != State.MoveWait:
        time.sleep(0.001)
    while state != State.Idle:
        time.sleep(0.001)

    # Move to place position
    place_pose_wp1 = [0.931071, -0.267948, 0.4, 0, 3.14159, 0]
    place_pose_wp2 = [0.971071, 0.307948, 0.4, 0, 3.14159, 0]
    place_pose_wp3 = [0.971071, 0.307948, 0.22, 0, 3.14159, 0]
    goalRobot.cmd = 3
    goalRobot.num = 3
    goalRobot.value = []
    for i in range(6):
        goalRobot.value.append(place_pose_wp1[i])
    for i in range(6):
        goalRobot.value.append(place_pose_wp2[i])
    for i in range(6):
        goalRobot.value.append(place_pose_wp3[i])
    acRobot.send_goal(goalRobot, active_cb=RobotMovingActiveCallback, feedback_cb=RobotMovingFeedbackCallback, done_cb=RobotMoveCompleteCallback)
    while state != State.MoveWait:
        time.sleep(0.001)
    while state != State.Idle:
        time.sleep(0.001)

    # Gripper release
    goalGripper.cmd = 2
    goalGripper.width = 39
    goalGripper.force = 0
    goalGripper.speed = 0
    acGripper.send_goal(goalGripper, active_cb=GripperMovingActiveCallback, feedback_cb=GripperMovingFeedbackCallback, done_cb=GripperMoveCompleteCallback)
    while state != State.MoveWait:
        time.sleep(0.001)
    while state != State.Idle:
        time.sleep(0.001)

    # Move to init position
    goalRobot.cmd = 2
    goalRobot.num = 1
    goalRobot.value = []
    for i in range(6):
        goalRobot.value.append(place_pose_wp2[i])
    acRobot.send_goal(goalRobot, active_cb=RobotMovingActiveCallback, feedback_cb=RobotMovingFeedbackCallback, done_cb=RobotMoveCompleteCallback)
    while state != State.MoveWait:
        time.sleep(0.001)
    while state != State.Idle:
        time.sleep(0.001)
    
    goalRobot.cmd = 1
    goalRobot.num = 1
    goalRobot.value = []
    for i in range(6):
        goalRobot.value.append(init_joint[i])
    acRobot.send_goal(goalRobot, active_cb=RobotMovingActiveCallback, feedback_cb=RobotMovingFeedbackCallback, done_cb=RobotMoveCompleteCallback)
    while state != State.MoveWait:
        time.sleep(0.001)
    while state != State.Idle:
        time.sleep(0.001)