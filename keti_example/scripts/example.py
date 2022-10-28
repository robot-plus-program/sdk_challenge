#! /usr/bin/env python3

import rospy
import threading
from dataclasses import dataclass
from keti_msgs.msg import RobotState
import actionlib
import keti_robot.msg
import keti_gripper.msg

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

def key_input_func():
    global state, State
    key_value = 0
    
    while not rospy.is_shutdown():
        if state == State.Idle:
            rospy.loginfo("\n\n Enter character and press \"Enter\"\n 1 : Robot move joint motion\n 2 : Robot move cartesian motion\n 3 : Robot move cartesian motion with balend\n 4 : Gripper move(grip)\n 5 : Gripper move(release)\n 6 : Robot & Gripper stop\n")

            key_value = input()

            if key_value == '1':
                state = State.RobotMoveJ
            elif key_value == '2':
                state = State.RobotMoveL
            elif key_value == '3':
                state = State.RobotMoveB
            elif key_value == '4':
                state = State.GripperMoveGrip
            elif key_value == '5':
                state = State.GripperMoveRelease
            elif key_value == '6':
                state = State.Stop
            else:
                pass

if __name__ == '__main__':
    rospy.init_node('keti_example_py_node', anonymous=True)
    rospy.Subscriber('/keti_robot_state', RobotState, RobotStateCallback, buff_size=1)

    acRobot = actionlib.SimpleActionClient('robot_move_action',keti_robot.msg.RobotMoveAction)
    goalRobot = keti_robot.msg.RobotMoveActionGoal().goal

    r = rospy.Rate(5)

    cmd_joint = [[0, 0, 1.57079, 0, 0, 1.57079],
                [0, 0, 1.57079, 0, 1.57079, 0]]
    cmd_pose = [[0.8, 0.2, 0.6, 0, 3.14159, 0],
                [0.8, -0.2, 0.6, 0, 3.14159, 0],
                [0.8, -0.2, 0.3, 0, 3.14159, 0],
                [0.8, 0.2, 0.3, 0, 3.14159, 0],
                [0.8, 0.0, 0.6, 0, 3.14159, 0]]

    robot_move_complete = False
    robot_moving = False

    gripper_move_complete = False
    gripper_moving = False

    acRobot.wait_for_server()

    acGripper = actionlib.SimpleActionClient('keti_gripper_node', keti_gripper.msg.GripperMoveAction)
    goalGripper = keti_gripper.msg.GripperMoveActionGoal().goal

    acGripper.wait_for_server()

    state = State.Idle

    t = threading.Thread(target=key_input_func)
    t.daemon = True
    t.start()

    cnt_joint = 0
    cnt_pose = 0

    while not rospy.is_shutdown():
        r.sleep()

        if state == State.Idle:
            pass
        elif state == State.MoveWait:
            pass
        elif state == State.RobotMoveJ:
            goalRobot.cmd = 1
            goalRobot.num = 1
            goalRobot.value = []
            for i in range(6):
                goalRobot.value.append(cmd_joint[cnt_joint%2][i])
            acRobot.send_goal(goalRobot, active_cb=RobotMovingActiveCallback, feedback_cb=RobotMovingFeedbackCallback, done_cb=RobotMoveCompleteCallback)
            cnt_joint += 1
        elif state == State.RobotMoveL:
            goalRobot.cmd = 2
            goalRobot.num = 1
            goalRobot.value = []
            for i in range(6):
                goalRobot.value.append(cmd_pose[cnt_pose%5][i])
            acRobot.send_goal(goalRobot, active_cb=RobotMovingActiveCallback, feedback_cb=RobotMovingFeedbackCallback, done_cb=RobotMoveCompleteCallback)
            cnt_pose += 1
        elif state == State.RobotMoveB:
            goalRobot.cmd = 3
            goalRobot.num = 4
            goalRobot.value = []
            for j in range(goalRobot.num):
                for i in range(6):
                    goalRobot.value.append(cmd_pose[j + 1][i])
            acRobot.send_goal(goalRobot, active_cb=RobotMovingActiveCallback, feedback_cb=RobotMovingFeedbackCallback, done_cb=RobotMoveCompleteCallback)
        elif state == State.GripperMoveGrip:
            goalGripper.cmd = 1
            goalGripper.width = 5
            goalGripper.force = 20
            goalGripper.speed = 10
            acGripper.send_goal(goalGripper, active_cb=GripperMovingActiveCallback, feedback_cb=GripperMovingFeedbackCallback, done_cb=GripperMoveCompleteCallback)
        elif state == State.GripperMoveRelease:
            goalGripper.cmd = 2
            goalGripper.width = 39
            goalGripper.force = 0
            goalGripper.speed = 0
            acGripper.send_goal(goalGripper, active_cb=GripperMovingActiveCallback, feedback_cb=GripperMovingFeedbackCallback, done_cb=GripperMoveCompleteCallback)
        elif state == State.Stop:
            acRobot.cancel_all_goals()
            acGripper.cancel_all_goals()
            state = State.Idle
