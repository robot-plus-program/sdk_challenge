#! /usr/bin/env python3

import rospy
import actionlib
import keti_gripper.msg
from keti_onrobot_2fg7 import *


class GripperMoveAction(object):
    feedback = keti_gripper.msg.GripperMoveFeedback()
    result = keti_gripper.msg.GripperMoveResult()

    def __init__(self, name, gripper):
        self.gripper = gripper

        self.action_name = name
        self._as = actionlib.SimpleActionServer(self.action_name, keti_gripper.msg.GripperMoveAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        r = rospy.Rate(10)
        success = True

        self.feedback.status = []
        self.feedback.width = []
        self.feedback.force = []
        self.feedback.grip = []

        rospy.loginfo('%s: Executing, creating gripper move sequence, cmd : %d, width : %f, force : %f, speed : %d' % (self.action_name, goal.cmd, goal.width, goal.force, goal.speed))

        if goal.cmd == 1:
            self.gripper.grip_ext(t_index=0, t_width=goal.width, n_force=goal.force, p_speed=goal.speed, f_wait=False)
        elif goal.cmd == 2:
            self.gripper.move(t_index=0, t_width=goal.width, f_wait=False)

        count = 0
        while True:
            r.sleep()
            
            if self._as.is_preempt_requested():
                rospy.loginfo('%s : Preempted' % self.action_name)
                self._as.set_preempted()
                success = False
                break

            self.feedback.status.append(self.gripper.getStatus(t_index=0))
            self.feedback.width.append(self.gripper.get_ext_width(t_index=0))
            self.feedback.force.append(self.gripper.get_force(t_index=0))
            self.feedback.grip.append(self.gripper.isGripped(t_index=0))
            self._as.publish_feedback(self.feedback)

            if self.gripper.isGripped(t_index=0) is True:
                break

            if self.gripper.getStatus(t_index=0) == 0:
                break

        if success:
            self.result.status = self.feedback.status
            rospy.loginfo('%s : Succeeded' % self.action_name)
            self._as.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node('keti_gripper_node')
    gripper = TWOFG(dev=Device(cb_ip='192.168.137.201'), t_index=0)
    server = GripperMoveAction(rospy.get_name(), gripper)
    rospy.spin()
