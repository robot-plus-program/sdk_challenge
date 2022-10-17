#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <keti_robot/RobotMoveAction.h>
#include <keti_msgs/RobotState.h>

#include "robot/sdk.h"
#include "robot/robotconf.h"

class RobotMoveActionClass{
protected:
    actionlib::SimpleActionServer<keti_robot::RobotMoveAction> as;
    std::string action_name;
    keti_robot::RobotMoveFeedback feedback;
    keti_robot::RobotMoveResult result;

public:
    RobotMoveActionClass(std::string name, ros::NodeHandle nh);
    ~RobotMoveActionClass();

    void executeCB(const keti_robot::RobotMoveGoalConstPtr &goal);
    void RobotStateCallback(const keti_msgs::RobotState &msg);

    ros::Subscriber subRobotState;

private:
    int robot_state;
    double current_joint[6];
    double current_position[6];
    double current_rotation[6];
    double current_T_matrix[16];
};