#pragma once

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <keti_robot_control/RobotState.h>
#include <keti_robot_control/GripperState.h>
#include <keti_robot_control/RobotMoveAction.h>
#include <keti_robot_control/GripperMoveAction.h>

#include <moveit_msgs/ExecuteTrajectoryAction.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sstream>
#include <stdio.h>
#include <pthread.h>
#include <string.h>

#include <inttypes.h>
#include <stdint.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>

#include "robot_server_typedef.h"

const int MAXCONNECTIONS = 5;
const int MAXWAITBUFSIZE = 4096;
const int MAXRECEIVEBUFSIZE = 4096;
const int MAXSENDBUFSIZE = 120*8 + 4;

const float RAD2DEG = 180.0/M_PI;
const float DEG2RAD = M_PI/180.0;

class RobotServer{
public:
    RobotServer(ros::NodeHandle nh, int server_port);
    ~RobotServer();

    void start();

private:
    int port;
    bool connected;
    int listenSockFD, clientSockFD;
    sockaddr_in server_addr, client_addr;
    char bufWait[MAXWAITBUFSIZE];
    char *ptrRecvBufIndx;
    unsigned char bufRecv[MAXRECEIVEBUFSIZE];
    std::string strRecv;
    unsigned char bufSend[MAXSENDBUFSIZE];

    void RobotStateCallback(const keti_robot_control::RobotState &msg);
    void GripperStateCallback(const keti_robot_control::GripperState &msg);
    void RobotMoveStateCallback(const moveit_msgs::ExecuteTrajectoryActionFeedback &feedback);

    RobotState robotState;
    GripperState gripperState;

    systemSTAT systemStat;
    RobotCmd robotCmd;

    pthread_t robot_thread, cmd_thread, data_thread, gripper_thread;
    static void* robot_func(void *arg);
    static void* cmd_func(void *arg);
    static void* data_func(void *arg);
    static void* gripper_func(void *arg);

    void initSocket();
    void connectSocket();

    // void cmd_func();
    // void data_func();
    // void robot_func();
    // void gripper_func();

    void RobotMoveResultCallback(const actionlib::SimpleClientGoalState &state, const keti_robot_control::RobotMoveResultConstPtr &result);
    void RobotMoveFeedbackCallback(const keti_robot_control::RobotMoveFeedbackConstPtr &feedback);
    void RobotMoveActiveCallback();

    void GripperMoveResultCallback(const actionlib::SimpleClientGoalState &state, const keti_robot_control::GripperMoveResultConstPtr &result);
    void GripperMoveFeedbackCallback(const keti_robot_control::GripperMoveFeedbackConstPtr &feedback);
    void GripperMoveActiveCallback();

protected:
    actionlib::SimpleActionClient<keti_robot_control::RobotMoveAction> *acRobot;
    actionlib::SimpleActionClient<keti_robot_control::GripperMoveAction> *acGripper;
    keti_robot_control::RobotMoveGoal goalRobot;
    keti_robot_control::GripperMoveGoal goalGripper;

    ros::Subscriber subRobotState;
    ros::Subscriber subGripperState;
    ros::Subscriber subRobotMoveState;
};