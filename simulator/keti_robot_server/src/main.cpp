#include <ros/ros.h>
#include "robot_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

static struct sigaction sigIntHandler;
static int sig = 0;

void my_handler(int s)
{
    if (sig == 0)
    {
        sig = s;
        exit(1);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_server_node");

    ros::NodeHandle nh;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    ros::AsyncSpinner spinner(3);
    spinner.start();

    RobotServer robotServerCmd(nh, 5000);
    robotServerCmd.start();

    RobotServer robotServerData(nh, 5001);
    robotServerData.start();

    RobotServer robotServerGripper(nh, 5002);
    robotServerGripper.start();

    ros::waitForShutdown();
}