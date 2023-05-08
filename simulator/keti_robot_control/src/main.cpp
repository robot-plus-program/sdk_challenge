#include <ros/ros.h>
#include "robot_control.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_control_node");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    RobotControl robotControl(nh, "robot_move_action", "gripper_move_action");
    robotControl.start();

    ros::waitForShutdown();

    return 0;
}