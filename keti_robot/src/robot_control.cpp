#include <ros/ros.h>
#include <keti_msgs/RobotState.h>

#include "robot_move_action.h"

static sdk_info robotInfor;
static RConf Conf;

void RobotStateCallback(const keti_msgs::RobotState &msg){
    ROS_INFO("robot_state : %d", msg.robot_state);
    ROS_INFO("current joint : %f, %f, %f, %f, %f, %f", msg.current_joint[0], msg.current_joint[1], msg.current_joint[2], msg.current_joint[3], msg.current_joint[4], msg.current_joint[5]);
    ROS_INFO("current position : %f, %f, %f", msg.current_position[0], msg.current_position[1], msg.current_position[2]);
    ROS_INFO("current rotation : %f, %f, %f", msg.current_rotation[0], msg.current_rotation[1], msg.current_rotation[2]);
    ROS_INFO("current pose 1: %f, %f, %f, %f", msg.current_T_matrix[0], msg.current_T_matrix[1], msg.current_T_matrix[2], msg.current_T_matrix[3]);
    ROS_INFO("current pose 2: %f, %f, %f, %f", msg.current_T_matrix[4], msg.current_T_matrix[5], msg.current_T_matrix[6], msg.current_T_matrix[7]);
    ROS_INFO("current pose 3: %f, %f, %f, %f", msg.current_T_matrix[8], msg.current_T_matrix[9], msg.current_T_matrix[10], msg.current_T_matrix[11]);
    ROS_INFO("current pose 4: %f, %f, %f, %f", msg.current_T_matrix[12], msg.current_T_matrix[13], msg.current_T_matrix[14], msg.current_T_matrix[15]);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_control_node");

	ros::NodeHandle nh;
    ros::Publisher pubRobotState = nh.advertise<keti_msgs::RobotState>("keti_robot_state",1);

    std::string robot_ip;
    nh.getParam("robot_ip", robot_ip);
    ROS_INFO("robot_ip : %s", robot_ip.c_str());

    RobotChange(M1013, robot_ip.c_str(), 1013);
    RobotConnect();

    ros::Rate loop_rate(100);

    keti_msgs::RobotState msg;
    msg.robot_state = 0;
    memset(msg.current_T_matrix.data(), 0, sizeof(double)*16);
    memset(msg.current_joint.data(), 0, sizeof(double)*6);
    memset(msg.current_position.data(), 0, sizeof(double)*6);
    memset(msg.current_rotation.data(), 0, sizeof(double)*6);

    RobotMoveActionClass robotMove("robot_move_action", nh);
    // robotMove.subRobotState = nh.subscribe("/keti_robot_state", 1, &RobotMoveActionClass::RobotStateCallback, &robotMove);

    while(ros::ok()){
        robotInfor = RobotInfo();

        msg.robot_state = robotInfor.state;
        memcpy(msg.current_joint.data(), robotInfor.jnt, sizeof(double)*6);
        memcpy(msg.current_T_matrix.data(), robotInfor.mat, sizeof(double)*16);
        msg.current_position[0] = msg.current_T_matrix[3];
        msg.current_position[1] = msg.current_T_matrix[7];
        msg.current_position[2] = msg.current_T_matrix[11];
        double deg_z1, deg_y, deg_z2;

        double mat[9] = {0,};
        for(unsigned int i = 0; i < 3; i++){
            for(unsigned int j = 0; j < 3; j++){
                mat[i*3 + j] = robotInfor.mat[i*4 + j];
            }
        }

        Conf.InverseRot(mat, &deg_z1,&deg_y,&deg_z2);
        msg.current_rotation[0] = deg_z1;
        msg.current_rotation[1] = deg_y;
        msg.current_rotation[2] = deg_z2;

        // deg_z1 = atan2(mat[1*3 + 2], mat[0*3 + 2])*180/M_PI;
        // deg_y = acos(mat[2*3 + 2])*180/M_PI;
        // deg_z2 = atan2(mat[2*3 + 1], -mat[2*3 + 0])*180/M_PI;

        // ROS_INFO("current rotation : %f, %f, %f", deg_z1, deg_y, deg_z2);

        // ROS_INFO("current mat 1: %f, %f, %f", mat[0], mat[1], mat[2]);
        // ROS_INFO("current mat 2: %f, %f, %f", mat[3], mat[4], mat[5]);
        // ROS_INFO("current mat 3: %f, %f, %f", mat[6], mat[7], mat[8]);

        // ROS_INFO("current pose 1: %f, %f, %f, %f", robotInfor.mat[0], robotInfor.mat[1], robotInfor.mat[2], robotInfor.mat[3]);
        // ROS_INFO("current pose 2: %f, %f, %f, %f", robotInfor.mat[4], robotInfor.mat[5], robotInfor.mat[6], robotInfor.mat[7]);
        // ROS_INFO("current pose 3: %f, %f, %f, %f", robotInfor.mat[8], robotInfor.mat[9], robotInfor.mat[10], robotInfor.mat[11]);
        // ROS_INFO("current pose 4: %f, %f, %f, %f", robotInfor.mat[12], robotInfor.mat[13], robotInfor.mat[14], robotInfor.mat[15]);

        pubRobotState.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
} 

