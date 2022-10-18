#include <ros/ros.h>
#include <keti_msgs/RobotState.h>

#include "robot_move_sim_action.h"

static keti_msgs::RobotState msg;

void RobotStateCallback(const keti_msgs::RobotState &msg){
    // ROS_INFO("robot_state : %d", msg.robot_state);
    ROS_INFO("current joint : %f, %f, %f, %f, %f, %f", msg.current_joint[0], msg.current_joint[1], msg.current_joint[2], msg.current_joint[3], msg.current_joint[4], msg.current_joint[5]);
    ROS_INFO("current position : %f, %f, %f", msg.current_position[0], msg.current_position[1], msg.current_position[2]);
    ROS_INFO("current rotation : %f, %f, %f", msg.current_rotation[0], msg.current_rotation[1], msg.current_rotation[2]);
    ROS_INFO("current pose 1: %f, %f, %f, %f", msg.current_T_matrix[0], msg.current_T_matrix[1], msg.current_T_matrix[2], msg.current_T_matrix[3]);
    ROS_INFO("current pose 2: %f, %f, %f, %f", msg.current_T_matrix[4], msg.current_T_matrix[5], msg.current_T_matrix[6], msg.current_T_matrix[7]);
    ROS_INFO("current pose 3: %f, %f, %f, %f", msg.current_T_matrix[8], msg.current_T_matrix[9], msg.current_T_matrix[10], msg.current_T_matrix[11]);
    ROS_INFO("current pose 4: %f, %f, %f, %f", msg.current_T_matrix[12], msg.current_T_matrix[13], msg.current_T_matrix[14], msg.current_T_matrix[15]);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_control_sim_node");
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

	ros::NodeHandle nh;
    ros::Publisher pubRobotState = nh.advertise<keti_msgs::RobotState>("keti_robot_state",1);

    ros::Rate loop_rate(100);

    msg.robot_state = 0;
    memset(msg.current_T_matrix.data(), 0, sizeof(double)*16);
    memset(msg.current_joint.data(), 0, sizeof(double)*6);
    memset(msg.current_position.data(), 0, sizeof(double)*6);
    memset(msg.current_rotation.data(), 0, sizeof(double)*6);

    RobotMoveSimActionClass robotMove("robot_move_sim_action", nh);
    robotMove.subRobotState = nh.subscribe("/keti_robot_state", 1, &RobotMoveSimActionClass::RobotStateCallback, &robotMove);
    // ros::Subscriber subRobotState = nh.subscribe("/keti_robot_state", 1, &RobotStateCallback);
    robotMove.subRobotMoveState = nh.subscribe("/execute_trajectory/feedback", 1, &RobotMoveSimActionClass::RobotMoveStateCallback, &robotMove);

    std::vector<double> current_joint;
    current_joint.assign(6, 0);
    geometry_msgs::Pose current_pose;

    double w, x, y, z;
    double T_matrix[16] = {0,};

    while(ros::ok()){
        msg.robot_state = robotMove.getCurrentState();
        robotMove.getCurrentJoint(&current_joint);
        robotMove.getCurrentPose(&current_pose);

        w = current_pose.orientation.w;
        x = current_pose.orientation.x;
        y = current_pose.orientation.y;
        z = current_pose.orientation.z;

        memset(T_matrix, 0, sizeof(double)*16);
        T_matrix[15] = 1;
        T_matrix[0*4 + 0] = 1 - 2*y*y - 2*x*x;
        T_matrix[0*4 + 1] = 2*x*y - 2*w*z;
        T_matrix[0*4 + 2] = 2*x*z + 2*w*y;
        T_matrix[1*4 + 0] = 2*x*y + 2*w*z;
        T_matrix[1*4 + 1] = 1 - 2*x*x - 2*z*z;
        T_matrix[1*4 + 2] = 2*y*z - 2*w*x;
        T_matrix[2*4 + 0] = 2*x*z - 2*w*y;
        T_matrix[2*4 + 1] = 2*y*z + 2*w*x;
        T_matrix[2*4 + 2] = 1 - 2*x*x - 2*y*y;

        T_matrix[0*4 + 3] = current_pose.position.x;
        T_matrix[1*4 + 3] = current_pose.position.y;
        T_matrix[2*4 + 3] = current_pose.position.z;

        std::copy(current_joint.begin(), current_joint.end(), msg.current_joint.begin());
        memcpy(msg.current_T_matrix.data(), T_matrix, sizeof(double)*16);
        msg.current_position[0] = msg.current_T_matrix[3];
        msg.current_position[1] = msg.current_T_matrix[7];
        msg.current_position[2] = msg.current_T_matrix[11];

        double deg_z1, deg_y, deg_z2;

        double mat[9] = {0,};
        for(unsigned int i = 0; i < 3; i++){
            for(unsigned int j = 0; j < 3; j++){
                mat[i*3 + j] = T_matrix[i*4 + j];
            }
        }

        deg_z1 = atan2(mat[1*3 + 2], mat[0*3 + 2])*180/M_PI;
        deg_y = acos(mat[2*3 + 2])*180/M_PI;
        deg_z2 = atan2(mat[2*3 + 1], -mat[2*3 + 0])*180/M_PI;

        msg.current_rotation[0] = deg_z1;
        msg.current_rotation[1] = deg_y;
        msg.current_rotation[2] = deg_z2;

        // ROS_INFO("current rotation : %f, %f, %f", deg_z1, deg_y, deg_z2);

        // ROS_INFO("current mat 1: %f, %f, %f", mat[0], mat[1], mat[2]);
        // ROS_INFO("current mat 2: %f, %f, %f", mat[3], mat[4], mat[5]);
        // ROS_INFO("current mat 3: %f, %f, %f", mat[6], mat[7], mat[8]);

        // ROS_INFO("current pose 1: %f, %f, %f, %f", msg.current_T_matrix[0], msg.current_T_matrix[1], msg.current_T_matrix[2], msg.current_T_matrix[3]);
        // ROS_INFO("current pose 2: %f, %f, %f, %f", msg.current_T_matrix[4], msg.current_T_matrix[5], msg.current_T_matrix[6], msg.current_T_matrix[7]);
        // ROS_INFO("current pose 3: %f, %f, %f, %f", msg.current_T_matrix[8], msg.current_T_matrix[9], msg.current_T_matrix[10], msg.current_T_matrix[11]);
        // ROS_INFO("current pose 4: %f, %f, %f, %f", msg.current_T_matrix[12], msg.current_T_matrix[13], msg.current_T_matrix[14], msg.current_T_matrix[15]);

        pubRobotState.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
} 

