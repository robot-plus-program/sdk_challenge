#include "robot_move_action.h"

RobotMoveActionClass::RobotMoveActionClass(std::string name, ros::NodeHandle nh) : as(nh, name, boost::bind(&RobotMoveActionClass::executeCB, this, _1), false), action_name(name)
{
    robot_state = 0;
    memset(current_joint, 0, sizeof(double)*6);
    memset(current_position, 0, sizeof(double)*3);
    memset(current_rotation, 0, sizeof(double)*3);
    memset(current_T_matrix, 0, sizeof(double)*16);

    as.start();
}

RobotMoveActionClass::~RobotMoveActionClass()
{
}

void RobotMoveActionClass::RobotStateCallback(const keti_msgs::RobotState &msg)
{
    // ROS_INFO("robot_state : %d", msg.robot_state);
    // ROS_INFO("current joint : %f, %f, %f, %f, %f, %f", msg.current_joint[0], msg.current_joint[1], msg.current_joint[2], msg.current_joint[3], msg.current_joint[4], msg.current_joint[5]);
    // ROS_INFO("current position : %f, %f, %f", msg.current_position[0], msg.current_position[1], msg.current_position[2]);
    // ROS_INFO("current rotation : %f, %f, %f", msg.current_rotation[0], msg.current_rotation[1], msg.current_rotation[2]);
    // ROS_INFO("current_T_matrix 1: %f, %f, %f, %f", msg.current_T_matrix[0], msg.current_T_matrix[1], msg.current_T_matrix[2], msg.current_T_matrix[3]);
    // ROS_INFO("current_T_matrix 2: %f, %f, %f, %f", msg.current_T_matrix[4], msg.current_T_matrix[5], msg.current_T_matrix[6], msg.current_T_matrix[7]);
    // ROS_INFO("current_T_matrix 3: %f, %f, %f, %f", msg.current_T_matrix[8], msg.current_T_matrix[9], msg.current_T_matrix[10], msg.current_T_matrix[11]);
    // ROS_INFO("current_T_matrix 4: %f, %f, %f, %f", msg.current_T_matrix[12], msg.current_T_matrix[13], msg.current_T_matrix[14], msg.current_T_matrix[15]);

    robot_state = msg.robot_state;
    memcpy(current_joint, msg.current_joint.data(), sizeof(double) * 6);
    memcpy(current_position, msg.current_position.data(), sizeof(double) * 3);
    memcpy(current_rotation, msg.current_rotation.data(), sizeof(double) * 3);
    memcpy(current_T_matrix, msg.current_T_matrix.data(), sizeof(double) * 16);
}

void RobotMoveActionClass::executeCB(const keti_robot::RobotMoveGoalConstPtr &goal)
{
    // helper variables
    ros::Rate r(10);
    bool success = true;

    feedback.sequence.clear();
    result.sequence.clear();

    // publish info to the console for the user
    // ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->cmd, feedback_.sequence[0], feedback_.sequence[1]);
    for(int i = 0; i < goal->num; i++){
        ROS_INFO("%s: Executing, creating robot move sequence, cmd : %d, value : %f, %f, %f, %f, %f, %f, velocity : %f",
                action_name.c_str(), goal->cmd, goal->value[i*6 + 0], 
                goal->value[i*6 + 1], goal->value[i*6 + 2], goal->value[i*6 + 3], 
                goal->value[i*6 + 4], goal->value[i*6 + 5], goal->velocity);
    }
    if(goal->velocity > 0){
        SetVelocity(goal->velocity);
        usleep(100000);
    }

    double value[6];
    memcpy(value, goal->value.data(), sizeof(double)*6);
    if (goal->cmd == 1)
    {
        movej(value);
    }
    else if (goal->cmd == 2)
    {
        double mat[9] = {0,};
        double ang_z1 = goal->value[3], ang_y = goal->value[4], ang_z2 = goal->value[5];
        double Rz1[9] = {cos(ang_z1), -sin(ang_z1), 0, sin(ang_z1), cos(ang_z1), 0, 0, 0, 1};
        double Ry[9] = {cos(ang_y), 0, sin(ang_y), 0, 1, 0, -sin(ang_y), 0, cos(ang_y)};
        double Rz2[9] = {cos(ang_z2), -sin(ang_z2), 0, sin(ang_z2), cos(ang_z2), 0, 0, 0, 1};
        double Rz1y[9] = {0,};
        double temp = 0;
        for(unsigned int i = 0; i < 3; i++){
            for(unsigned int j = 0; j < 3; j++){
                temp = 0;
                for(unsigned int k = 0; k < 3; k++){
                    temp += Rz1[i*3 + k]*Ry[k*3 + j];
                }
                Rz1y[i*3 + j] = temp;
            }
        }

        for(unsigned int i = 0; i < 3; i++){
            for(unsigned int j = 0; j < 3; j++){
                temp = 0;
                for(unsigned int k = 0; k < 3; k++){
                    temp += Rz1y[i*3 + k]*Rz2[k*3 + j];
                }
                mat[i*3 + j] = temp;
            }
        }

        double result_mat[16];
        memcpy(result_mat, current_T_matrix, sizeof(double)*16);
        result_mat[3]  = value[0];
        result_mat[7]  = value[1];
        result_mat[11] = value[2];
        for(unsigned int i = 0; i < 3; i++){
            for(unsigned int j = 0; j < 3; j++){
                result_mat[i*4 + j] = mat[i*3 + j];
            }
        }

        movel(base, result_mat);
    }
    else if(goal->cmd == 3){
        double moveb_mat[5][16];
        for(int indx = 0; indx < goal->num; indx++){
            double mat[9] = {0,};
            double ang_z1 = goal->value[indx*6 + 3], ang_y = goal->value[indx*6 + 4], ang_z2 = goal->value[indx*6 + 5];
            double Rz1[9] = {cos(ang_z1), -sin(ang_z1), 0, sin(ang_z1), cos(ang_z1), 0, 0, 0, 1};
            double Ry[9] = {cos(ang_y), 0, sin(ang_y), 0, 1, 0, -sin(ang_y), 0, cos(ang_y)};
            double Rz2[9] = {cos(ang_z2), -sin(ang_z2), 0, sin(ang_z2), cos(ang_z2), 0, 0, 0, 1};
            double Rz1y[9] = {0,};
            double temp = 0;
            for(unsigned int i = 0; i < 3; i++){
                for(unsigned int j = 0; j < 3; j++){
                    temp = 0;
                    for(unsigned int k = 0; k < 3; k++){
                        temp += Rz1[i*3 + k]*Ry[k*3 + j];
                    }
                    Rz1y[i*3 + j] = temp;
                }
            }

            for(unsigned int i = 0; i < 3; i++){
                for(unsigned int j = 0; j < 3; j++){
                    temp = 0;
                    for(unsigned int k = 0; k < 3; k++){
                        temp += Rz1y[i*3 + k]*Rz2[k*3 + j];
                    }
                    mat[i*3 + j] = temp;
                }
            }

            double result_mat[16];
            memcpy(result_mat, current_T_matrix, sizeof(double)*16);
            result_mat[3]  = goal->value[indx*6 + 0];
            result_mat[7]  = goal->value[indx*6 + 1];
            result_mat[11] = goal->value[indx*6 + 2];
            for(unsigned int i = 0; i < 3; i++){
                for(unsigned int j = 0; j < 3; j++){
                    result_mat[i*4 + j] = mat[i*3 + j];
                }
            }

            memcpy(moveb_mat[indx], result_mat, sizeof(double)*16);

            // for(unsigned int i = 0; i < 4; i++){
            //     for(unsigned int j = 0; j < 4; j++){
            //         printf("%f\t", moveb_mat[indx][i*4 + j]);
            //     }
            //     printf("\n");
            // }
            // printf("\n");
        }
        // moveb(base, 40, goal->num, moveb_mat[0], moveb_mat[1], moveb_mat[2], moveb_mat[3], moveb_mat[4]);
        moveb(base, 40, goal->num, moveb_mat[0], moveb_mat[1], moveb_mat[2], moveb_mat[3]);
    }
    else{
        return;
    }

    int count = 0;
    bool moving = false;
    while(ros::ok()){
        r.sleep();
        ROS_INFO("Current robot state : %d", robot_state);

        if (as.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name.c_str());
            // set the action state to preempted
            as.setPreempted();
            success = false;
            Stop();
            break;
        }

        feedback.sequence.push_back(robot_state);
        as.publishFeedback(feedback);

        if(robot_state == 1) count++;
        if(count >= 30) break;

        if(robot_state == 2) moving = true;
        if(robot_state == 1 && moving) break;
    }

    if (success)
    {
        result.sequence.push_back(1);
        ROS_INFO("%s: Succeeded", action_name.c_str());
        // set the action state to succeeded
        as.setSucceeded(result);
    }
}