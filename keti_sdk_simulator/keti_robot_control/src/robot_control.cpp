#include "robot_control.h"

RobotControl::RobotControl(ros::NodeHandle nh, std::string name1, std::string name2) : 
        asRobot(nh, name1, boost::bind(&RobotControl::executeCBRobot, this, _1), false), robot_action_name(name1),
        asGripper(nh, name2, boost::bind(&RobotControl::executeCBGripper, this, _1), false), gripper_action_name(name2){

    move_group_gripper = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER);
    move_group_robot = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ROBOT);

    robot_joint_model_group = move_group_robot->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ROBOT);

    ROS_INFO_NAMED(PLANNING_GROUP_ROBOT, "Planning frame: %s", move_group_robot->getPlanningFrame().c_str());
    ROS_INFO_NAMED(PLANNING_GROUP_ROBOT, "End effector link: %s", move_group_robot->getEndEffectorLink().c_str());

    pubRobotState = nh.advertise<keti_robot_control::RobotState>("keti_robot_state", 1);
    pubGripperState = nh.advertise<keti_robot_control::GripperState>("keti_gripper_state", 1);

    current_joint.assign(6, 0);
    current_T_matrix.assign(16, 0);

    msgRobotState.state = 1;

    current_grip.assign(2, 0);

    msgGripperState.state = 1;
    msgGripperState.width = 0;

    current_joint = move_group_robot->getCurrentJointValues();
    ROS_INFO("current joint : %f, %f, %f, %f, %f, %f", current_joint[0], current_joint[1], current_joint[2], current_joint[3], current_joint[4], current_joint[5]);

    current_pose = move_group_robot->getCurrentPose().pose;
    ROS_INFO("current pose : %f, %f, %f, %f, %f, %f, %f", current_pose.position.x, current_pose.position.y, current_pose.position.z,
                                                            current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

    current_grip = move_group_gripper->getCurrentJointValues();
    ROS_INFO("current grip size : %f, %f", current_grip[0], current_grip[1]);

    subRobotMoveState = nh.subscribe("/execute_trajectory/feedback", 1, &RobotControl::RobotMoveStateCallback, this);

    asRobot.start();
    asGripper.start();
}

RobotControl::~RobotControl(){

}

void RobotControl::start(){

    ros::Rate loop_rate(10);

    while(ros::ok()){
        current_joint = move_group_robot->getCurrentJointValues();
        current_pose = move_group_robot->getCurrentPose().pose;

        double sqw = current_pose.orientation.w*current_pose.orientation.w;
        double sqx = current_pose.orientation.x*current_pose.orientation.x;
        double sqy = current_pose.orientation.y*current_pose.orientation.y;
        double sqz = current_pose.orientation.z*current_pose.orientation.z;

        // invs (inverse square length) is only required if quaternion is not already normalised
        double invs = 1 / (sqx + sqy + sqz + sqw);
        current_T_matrix[0*4 + 0] = ( sqx - sqy - sqz + sqw)*invs; // since sqw + sqx + sqy + sqz =1/invs*invs
        current_T_matrix[1*4 + 1] = (-sqx + sqy - sqz + sqw)*invs;
        current_T_matrix[2*4 + 2] = (-sqx - sqy + sqz + sqw)*invs;
        
        double tmp1 = current_pose.orientation.x*current_pose.orientation.y;
        double tmp2 = current_pose.orientation.z*current_pose.orientation.w;
        current_T_matrix[1*4 + 0] = 2.0 * (tmp1 + tmp2)*invs;
        current_T_matrix[0*4 + 1] = 2.0 * (tmp1 - tmp2)*invs;
        
        tmp1 = current_pose.orientation.x*current_pose.orientation.z;
        tmp2 = current_pose.orientation.y*current_pose.orientation.w;
        current_T_matrix[2*4 + 0] = 2.0 * (tmp1 - tmp2)*invs;
        current_T_matrix[0*4 + 2] = 2.0 * (tmp1 + tmp2)*invs;
        tmp1 = current_pose.orientation.y*current_pose.orientation.z;
        tmp2 = current_pose.orientation.x*current_pose.orientation.w;
        current_T_matrix[2*4 + 1] = 2.0 * (tmp1 + tmp2)*invs;
        current_T_matrix[1*4 + 2] = 2.0 * (tmp1 - tmp2)*invs;

        current_T_matrix[0*4 + 3] = current_pose.position.x;
        current_T_matrix[1*4 + 3] = current_pose.position.y;
        current_T_matrix[2*4 + 3] = current_pose.position.z;
        current_T_matrix[3*4 + 3] = 1;

        memcpy(msgRobotState.current_joint.data(), current_joint.data(), sizeof(double)*6);
        memcpy(msgRobotState.current_T_matrix.data(), current_T_matrix.data(), sizeof(double)*16);

        // ROS_INFO("%f, %f, %f, %f", current_T_matrix[0], current_T_matrix[1], current_T_matrix[2], current_T_matrix[3]);
        // ROS_INFO("%f, %f, %f, %f", current_T_matrix[4], current_T_matrix[5], current_T_matrix[6], current_T_matrix[7]);
        // ROS_INFO("%f, %f, %f, %f", current_T_matrix[8], current_T_matrix[9], current_T_matrix[10], current_T_matrix[11]);
        // ROS_INFO("%f, %f, %f, %f", current_T_matrix[12], current_T_matrix[13], current_T_matrix[14], current_T_matrix[15]);
        // ROS_INFO("current orientation : %f, %f, %f, %f\n\n", 
        //     current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z);

        current_grip = move_group_gripper->getCurrentJointValues();
        msgGripperState.width = current_grip[0];

        pubRobotState.publish(msgRobotState);
        pubGripperState.publish(msgGripperState);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void RobotControl::executeCBRobot(const keti_robot_control::RobotMoveGoalConstPtr &goal){
    ros::Rate loop_rate(10);
    bool success = true;

    feedbackRobot.sequence.clear();
    resultRobot.sequence.clear();


    if(goal->cmd == 1){
        ROS_INFO("%s: Executing, creating robot move sequence, cmd : %d, value : %f, %f, %f, %f, %f, %f", robot_action_name.c_str(),
                goal->cmd, goal->value[0], goal->value[1], goal->value[2], goal->value[3], goal->value[4], goal->value[5]);
        std::vector<double> target_joint(6, 0);
        memcpy(target_joint.data(), goal->value.data(), sizeof(double)*6);
        
        move_group_robot->setPlanningTime(2.0);
        move_group_robot->setJointValueTarget(target_joint);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (move_group_robot->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED(PLANNING_GROUP_ROBOT, "Visualizing plan 1 (joint goal) %s", success ? "SUCCEED" : "FAILED");
        if(success){
            move_group_robot->asyncExecute(plan);
            ros::Duration().sleep();
        }
    }
    else if(goal->cmd == 2){
        ROS_INFO("%s: Executing, creating robot move sequence, cmd : %d, value : ", robot_action_name.c_str(), goal->cmd);
        ROS_INFO("%f, %f, %f, %f", goal->value[0], goal->value[1], goal->value[2], goal->value[3]);
        ROS_INFO("%f, %f, %f, %f", goal->value[4], goal->value[5], goal->value[6], goal->value[7]);
        ROS_INFO("%f, %f, %f, %f", goal->value[8], goal->value[9], goal->value[10], goal->value[11]);
        ROS_INFO("%f, %f, %f, %f", goal->value[12], goal->value[13], goal->value[14], goal->value[15]);

        double tr = goal->value[0*4 + 0] + goal->value[1*4 + 1] + goal->value[2*4 + 2];
        double m00, m01, m02, m10, m11, m12, m20, m21, m22;
        double qw, qx, qy, qz;
        m00 = goal->value[0*4 + 0]; m01 = goal->value[0*4 + 1]; m02 = goal->value[0*4 + 2];
        m10 = goal->value[1*4 + 0]; m11 = goal->value[1*4 + 1]; m12 = goal->value[1*4 + 2];
        m20 = goal->value[2*4 + 0]; m21 = goal->value[2*4 + 1]; m22 = goal->value[2*4 + 2];

        if (tr > 0) {
            double S = sqrt(tr + 1.0) * 2; // S=4*qw
            qw = 0.25 * S;
            qx = (m21 - m12) / S;
            qy = (m02 - m20) / S;
            qz = (m10 - m01) / S;
        }
        else if ((m00 > m11) & (m00 > m22)) {
            double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
            qw = (m21 - m12) / S;
            qx = 0.25 * S;
            qy = (m01 + m10) / S;
            qz = (m02 + m20) / S;
        }
        else if (m11 > m22) {
            double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
            qw = (m02 - m20) / S;
            qx = (m01 + m10) / S;
            qy = 0.25 * S;
            qz = (m12 + m21) / S;
        }
        else {
            double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
            qw = (m10 - m01) / S;
            qx = (m02 + m20) / S;
            qy = (m12 + m21) / S;
            qz = 0.25 * S;
        }

        geometry_msgs::Pose current_pose = move_group_robot->getCurrentPose().pose;
        geometry_msgs::Pose target_pose;
        target_pose.position.x = goal->value[3];
        target_pose.position.y = goal->value[7];
        target_pose.position.z = goal->value[11];
        target_pose.orientation.x = qx;
        target_pose.orientation.y = qy;
        target_pose.orientation.z = qz;
        target_pose.orientation.w = qw;

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(current_pose);
        waypoints.push_back(target_pose);

        moveit_msgs::RobotTrajectory trajectory;
        double jump_threshold = 0;
        double eef_step = 0.01;
        double fraction = move_group_robot->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_NAMED(PLANNING_GROUP_ROBOT, "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

        bool success = false;

        if(fraction >= 0.7)
        {
            move_group_robot->asyncExecute(trajectory);
            ros::Duration().sleep();
            success = true;
        }
    }
    else if(goal->cmd == 3){
        ROS_INFO("%s: Executing, creating robot move sequence, cmd : %d, num : %d", robot_action_name.c_str(), goal->cmd, goal->num);
        std::vector<geometry_msgs::Pose> waypoints;

        geometry_msgs::Pose current_pose = move_group_robot->getCurrentPose().pose;
        waypoints.push_back(current_pose);

        for(unsigned int num = 0; num < goal->num; num++){
            // ROS_INFO("%f, %f, %f, %f", goal->value[16*num + 0], goal->value[16*num + 1], goal->value[16*num + 2], goal->value[16*num + 3]);
            // ROS_INFO("%f, %f, %f, %f", goal->value[16*num + 4], goal->value[16*num + 5], goal->value[16*num + 6], goal->value[16*num + 7]);
            // ROS_INFO("%f, %f, %f, %f", goal->value[16*num + 8], goal->value[16*num + 9], goal->value[16*num + 10], goal->value[16*num + 11]);
            // ROS_INFO("%f, %f, %f, %f", goal->value[16*num + 12], goal->value[16*num + 13], goal->value[16*num + 14], goal->value[16*num + 15]);

            double value[16];
            for(unsigned int i = 0; i < 16; i++){
                value[i] = goal->value[16*num + i];
            }

            double tr = value[0*4 + 0] + value[1*4 + 1] + value[2*4 + 2];
            double m00, m01, m02, m10, m11, m12, m20, m21, m22;
            double qw, qx, qy, qz;
            m00 = value[0*4 + 0]; m01 = value[0*4 + 1]; m02 = value[0*4 + 2];
            m10 = value[1*4 + 0]; m11 = value[1*4 + 1]; m12 = value[1*4 + 2];
            m20 = value[2*4 + 0]; m21 = value[2*4 + 1]; m22 = value[2*4 + 2];

            if (tr > 0) {
                double S = sqrt(tr + 1.0) * 2; // S=4*qw
                qw = 0.25 * S;
                qx = (m21 - m12) / S;
                qy = (m02 - m20) / S;
                qz = (m10 - m01) / S;
            }
            else if ((m00 > m11) & (m00 > m22)) {
                double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
                qw = (m21 - m12) / S;
                qx = 0.25 * S;
                qy = (m01 + m10) / S;
                qz = (m02 + m20) / S;
            }
            else if (m11 > m22) {
                double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
                qw = (m02 - m20) / S;
                qx = (m01 + m10) / S;
                qy = 0.25 * S;
                qz = (m12 + m21) / S;
            }
            else {
                double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
                qw = (m10 - m01) / S;
                qx = (m02 + m20) / S;
                qy = (m12 + m21) / S;
                qz = 0.25 * S;
            }

            geometry_msgs::Pose target_pose;
            target_pose.position.x = value[3];
            target_pose.position.y = value[7];
            target_pose.position.z = value[11];
            target_pose.orientation.x = qx;
            target_pose.orientation.y = qy;
            target_pose.orientation.z = qz;
            target_pose.orientation.w = qw;
            target_pose.orientation = current_pose.orientation;
            waypoints.push_back(target_pose);
        }

        moveit_msgs::RobotTrajectory trajectory;
        double jump_threshold = 0.0;
        double eef_step = 0.01;
        double fraction = move_group_robot->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_NAMED(PLANNING_GROUP_ROBOT, "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

        bool success = false;

        if(fraction >= 0.7){
            move_group_robot->asyncExecute(trajectory);
            ros::Duration().sleep();
            success = true;
        }
    }

    while(ros::ok())
    {
        loop_rate.sleep();

        if(asRobot.isPreemptRequested() || !ros::ok){
            ROS_INFO("%s: Preempted", robot_action_name.c_str());
            // set the action state to preempted
            asRobot.setPreempted();
            success = false;
            break;
        }

        feedbackRobot.sequence.push_back(msgRobotState.state);
        asRobot.publishFeedback(feedbackRobot);

        if(move_finish) break;
    }

    if(success){
        resultRobot.sequence.push_back(1);
    }
    else{
        resultRobot.sequence.push_back(0);
    }
    // ROS_INFO("%s : Succeeded", robot_action_name.c_str());
    // set the action state to succeeded
    asRobot.setSucceeded(resultRobot);
}

void RobotControl::executeCBGripper(const keti_robot_control::GripperMoveGoalConstPtr &goal){
    ros::Rate loop_rate(10);
    bool success = true;

    feedbackGripper.sequence.clear();
    feedbackGripper.width.clear();
    resultGripper.sequence.clear();

    ROS_INFO("%s : Executing, creating robot move sequence, cmd : %d", gripper_action_name.c_str(), goal->cmd);

    if(goal->cmd == 1){
        move_gripper(0.04);
    }
    else if(goal->cmd == 2){
        move_gripper(0.0);
    }

    while(ros::ok()){
        loop_rate.sleep();

        if(asGripper.isPreemptRequested() || !ros::ok){
            ROS_INFO("%s : Preempted", gripper_action_name.c_str());
            // set the action state to preempted
            asGripper.setPreempted();
            success = false;
            // Stop();
            break;
        }

        feedbackGripper.sequence.push_back(msgGripperState.state);
        // feedbackGripper.width.push_back(msgGripperState.width);
        asGripper.publishFeedback(feedbackGripper);

        if(move_finish) break;
    }

    if(success){
        resultGripper.sequence.push_back(1);
    }
    else{
        resultGripper.sequence.push_back(0);
    }
    ROS_INFO("%s : Succeeded", gripper_action_name.c_str());
    // set the action state to succeeded
    asGripper.setSucceeded(resultGripper);
}

void RobotControl::RobotMoveStateCallback(const moveit_msgs::ExecuteTrajectoryActionFeedback &feedback){
    ROS_INFO("%d, %s", feedback.status.status, feedback.status.text.c_str());
    if(feedback.status.status == 1){
        plan_accepted = true;
        moving = true;
        move_finish = false;
        msgRobotState.state = 2;
        msgGripperState.state = 2;
    }
    else if(feedback.status.status == 3){
        plan_accepted = false;
        moving = false;
        move_finish = true;
        msgRobotState.state = 1;
        msgGripperState.state = 1;
    }
}

bool RobotControl::movej(std::vector<double> target_joint){
    move_group_robot->setPlanningTime(2.0);
    move_group_robot->setJointValueTarget(target_joint);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group_robot->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED(PLANNING_GROUP_ROBOT, "Visualizing plan 1 (joint goal) %s", success ? "SUCCEED" : "FAILED");
    if (success)
    {
        move_group_robot->asyncExecute(plan);
        ros::Duration().sleep();
    }

    return success;
}

bool RobotControl::movel(geometry_msgs::Pose target_pose){
    geometry_msgs::Pose current_pose = move_group_robot->getCurrentPose().pose;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(current_pose);
    waypoints.push_back(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
    double jump_threshold = 0.0;
    double eef_step = 0.01;
    double fraction = move_group_robot->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED(PLANNING_GROUP_ROBOT, "Visualizing plan (Cartesian path to pick) (%.2f%% achieved)", fraction * 100.0);

    bool success = false;

    if(fraction >= 0.7){
        move_group_robot->asyncExecute(trajectory);
        ros::Duration().sleep();
        success = true;
    }

    return success;
}

bool RobotControl::moveb(std::vector<geometry_msgs::Pose> waypoints){
    geometry_msgs::Pose current_pose = move_group_robot->getCurrentPose().pose;

    moveit_msgs::RobotTrajectory trajectory;
    double jump_threshold = 0.0;
    double eef_step = 0.01;
    double fraction = move_group_robot->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED(PLANNING_GROUP_ROBOT, "Visualizing plan (Cartesian path to pick) (%.2f%% achieved)", fraction * 100.0);

    bool success = false;

    if(fraction >= 0.7){
        move_group_robot->asyncExecute(trajectory);
        ros::Duration().sleep();
        success = true;
    }

    return success;
}

bool RobotControl::move_gripper(double target_dist){
    std::vector<double> target_joint = {target_dist, target_dist};

    move_group_gripper->setPlanningTime(2.0);
    move_group_gripper->setJointValueTarget(target_joint);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group_gripper->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED(PLANNING_GROUP_GRIPPER, "Visualizing plan 1 (joint goal) %s", success ? "SUCCEED" : "FAILED");
    if(success){
        move_group_gripper->asyncExecute(plan);
        ros::Duration().sleep();
    }

    return success;
}

void RobotControl::RobotStateCallback(const keti_robot_control::RobotState &msg){
    // ROS_INFO("robot_state : %d", msg.state);
    // ROS_INFO("current joint : %f, %f, %f, %f, %f, %f", msg.current_joint[0], msg.current_joint[1], msg.current_joint[2], msg.current_joint[3], msg.current_joint[4], msg.current_joint[5]);
    // ROS_INFO("current T matrix : ");
    // ROS_INFO("%f, %f, %f, %f", msg.current_T_matrix[0], msg.current_T_matrix[1], msg.current_T_matrix[2], msg.current_T_matrix[3]);
    // ROS_INFO("%f, %f, %f, %f", msg.current_T_matrix[4], msg.current_T_matrix[5], msg.current_T_matrix[6], msg.current_T_matrix[7]);
    // ROS_INFO("%f, %f, %f, %f", msg.current_T_matrix[8], msg.current_T_matrix[9], msg.current_T_matrix[10], msg.current_T_matrix[11]);
    // ROS_INFO("%f, %f, %f, %f", msg.current_T_matrix[12], msg.current_T_matrix[13], msg.current_T_matrix[14], msg.current_T_matrix[15]);
}

void RobotControl::GripperStateCallback(const keti_robot_control::GripperState &msg){
    // ROS_INFO("gripper_state : %d", msg.state);
    // ROS_INFO("width : %f", msg.width);
}