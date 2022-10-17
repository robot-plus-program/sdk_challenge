#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <keti_robot/RobotMoveAction.h>
#include <keti_msgs/RobotState.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

const std::string PLANNING_GROUP = "arm";
const std::vector<double> init_joint = {0, 0, M_PI_2, 0, M_PI_2, 0};

class RobotMoveSimActionClass{
protected:
    actionlib::SimpleActionServer<keti_robot::RobotMoveAction> as;
    std::string action_name;
    keti_robot::RobotMoveFeedback feedback;
    keti_robot::RobotMoveResult result;

public:
    RobotMoveSimActionClass(std::string name, ros::NodeHandle nh);
    ~RobotMoveSimActionClass();

    void executeCB(const keti_robot::RobotMoveGoalConstPtr &goal);
    void RobotStateCallback(const keti_msgs::RobotState &msg);
    void RobotMoveStateCallback(const moveit_msgs::ExecuteTrajectoryActionFeedback &feedback);
    void getCurrentJoint(std::vector<double> *vec_dst);
    void getCurrentPose(geometry_msgs::Pose *pose_dst);

    ros::Subscriber subRobotState;
    ros::Subscriber subRobotMoveState;
private:
    int robot_state;
    double current_joint[6];
    double current_position[6];
    double current_rotation[6];
    double current_T_matrix[16];

    moveit::planning_interface::MoveGroupInterface *move_group_interface;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group;

    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
};