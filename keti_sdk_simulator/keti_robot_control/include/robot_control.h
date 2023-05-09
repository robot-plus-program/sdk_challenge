#pragma once

#include <ros/ros.h>

#include <keti_robot_control/RobotState.h>
#include <keti_robot_control/GripperState.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

#include <actionlib/server/simple_action_server.h>
#include <keti_robot_control/RobotMoveAction.h>
#include <keti_robot_control/GripperMoveAction.h>

const std::string PLANNING_GROUP_ROBOT = "keti_robot";
const std::string PLANNING_GROUP_GRIPPER = "keti_gripper";

class RobotControl{
public:
    RobotControl(ros::NodeHandle nh, std::string robot_server_name, std::string gripper_server_name);
    ~RobotControl();

    ros::Subscriber subRobotMoveState;
    ros::Subscriber subRobotState, subGripperState;

    void RobotMoveStateCallback(const moveit_msgs::ExecuteTrajectoryActionFeedback &feedback);
    void RobotStateCallback(const keti_robot_control::RobotState &msg);
    void GripperStateCallback(const keti_robot_control::GripperState &msg);

    void start();

private:
    moveit::planning_interface::MoveGroupInterface *move_group_robot, *move_group_gripper;
    const moveit::core::JointModelGroup *robot_joint_model_group, *gripper_joint_model_group;
    bool moving, move_finish, plan_accepted;

    bool movej(std::vector<double> target_joint);
    bool movel(geometry_msgs::Pose target_pose);
    bool moveb(std::vector<geometry_msgs::Pose> waypoints);
    bool move_gripper(double target_dist);

    void executeCBRobot(const keti_robot_control::RobotMoveGoalConstPtr &goal);
    void executeCBGripper(const keti_robot_control::GripperMoveGoalConstPtr &gaol);

    ros::Publisher pubRobotState, pubGripperState;

    keti_robot_control::RobotState msgRobotState;
    keti_robot_control::GripperState msgGripperState;

    std::vector<double> current_joint, current_T_matrix, current_grip;
    geometry_msgs::Pose current_pose;

    std::string robot_action_name, gripper_action_name;

protected:
    actionlib::SimpleActionServer<keti_robot_control::RobotMoveAction> asRobot;
    actionlib::SimpleActionServer<keti_robot_control::GripperMoveAction> asGripper;
    keti_robot_control::RobotMoveFeedback feedbackRobot;
    keti_robot_control::RobotMoveResult resultRobot;
    keti_robot_control::GripperMoveFeedback feedbackGripper;
    keti_robot_control::GripperMoveResult resultGripper;
};