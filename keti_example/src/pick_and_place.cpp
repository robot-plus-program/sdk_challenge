#include <ros/ros.h>
#include <keti_msgs/RobotState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <keti_robot/RobotMoveAction.h>
#include <keti_gripper/GripperMoveAction.h>
#include <keti_msgs/VisionState.h>
#include <keti_msgs/VisionComm.h>

static int robot_state = 0;
static double current_joint[6];
static double current_position[3];
static double current_rotation[3];
static double current_T_matrix[16];
static bool print_enable = false;
static bool robot_move_complete = false;
static bool robot_moving = false;
static bool gripper_move_complete = false;
static bool gripper_moving = false;
static int step = 0;

enum State{Idle=0, MoveWait, RobotMoveJ, RobotMoveL, RobotMoveB, GripperMoveGrip, GripperMoveRelease, Stop};
static int state = 0;

void RobotStateCallback(const keti_msgs::RobotState &msg){
    robot_state = msg.robot_state;
    memcpy(current_joint, msg.current_joint.data(), sizeof(double)*6);
    memcpy(current_position, msg.current_position.data(), sizeof(double)*3);
    memcpy(current_rotation, msg.current_rotation.data(), sizeof(double)*3);
    memcpy(current_T_matrix, msg.current_T_matrix.data(), sizeof(double)*16);

    if(print_enable){
        ROS_INFO("robot_state : %d", robot_state);
        ROS_INFO("current joint : %f, %f, %f, %f, %f, %f", current_joint[0], current_joint[1], current_joint[2], current_joint[3], current_joint[4], current_joint[5]);
        ROS_INFO("current position : %f, %f, %f", current_position[0], current_position[1], current_position[2]);
        ROS_INFO("current rotation : %f, %f, %f", current_rotation[0], current_rotation[1], current_rotation[2]);
        ROS_INFO("current_T_matrix 1: %f, %f, %f, %f", current_T_matrix[0], current_T_matrix[1], current_T_matrix[2], current_T_matrix[3]);
        ROS_INFO("current_T_matrix 2: %f, %f, %f, %f", current_T_matrix[4], current_T_matrix[5], current_T_matrix[6], current_T_matrix[7]);
        ROS_INFO("current_T_matrix 3: %f, %f, %f, %f", current_T_matrix[8], current_T_matrix[9], current_T_matrix[10], current_T_matrix[11]);
        ROS_INFO("current_T_matrix 4: %f, %f, %f, %f", current_T_matrix[12], current_T_matrix[13], current_T_matrix[14], current_T_matrix[15]);
    }
}

void RobotMoveCompleteCallback(const actionlib::SimpleClientGoalState& status, const keti_robot::RobotMoveResultConstPtr& result){
    ROS_INFO("Robot move complete");
    robot_move_complete = true;

    state = State::Idle;
}

void RobotMovingFeedbackCacllback(const keti_robot::RobotMoveFeedbackConstPtr& feedback){
    int state = feedback->sequence.back();
    if(state == 1){
        robot_moving = false;
        ROS_INFO("Robot don't move");
    }
    else if(state == 2){
        robot_moving = true;
        robot_move_complete = false;
        ROS_INFO("Robot moving");
    }
}

void RobotMovingActiveCallback(){
    ROS_INFO("Goal just went active");
    robot_moving = false;
    robot_move_complete = false;

    state = State::MoveWait;
}

void GripperMoveCompleteCallback(const actionlib::SimpleClientGoalState& status, const keti_gripper::GripperMoveResultConstPtr& result){
    ROS_INFO("Gripper move complete");

    gripper_move_complete = true;
    gripper_moving = false;

    state = State::Idle;
}

void GripperMovingFeedbackCallback(const keti_gripper::GripperMoveFeedbackConstPtr& feedback){
    ROS_INFO("gripper status : %d", feedback->status.back());
    ROS_INFO("gripper width : %f", feedback->width.back());

    gripper_move_complete = false;
    gripper_moving = true;
}

void GripperMovingActiveCallback(){
    ROS_INFO("Goal just went active");

    gripper_move_complete = false;
    gripper_moving = false;

    state = State::MoveWait;
}

void VisionStateCallback(const keti_msgs::VisionState &msg){
}

int main(int argc, char **argv){
    ros::init(argc, argv, "keti_pick_and_place_sample_node");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    
    bool print;
    nh.getParam("print", print);
    ROS_INFO("print : %d", print);
    print_enable = print;

    ros::Subscriber subRobotState = nh.subscribe("/keti_robot_state", 1, RobotStateCallback);
    ros::Subscriber subVisionState = nh.subscribe("/keti_vision_state", 1, VisionStateCallback);
    ros::ServiceClient srvVisionComm = nh.serviceClient<keti_msgs::VisionComm>("keti_vision_comm");
    keti_msgs::VisionComm visionComm;

    actionlib::SimpleActionClient<keti_robot::RobotMoveAction> acRobot("robot_move_action", true);
    keti_robot::RobotMoveGoal goalRobot;

    actionlib::SimpleActionClient<keti_gripper::GripperMoveAction> acGripper("keti_gripper_node", true);
    keti_gripper::GripperMoveGoal goalGripper;
    
    ros::Rate loop_rate(5);

    robot_move_complete = false;
    robot_moving = false;

    gripper_move_complete = false;
    gripper_moving = false;

    acRobot.waitForServer();
    acGripper.waitForServer();

    state = State::Idle;

    // Move to init joint position
    double init_joint[6] = {0, 0, M_PI_2, 0, M_PI_2, 0};
    goalRobot.cmd = 1;
    goalRobot.num = 1;
    goalRobot.value.clear();
    for(int i = 0; i < 6; i++){
        goalRobot.value.push_back(init_joint[i]);
    }
    acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle){
        usleep(1000);
    }

    // Move to camera shooting position
    double camera_pose[6] = {0.850, -0.245, 0.700, 0, M_PI, 0};
    // double camera_joint[6] = {-14.72, 45.64, 68.48, 0, 63.61, -14.72};
    goalRobot.cmd = 2;
    goalRobot.num = 1;
    goalRobot.value.clear();
    for(int i = 0; i < 6; i++){
        goalRobot.value.push_back(camera_pose[i]);
    }
    acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle){
        usleep(1000);
    }

    // Get cmera data(communication with vision application)
    ROS_INFO("Wait return vision module");
    if (srvVisionComm.call(visionComm))
    {
        ROS_INFO("Succeed to call service vision request");
        ROS_INFO("vision state : %d", visionComm.response.connected);
        if(visionComm.response.value.size() >= 3){
            ROS_INFO("vision data : %f, %f, %f", visionComm.response.value[0], visionComm.response.value[1], visionComm.response.value[2]);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service vision request");
    }

    if(!visionComm.response.connected){
        ROS_ERROR("Disconnected vision socket server");
        visionComm.response.value.clear();
        visionComm.response.value.push_back(0);
        visionComm.response.value.push_back(0);
        visionComm.response.value.push_back(0);
    }

    // Move to pick position
    double pick_pose[6] = {0,};
    double offset[3] = {0.0818452352279836, -0.023331711769110135, 0.019284411871427095};

    double pick_pose_wp1[6] = {0.85, -0.245, 0.35, 0, M_PI, 0};
    double pick_pose_wp2[6] = {0.931071, -0.267948, 0.35, 0, M_PI, 0};
    double pick_pose_wp3[6] = {0.931071, -0.267948, 0.22, 0, M_PI, 0};
    
    goalRobot.cmd = 2;
    goalRobot.num = 1;
    goalRobot.value.clear();
    for(int j = 0; j < 6; j++){
        goalRobot.value.push_back(pick_pose_wp1[j]);
    }
    acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);

    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle){
        usleep(1000);
    }

    goalRobot.cmd = 2;
    goalRobot.num = 1;
    goalRobot.value.clear();
    for(int j = 0; j < 6; j++){
        goalRobot.value.push_back(pick_pose_wp2[j]);
    }
    acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);

    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle){
        usleep(1000);
    }

    goalRobot.cmd = 2;
    goalRobot.num = 1;
    goalRobot.value.clear();
    for(int j = 0; j < 6; j++){
        goalRobot.value.push_back(pick_pose_wp3[j]);
    }
    acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);

    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle){
        usleep(1000);
    }

    // Gripper grip
    goalGripper.cmd = 1;
    goalGripper.width = 5;
    goalGripper.force = 20;
    goalGripper.speed = 10;
    acGripper.sendGoal(goalGripper, &GripperMoveCompleteCallback, &GripperMovingActiveCallback, &GripperMovingFeedbackCallback);
    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle){
        usleep(1000);
    }

    // Move to place position
    double place_pose_wp1[6] = {0.931071, -0.267948, 0.4, 0, M_PI, 0};
    double place_pose_wp2[6] = {0.971071, 0.307948, 0.4, 0, M_PI, 0};
    double place_pose_wp3[6] = {0.971071, 0.307948, 0.22, 0, M_PI, 0};
    
    goalRobot.cmd = 3;
    goalRobot.num = 3;
    goalRobot.value.clear();
    for(int j = 0; j < 6; j++){
        goalRobot.value.push_back(place_pose_wp1[j]);
    }
    for(int j = 0; j < 6; j++){
        goalRobot.value.push_back(place_pose_wp2[j]);
    }
    for(int j = 0; j < 6; j++){
        goalRobot.value.push_back(place_pose_wp3[j]);
    }
    acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle){
        usleep(1000);
    }

    // Gripper release
    goalGripper.cmd = 2;
    goalGripper.width = 39;
    goalGripper.force = 0;
    goalGripper.speed = 0;
    acGripper.sendGoal(goalGripper, &GripperMoveCompleteCallback, &GripperMovingActiveCallback, &GripperMovingFeedbackCallback);
    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle){
        usleep(1000);
    }

    // Move to init position
    goalRobot.cmd = 2;
    goalRobot.num = 1;
    goalRobot.value.clear();
    for(int i = 0; i < 6; i++){
        goalRobot.value.push_back(place_pose_wp2[i]);
    }
    acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle){
        usleep(1000);
    }

    goalRobot.cmd = 1;
    goalRobot.num = 1;
    goalRobot.value.clear();
    for(int i = 0; i < 6; i++){
        goalRobot.value.push_back(init_joint[i]);
    }
    acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle){
        usleep(1000);
    }
}