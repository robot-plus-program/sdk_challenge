#include <ros/ros.h>
#include <keti_msgs/RobotState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <keti_robot/RobotMoveAction.h>
#include <keti_gripper/GripperMoveAction.h>

static int robot_state = 0;
static double current_joint[6];
static double current_position[3];
static double current_rotation[3];
static double current_T_matrix[16];
static bool print_enable = true;
static bool robot_move_complete = false;
static bool robot_moving = false;;
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

static void* key_input_func(void *arg){
    char key_value = 0;

    while(ros::ok()){
        if(state == State::Idle){
            ROS_INFO("\n\n Enter character and press \"Enter\"\n 1 : Robot move joint motion\n 2 : Robot move cartesian motion\n 3 : Robot move cartesian motion with balend\n 4 : Gripper move(grip)\n 5 : Gripper move(release)\n 6 : Robot & Gripper stop\n");

            std::cin >> key_value;

            switch(key_value){
                case '1':
                    state = State::RobotMoveJ;
                    break;
                case '2':
                    state = State::RobotMoveL;
                    break;
                case '3':
                    state = State::RobotMoveB;
                    break;
                case '4':
                    state = State::GripperMoveGrip;
                    break;
                case '5':
                    state = State::GripperMoveRelease;
                    break;
                case '6':
                    state = State::Stop;
                    break;
                default :
                    break;
            }
        }
    }
    return NULL;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "keti_example_node");

    ros::NodeHandle nh;
    std::string mode;
    nh.getParam("mode", mode);
    ROS_INFO("mode : %s", mode.c_str());
    ros::Subscriber subRobotState = nh.subscribe("/keti_robot_state", 1, RobotStateCallback);

    bool real;
    if(mode.compare("real")){
        real = false;
    }
    else{
        real = true;
    }

    actionlib::SimpleActionClient<keti_robot::RobotMoveAction> acRobot(real ? "robot_move_action" : "robot_move_sim_action", true);
    keti_robot::RobotMoveGoal goalRobot;

    ros::Rate loop_rate(5);

    double cmd_joint[2][6] = {{0, 0, M_PI_2, 0, 0, M_PI_2},
                            {0, 0, M_PI_2, 0, M_PI_2, 0}};
    double cmd_pose[5][6] = {{0.8, 0.2, 0.6, 0, M_PI, 0},
                            {0.8, -0.2, 0.6, 0, M_PI, 0},
                            {0.8, -0.2, 0.3, 0, M_PI, 0},
                            {0.8, 0.2, 0.3, 0, M_PI, 0},
                            {0.8, 0.0, 0.6, 0, M_PI, 0}};

    robot_move_complete = false;
    robot_moving = false;

    gripper_move_complete = false;
    gripper_moving = false;

    acRobot.waitForServer();
    
    actionlib::SimpleActionClient<keti_gripper::GripperMoveAction> *acGripper;
    if(real){
        acGripper = new actionlib::SimpleActionClient<keti_gripper::GripperMoveAction>("keti_gripper_node", true);
        acGripper->waitForServer();
    }
    keti_gripper::GripperMoveGoal goalGripper;

    state = State::Idle;

    pthread_t key_input_thread;
    pthread_create(&key_input_thread, NULL, key_input_func, NULL);
    pthread_detach(key_input_thread);

    int cnt_joint = 0;
    int cnt_pose = 0;

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();

        switch(state){
            case State::Idle:
            {
                break;
            }
            case State::MoveWait:
            {
                break;
            }
            case State::RobotMoveJ:
            {
                goalRobot.cmd = 1;
                goalRobot.num = 1;
                goalRobot.value.clear();
                for(int i = 0; i < 6; i++){
                    goalRobot.value.push_back(cmd_joint[cnt_joint%2][i]);
                }
                acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
                cnt_joint++;
                break;
            }
            case State::RobotMoveL:
            {
                goalRobot.cmd = 2;
                goalRobot.num = 1;
                goalRobot.value.clear();
                for(int i = 0; i < 6; i++){
                    goalRobot.value.push_back(cmd_pose[cnt_pose%5][i]);
                }
                acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
                cnt_pose++;
                break;
            }
            case State::RobotMoveB:
            {
                goalRobot.cmd = 3;
                goalRobot.num = 5;
                goalRobot.value.clear();
                for(int i = 0; i < 5; i++){
                    for(int j = 0; j < 6; j++){
                        goalRobot.value.push_back(cmd_pose[i][j]);
                    }
                }
                acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
                cnt_pose++;
                break;
            }
            case State::GripperMoveGrip:
            {
                if(real){
                    goalGripper.cmd = 1;
                    goalGripper.width = 5;
                    goalGripper.force = 20;
                    goalGripper.speed = 10;
                    acGripper->sendGoal(goalGripper, &GripperMoveCompleteCallback, &GripperMovingActiveCallback, &GripperMovingFeedbackCallback);
                }
                else{
                    goalRobot.cmd = 4;
                    acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
                }

                break;
            }
            case State::GripperMoveRelease:
            {
                if(real){
                    goalGripper.cmd = 2;
                    goalGripper.width = 39;
                    goalGripper.force = 0;
                    goalGripper.speed = 0;
                    acGripper->sendGoal(goalGripper, &GripperMoveCompleteCallback, &GripperMovingActiveCallback, &GripperMovingFeedbackCallback);
                }
                else{
                    goalRobot.cmd = 5;
                    acRobot.sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
                }

                break;
            }
            case State::Stop:
            {
                acRobot.cancelAllGoals();
                if(real){
                    acGripper->cancelAllGoals();
                }
                state = State::Idle;
            }
            default:
            {
                break;
            }
        }
    }

    delete acGripper;
}