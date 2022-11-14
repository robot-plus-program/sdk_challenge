#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <keti_msgs/RobotState.h>
#include <keti_robot/RobotMoveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

static cv::Mat current_frame;
static rs2::frameset data;
static rs2_intrinsics intrin;
static float point[3], pixel[2], depth, robot_point[3];
static double init_joint[6] = {0, 0, M_PI_2, 0, M_PI_2, 0};

static int robot_state = 0;
static double current_joint[6];
static double current_position[3];
static double current_rotation[3];
static double current_T_matrix[16];

static bool robot_move_complete = false;
static bool robot_moving = false;

enum State{Idle=0, MoveWait, RobotMoveJ, RobotMoveL, RobotMoveB, GripperMoveGrip, GripperMoveRelease, Stop};
static int state = 0;

keti_robot::RobotMoveGoal goalRobot;
actionlib::SimpleActionClient<keti_robot::RobotMoveAction> *acRobot;

void RobotStateCallback(const keti_msgs::RobotState &msg){
    robot_state = msg.robot_state;
    memcpy(current_joint, msg.current_joint.data(), sizeof(double)*6);
    memcpy(current_position, msg.current_position.data(), sizeof(double)*3);
    memcpy(current_rotation, msg.current_rotation.data(), sizeof(double)*3);
    memcpy(current_T_matrix, msg.current_T_matrix.data(), sizeof(double)*16);

    // ROS_INFO("robot_state : %d", robot_state);
    // ROS_INFO("current joint : %f, %f, %f, %f, %f, %f", current_joint[0], current_joint[1], current_joint[2], current_joint[3], current_joint[4], current_joint[5]);
    // ROS_INFO("current position : %f, %f, %f", current_position[0], current_position[1], current_position[2]);
    // ROS_INFO("current rotation : %f, %f, %f", current_rotation[0], current_rotation[1], current_rotation[2]);
    // ROS_INFO("current_T_matrix 1: %f, %f, %f, %f", current_T_matrix[0], current_T_matrix[1], current_T_matrix[2], current_T_matrix[3]);
    // ROS_INFO("current_T_matrix 2: %f, %f, %f, %f", current_T_matrix[4], current_T_matrix[5], current_T_matrix[6], current_T_matrix[7]);
    // ROS_INFO("current_T_matrix 3: %f, %f, %f, %f", current_T_matrix[8], current_T_matrix[9], current_T_matrix[10], current_T_matrix[11]);
    // ROS_INFO("current_T_matrix 4: %f, %f, %f, %f", current_T_matrix[12], current_T_matrix[13], current_T_matrix[14], current_T_matrix[15]);
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
        // ROS_INFO("Robot don't move");
    }
    else if(state == 2){
        robot_moving = true;
        robot_move_complete = false;
        // ROS_INFO("Robot moving");
    }
}

void RobotMovingActiveCallback(){
    ROS_INFO("Goal just went active");
    robot_moving = false;
    robot_move_complete = false;

    state = State::MoveWait;
}

void movej(double cmd_joint[6], bool move_wait = true){
    goalRobot.cmd = 1;
    goalRobot.num = 1;
    goalRobot.value.clear();
    for(int i = 0; i < 6; i++){
        goalRobot.value.push_back(cmd_joint[i]);
    }
    acRobot->sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle && move_wait){
        usleep(1000);
    }
}

void movel(double cmd_pose[6], bool move_wait = true){
    goalRobot.cmd = 2;
    goalRobot.num = 1;
    goalRobot.value.clear();
    for(int i = 0; i < 6; i++){
        goalRobot.value.push_back(cmd_pose[i]);
    }
    acRobot->sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle && move_wait){
        usleep(1000);
    }
}

void moveb(double *cmd_pose, int num, bool move_wait = true){
    goalRobot.cmd = 3;
    goalRobot.num = 3;
    goalRobot.value.clear();
    for(int i = 0; i < num; i++){
        for(int j = 0; j < 6; j++){
            goalRobot.value.push_back(cmd_pose[i*6 + j]);
        }
    }
    acRobot->sendGoal(goalRobot, &RobotMoveCompleteCallback, &RobotMovingActiveCallback, &RobotMovingFeedbackCacllback);
    while(state != State::MoveWait){
        usleep(1000);
    }
    while(state != State::Idle && move_wait){
        usleep(1000);
    }
}

void mouse_event(int event, int x, int y, int flags, void*)
{
    if(event == cv::EVENT_LBUTTONDOWN){
        ROS_INFO("x : %d, y : %d", x, y);
        ROS_INFO("z : %f", data.get_depth_frame().get_distance(x, y));
        pixel[0] = x;
        pixel[1] = y;
        depth = data.get_depth_frame().get_distance(x, y);
        rs2_deproject_pixel_to_point(point, &intrin, pixel, depth);
        ROS_INFO("camera x : %f, y : %f, z : %f", point[0], point[1], point[2]);
        robot_point[0]=-point[1]+(80.6/1000);
        robot_point[1]=-point[0]+(30.09/1000);
        robot_point[2]=-point[2]+(72.5/1000);
        ROS_INFO("robot x : %f, y : %f, z : %f", robot_point[0], robot_point[1], robot_point[2]);
    }
}

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
        ros::shutdown();
        printf("\n Finished Program \n");
        exit(1);
    }
}

int main(int argc, char **argv)
{
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    ros::init(argc, argv, "keti_calibration_sample_node");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    ros::Rate rate(10);

    ros::Subscriber subRobotState = nh.subscribe("/keti_robot_state", 1, RobotStateCallback);

    // actionlib::SimpleActionClient<keti_robot::RobotMoveAction> acRobot("robot_move_action", true);
    acRobot = new actionlib::SimpleActionClient<keti_robot::RobotMoveAction>("robot_move_action", true);

    robot_move_complete = false;
    robot_moving = false;

    acRobot->waitForServer();

    state = State::Idle;

    std::string window_name = "result";
    cv::namedWindow(window_name);
    cv::setMouseCallback(window_name, mouse_event);

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    pipe.start();
    intrin = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

        movej(init_joint);

        double camera_pose[6] = {0.850, 0.245, 0.700, 0, M_PI, 0};
        movel(camera_pose);
        
        memset(robot_point, 0, sizeof(float)*3);

        data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
    
        rs2::frame depth = data.get_color_frame().apply_filter(color_map);

        // Query frame size (width and height)
        int w = depth.as<rs2::video_frame>().get_width();
        int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        cv::Mat frame(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        frame.copyTo(current_frame);

        // Display the current frame
        if(!current_frame.empty())
            cv::imshow(window_name, current_frame);

        cv::waitKey(0);

        double goal_pose[6] = {current_position[0] + robot_point[0],
                                current_position[1] + robot_point[1],
                                current_position[2] + robot_point[2],
                                0, M_PI, 0};

        ROS_INFO("goal position x : %f, y : %f, z : %f", goal_pose[0], goal_pose[1], goal_pose[2]);
        if(goal_pose[2] < 0.21) goal_pose[2] = 0.21;
        movel(goal_pose);

        cv::waitKey(0);
    }

    // Close down OpenCV
    cv::destroyWindow(window_name);

    pipe.stop();
}