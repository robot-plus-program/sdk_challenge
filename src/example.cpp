#include <iostream>
#include <pthread.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include "sdk.h"

#include "zimmergripper.h"

static struct sigaction sigIntHandler;
static int sig = 0;

enum State{Wait=1, Moving};
enum Cmd{RobotMoveJ = 1, RobotMoveL, RobotMoveB, GripperMoveGrip, GripperMoveRelease, RobotStop, GripperStop};
static bool robot_connected = false;
static int state = 0;
static int cmd = 0;
static double current_joint[6] = {0,};
static double current_T_matrix[16] = {0,};

static ZimmerGripper gripper;
static bool gripper_connected = false;

static pthread_t key_input_thread;
static pthread_t data_update_thread;

static void* key_input_func(void *arg){
    char key_value = 0;

    while(robot_connected && gripper_connected){
        std::cout << "\n Endter character and press \"Enter\"" << std::endl;
        std::cout << " 1 : Robot move joint motion" << std::endl;
        std::cout << " 2 : Robot move Cartesian motion" << std::endl;
        std::cout << " 3 : Robot move Cartesian motion with blend" << std::endl;
        std::cout << " 4 : Gripper move(grip)" << std::endl;
        std::cout << " 5 : Gripper move(release)" << std::endl;

        std::cin >> key_value;

        switch(key_value){
            case '1':
                cmd = Cmd::RobotMoveJ;
                break;
            case '2':
                cmd = Cmd::RobotMoveL;
                break;
            case '3':
                cmd = Cmd::RobotMoveB;
                break;
            case '4':
                cmd = Cmd::GripperMoveGrip;
                break;
            case '5':
                cmd = Cmd::GripperMoveRelease;
                break;
            default :
                break;
        }

        while(cmd != 0)
            usleep(1000);
    }
    return NULL;
}

static void* data_update_func(void* arg){
    while(robot_connected){
        sdk_info robotInfor = RobotInfo();

        if(robotInfor.state == 2) {
            state = State::Moving;
            cmd = 0;
        }
        else if(robotInfor.state == 1){
            state = State::Wait;
        }

        memcpy(current_joint, robotInfor.jnt, sizeof(double)*6);
        memcpy(current_T_matrix, robotInfor.mat, sizeof(double)*6);

        usleep(10000);
    }

    return nullptr;
}

void my_handler(int s)
{
    if (sig == 0)
    {
        std::cout << "\n\tCatched Ctrl+c\t" << std::endl;
        sig = s;

        gripper_connected = false;
        gripper.disconnect();

        robot_connected = false;
        pthread_cancel(key_input_thread);
        pthread_cancel(data_update_thread);
        RobotDisconnect();
        exit(1);
    }
}

int main(int argc, char **argv){
    if(argc != 3){
        std::cout << "\n\nPlease check the input arguments!!\n\n" << std::endl;
        return 0;
    }
    std::string robot_ip = argv[1];
    std::string gripper_ip = argv[2];

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    SetRobotConf(RB10, robot_ip.c_str(), 5000);
    robot_connected = RobotConnect();
    // robot_connected = true;

    gripper.connect(gripper_ip.c_str(), 5002);
    gripper_connected = gripper.isConnected();
    if(gripper_connected)
        gripper.gripper_init();

    std::cout << "wait..." << std::endl;

    pthread_create(&key_input_thread, NULL, key_input_func, NULL);

    pthread_create(&data_update_thread, NULL, data_update_func, NULL);
    
    double cmd_joint[2][6] = {{0, 0, 0, 0, 0, 0},
                            {0, 0, M_PI_2, 0, M_PI_2, 0}};

    double cmd_rot[9] = {0, 0, 1, 1, 0, 0, 0, 1, 0};
    double cmd_pos[5][3] = {{0.787323, -0.356279, 0.695886}, 
                            {0.787323, -0.356279, 0.395886}, 
                            {0.787323, 0.356279, 0.395886}, 
                            {0.787323, 0.356279, 0.695886},
                            {0.687323, -0.156279, 0.695886}};

    int cnt_joint = 1;
    int cnt_pose = 0;

    while(robot_connected && gripper_connected){
        // std::cout << "state : " << state << std::endl;
        // std::cout << "cnt : " << cnt_joint << std::endl;
        if(state == State::Wait)
        {
            switch(cmd)
            {
                case Cmd::RobotMoveJ:
                {
                    movej(cmd_joint[cnt_joint%2]);
                    cnt_joint++;
                    cmd = 0;

                    break;
                }
                case Cmd::RobotMoveL:
                {
                    double cmd_mat[16] = {0,};
                    for(unsigned int i = 0; i < 3; i++){
                        for(unsigned int j = 0; j < 3; j++){
                            cmd_mat[i*4 + j] = cmd_rot[i*3 + j];
                        }
                    }
                    cmd_mat[3] = cmd_pos[cnt_pose%5][0];
                    cmd_mat[7] = cmd_pos[cnt_pose%5][1];
                    cmd_mat[11] = cmd_pos[cnt_pose%5][2];
                    cmd_mat[15] = 1;

                    std::cout << "cmd T matrix : " << std::endl;
                    for(unsigned int i = 0; i < 16; i++){
                        std::cout << cmd_mat[i] << ((i%4) == 3 ? "\n" : "\t");
                    }
                    std::cout << std::endl;

                    movel(base, cmd_mat);
                    cnt_pose++;
                    cmd = 0;

                    break;
                }
                case Cmd::RobotMoveB:
                {
                    double cmd_mat[5][16] = {0,};

                    for(unsigned int num = 0; num < 5; num++){
                        for(unsigned int i = 0; i < 3; i++){
                            for(unsigned int j = 0; j < 3; j++){
                                cmd_mat[num][i*4 + j] = cmd_rot[i*3 + j];
                            }
                        }

                        cmd_mat[num][3] = cmd_pos[num][0];
                        cmd_mat[num][7] = cmd_pos[num][1];
                        cmd_mat[num][11] = cmd_pos[num][2];
                        cmd_mat[num][15] = 1;

                        std::cout << "cmd T matrix " << num + 1 << " : " << std::endl;
                        for(unsigned int i = 0; i < 16; i++){
                            std::cout << cmd_mat[num][i] << "\t";
                        }
                        std::cout << std::endl;
                    }

                    moveb(base, 1.0, 5, cmd_mat[0], cmd_mat[1], cmd_mat[2], cmd_mat[3], cmd_mat[4]);
                    cmd = 0;

                    break;
                }
                case Cmd::GripperMoveGrip:
                {
                    gripper.gripper_grip();
                    cmd = 0;
                    break;
                }
                case Cmd::GripperMoveRelease:
                {
                    gripper.gripper_release();
                    cmd = 0;
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        usleep(1000);
    }

    std::cout << "finish" << std::endl;

    return 0;
}