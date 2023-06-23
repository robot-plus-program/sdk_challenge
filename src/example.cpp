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
enum Cmd{RobotMoveJ = 1, RobotMoveL, RobotMoveB, GripperMoveGrip, GripperMoveRelease};
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
        // cout << "current_state : " << robotInfor.state << endl;;
        // cout << "current_joint : " << endl;
        // cout << robotInfor.jnt[0] << ", " << robotInfor.jnt[1] << ", " << robotInfor.jnt[2] << ", " << robotInfor.jnt[3] << ", " << robotInfor.jnt[4] << ", " << robotInfor.jnt[5] << endl;

        // cout << "current_T_matrix : " << endl;
        // cout << robotInfor.mat[0] << ", " << robotInfor.mat[1] << ", " << robotInfor.mat[2] << ", " << robotInfor.mat[3] << endl;
        // cout << robotInfor.mat[4] << ", " << robotInfor.mat[5] << ", " << robotInfor.mat[6] << ", " << robotInfor.mat[7] << endl;
        // cout << robotInfor.mat[8] << ", " << robotInfor.mat[9] << ", " << robotInfor.mat[10] << ", " << robotInfor.mat[11] << endl;
        // cout << robotInfor.mat[12] << ", " << robotInfor.mat[13] << ", " << robotInfor.mat[14] << ", " << robotInfor.mat[15] << endl;

        if(robotInfor.state == 2) {
            state = State::Moving;
            cmd = 0;
        }
        else if(robotInfor.state == 1){
            state = State::Wait;
        }

        memcpy(current_joint, robotInfor.jnt, sizeof(double)*6);
        // memcpy(current_T_matrix, robotInfor.mat, sizeof(double)*16);

        // printf("current_state : %f", state);
        // printf("current_joint : ");
        // for(unsigned int i = 0; i < 6; i++){
        //     printf("%f ", current_joint[i]);
        // }
        // printf("\n");

        // printf("current_T_matrix : \n");
        // for(unsigned int i = 1; i <= 16; i++){
        //     printf("%f ", current_T_matrix[i-1]);
        //     if(i%4 == 0){
        //         printf("\n");
        //     }
        // }
        // printf("\n");

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
    if(argc != 4){
        std::cout << "\n\nPlease check the input arguments!!\n\n" << std::endl;
        return 0;
    }
    std::string robot_ip = argv[1];
    std::string gripper_ip = argv[2];
    int gripper_port = atoi(argv[3]);

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    SetRobotConf(RB10, robot_ip.c_str(), 5000);
    robot_connected = RobotConnect();

    gripper.connect(gripper_ip.c_str(), gripper_port);
    gripper_connected = gripper.isConnected();
    std::cout << "wait..." << std::endl;
    if(gripper_connected)
        gripper.gripper_init();


    pthread_create(&key_input_thread, NULL, key_input_func, NULL);

    pthread_create(&data_update_thread, NULL, data_update_func, NULL);
    
    double cmd_joint[2][6] = {{-M_PI_2, 0, M_PI_2, 0, M_PI_2, -M_PI},
                            {-M_PI_2, M_PI_4, M_PI_2, -M_PI_2, M_PI_2, 0}};

    double cmd_rot[9] = {-1, 0, 0, 
                        0, 0, 1, 
                        0, 1, 0};
    double cmd_pos[5][3] = {{-0.394545, -0.688601, 0.510546}, 
                            {-0.394545, -0.688601, 0.330724}, 
                            {0.2159, -0.787392, 0.330724}, 
                            {0.2159, -0.524593, 0.622947},
                            {-0.1579, -0.688602, 0.695799}};

    int cnt_joint = 0;
    int cnt_pose = 0;

    SetVelocity(20);
    
    while(robot_connected && gripper_connected){
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

                    moveb(base, 10, 5, cmd_mat[0], cmd_mat[1], cmd_mat[2], cmd_mat[3], cmd_mat[4]);
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