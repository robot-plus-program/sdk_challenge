#include <iostream>
#include <pthread.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include "sdkv2.h"

static struct sigaction sigIntHandler;
static int sig = 0;

sdk robot;

void my_handler(int s)
{
    if (sig == 0)
    {
        std::cout << "\n\tCatched Ctrl+c\t" << std::endl;
        sig = s;

        robot.Stop();
        robot.RobotDisconnect();
        exit(1);
    }
}

int main(){
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    std::string robot_ip = "192.168.3.101";

    robot.SetRobotConf(RB10, robot_ip.c_str(), 5000);
    robot.RobotConnect();


    robot.SetVelocity(30);
    double init_joint[6] = {-M_PI_2, 0, M_PI_2, 0, M_PI_2, 37 * M_PI / 180.0};
    robot.movej(init_joint);
    // robot.WaitMove();
    usleep(2000000);

    double test_joint[6][6] = {{-M_PI_2, 0, M_PI_2, 0, M_PI_2, 37 * M_PI / 180.0},
                            {-0.985085, 0.52292, 1.54011, -1.08979, 2.31368, 0.649478},
                            {-1.9056, 0.480037, 1.4883, -0.417243, 0.578421, 0.649854},
                            {-1.72464, -0.0158068, 1.95668, 0.602384, 1.44353, 0.649812},
                            {-1.52724, 0.766861, 1.18157, -1.13765, 1.57638, 0.649679},
                            {-M_PI_2, 0, M_PI_2, 0, M_PI_2, 37 * M_PI / 180.0}
                            };

    double test_pos[6][3] = {{0.575497, -0.688008, 0.572438},
                            {-0.781304, -0.688008, 0.572438},
                            {-0.15761, -0.688601, 0.694795},
                            {-0.157609, -1.20161, 0.343542},
                            {-0.15761, -0.421359, 0.343545},
                            {-0.15761, -0.688601, 0.694795}};
    double test_rot[9] = {0.799191, 0, 0.601077,
                        0.601077, 0, -0.799191,
                        0, 1, 0};
    
    double cmd_mat[16] = {0,};
    cmd_mat[15] = 1;
    for(unsigned int i = 0; i < 3; i++){
        for(unsigned int j = 0; j < 3; j++){
            cmd_mat[i*4 + j] = test_rot[i*3 + j];
        }
    }

    for(unsigned int i = 3; i < 6; i++){
        cmd_mat[3] = test_pos[i][0];
        cmd_mat[7] = test_pos[i][1];
        cmd_mat[11] = test_pos[i][2];
        robot.movel(0, cmd_mat);
        robot.WaitMove();
    }

    // for(unsigned int i = 0; i < 6; i++){
    //     robot.movej(test_joint[i]);
    //     robot.WaitMove();
    // }

    robot.RobotDisconnect();

    return 0;
}