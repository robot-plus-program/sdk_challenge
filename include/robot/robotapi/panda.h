#ifndef PANDA_H
#define PANDA_H


#include <iostream>
#include "../robotconf.h"
#include "../libcustom/tcpclient.h"
#include "robot.h"

const int MAXCONNECTIONS = 5;
const int MAXWAITBUFSIZE = 4096;
const int MAXSENDBUFSIZE = 4096;
const int MAXRECEIVEBUFSIZE = 4096;
const int RECVBUFSIZE = 491 + 3;
const int SENDBUFSIZE = 1162 + 2;

const uint8_t SOP_TX = 0x5D;
const uint8_t SOP_RX = 0x5E;
const uint8_t EOP = 0x5F;
const uint8_t CMD_TEMO = 0x30;
const uint8_t CMD_INIT = 0x31;
const uint8_t CMD_STOP = 0x32;
const uint8_t CMD_JOINT_MOVE = 0x33;
const uint8_t CMD_POSE_MOVE = 0X34;
const uint8_t CMD_STATE = 0x36;

#pragma pack(push, 1)
typedef union{
    struct{
        double O_T_EE[16];
        double O_T_EE_d[16];
        double q[7];
        double q_d[7];
        double dq[7];
        double dq_d[7];

        char current_erros;
        char last_motion_erros;

        char robot_mode;
        double time_ms;
    }robot_state;
    char char_data[491];
}RobotStateData;

typedef union{
    struct{
        char cmd;
        char num;
        double joint[7*5];
        double pose[16*5];
        double vel[5];
        double time[5];
        double acc_time[5];
    }data;
    char char_data[1162];
}CmdData;

#pragma pack(pop)

class panda : public robot
{
    TCPClient *tcpClient;
    double vel;
public:
    panda();

    void InitSocket(TCPClient *sock1,TCPClient *sock2,ReciveData *Info,int cord_type);
    void MoveL(WayPoints *data);
    void MoveJ(WayPoints *data);
    void MoveB(WayPoints *data);
    void MoveC(WayPoints *data);
    void RobotInfo();

    void Stop();
    //acc[deg/s^2],vel[deg/s]
    //acc[mm/s^2],vel[mm/s]
    void SetVelocity(double v);
    void settcp(bool on);


    ReciveData *recvsock;

     char bufSend[MAXSENDBUFSIZE] = {0,};
private:
    RobotStateData recvRobotState;
    CmdData cmdData;
};

#endif // PANDA_H
