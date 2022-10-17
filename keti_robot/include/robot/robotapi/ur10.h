#ifndef UR10_H
#define UR10_H

#include "qstring.h"

#include "../robotconf.h"
#include "../libcustom/tcpclient.h"
#include "robot.h"

#pragma pack(push,1)
struct Realtime5p9
{
    int MessageSize;
    double Time;
    double q_target[6];
    double qd_target[6];
    double qdd_target[6];
    double I_target[6];
    double M_target[6];
    double q_actual[6];
    double qd_actual[6];
    double I_actual[6];
    double I_control[6];
    double Tool_vector_act[6];
    double TCP_Spd_Act[6];
    double TCP_Foce[6];
    double Tool_vector_tar[6];

    double TCP_Spd_tar[6];
    double DI_bits;
    double Motor_Temp[6];
    double Controller_Timer;
    double Test_value;
    double RobotMode;
    double JointMode[6];
    double SafetyMode;
    double GNUPlot_103to108[6];
    double Tool_Acc_values[3];
    double GNUPlot_112to117[6];
    double Speed_scaling;
    double LinearMomentumNorn;
    double GNUPlot_120;

    double GNUPlot_121;
    double V_main;
    double V_robot;
    double I_robot;
    double V_act[6];
    double DO;
    double ProgramState;
    double ElbowPos[3];
    double ElbowVel[3];
    double SafetyStatus;
    double GNUPlot_140;
    double GNUPlot_141;
    double GNUPlot_142;
};
#pragma pack(pop)



class ur10
{
    TCPClient *cmdSocket;
    TCPClient *dataSocket;
    Realtime5p9 urdata;

    QString blendMsg;
    ReciveData *RecvData={};
public:
    ur10();

    cord TCPcord={};
    void movel(float x, float y, float z, float rx, float ry, float rz,float acc=1.2,float vel=.25, float time=0);
    void movej(float j1, float j2, float j3, float j4, float j5, float j6,float acc=1.2,float vel=0.25, float time=0);
    void moveb(WayPoints *data);
    void InitSocket(TCPClient *sock1,TCPClient *sock2,ReciveData *Info,int cord_type);
    void RobotInfo();
    void moveb_clear(void);
    void moveb_addpoint(float r,float x, float y, float z, float rx, float ry, float rz,float acc,float vel);
    void moveb_move();
};

#endif // UR10_H
