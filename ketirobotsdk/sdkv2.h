#ifndef SKD_H
#define SKD_H

#include "robotapi/rb10_v2.h"
#include "robotapi/indy7_v2.h"
#include "robotapi/m1013_v3.h"
#include "robotapi/ur10_v2.h"
#include "robotapi/robot.h"

#include "robotconf.h"
#include "libcustom/tcpclient2.h"
#include "libcustom/timer.h"
#include "robotconf.h"

#define TestDummy 0
#define RB10    1
#define UR10    2
#define M1013   3
#define Indy7   4

struct sdk_info{
    double jnt[7];
    double mat[16];
    double state;
    double error;
};



class sdk
{
    int robot_id;
    int CurrentR, BeforeR=-1;

    double ChgMat[16];
    robot *Robot;
    string DataSockip;
    uint16_t DataSockport;
    string CMDSockip;
    uint16_t CMDSockport;

    int DataRecvInterval=100;
    bool DatasockUse=false;
    bool RobotConnectState=false;
    bool ReqInit=false;
    int CMDSockstate=-1,DataSockState=-1;

    TCPClient CMDScok;
    TCPClient DataSock;
    bool connection_state=false;

    ReciveData recvdata;
    TMatrixInfo recv_mat;

    RConf Conf;
    int rottype=zyz_deg;

    timer testimer;

public:

    sdk();
    ~sdk();

    void SetRobotConf(int index,const char *_ip,int _port);
    void RobotChangeFrom(int index);
    bool RobotConnect();
    void RobotDisconnect();

    void movej(double *pnt);
    void movej(double *pnt,double acc);
    void movel(double ref, double *pnt);
    void moveb(double ref, double blend_on, double num_pnt,
                    double *num1=nullptr, double *num2=nullptr,
                    double *num3=nullptr, double *num4=nullptr, double *num5=nullptr);
    void movec(double ref, double *num1, double *num2);
    void Stop();
    void SetVelocity(double v);
    sdk_info RobotInfo();

    void ControlBoxDigitalOut(int out);
    int ControlBoxDigitalIn();

    int IsConnected();
    void WaitMove();
    void set_speed_acc_j(double v,double a);

    void Resume();
    void Pause();
};

//sdk *sys;

extern "C"{
    int SetRobotConf(int index,const char *_ip,int _port);
    void RobotChangeFrom(int robot_id,int index);
    bool RobotConnect(int robot_id);
    void RobotDisconnect(int robot_id);

    void movej(int robot_id, double *pnt);
    void movel(int robot_id,double ref, double *pnt);
    void moveb(int robot_id,double ref, double blend_on, double num_pnt,
                    double *num1=nullptr, double *num2=nullptr,
                    double *num3=nullptr, double *num4=nullptr, double *num5=nullptr);
    void movec(int robot_id, double ref, double *num1, double *num2);
    void Stop(int robot_id);
    void SetVelocity(int robot_id,double v);
    sdk_info RobotInfo(int robot_id);

    void ControlBoxDigitalOut(int robot_id,int out);
    int ControlBoxDigitalIn(int robot_id);

    int IsConnected(int robot_id);
    void WaitMove(int robot_id);
    void set_speed_acc_j(int robot_id,double v,double a);

    void Pause(int robot_id);
    void Resume(int robot_id);
}

#endif // SKD_H
