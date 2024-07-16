/*#ifndef SDK_H
#define SDK_H

#include "robotapi/rb10_v2.h"
#include "robotapi/indy7_v2.h"
#include "robotapi/m1013_v2.h"
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

const bool base=0;
const bool tcp=1;

//robot *Rot = new rb10;
//robot *Rot;
//string DataSockip;
//uint16_t DataSockport;
//string CMDSockip;
//uint16_t CMDSockport;
//ReciveData recvdata;
//TMatrixInfo recv_mat;

//TCPClient CMDScok;
//TCPClient DataSock;
//timer testimer;

struct sdk_info{
    double jnt[7];
    double mat[16];
    double state;
};
struct  DIInput
{
     bool bit[16];
};


void SubRobotInfo();

extern "C" {
    void movej(double *pnt);
    void movel(double ref, double *pnt);
    void moveb(double ref, double blend_on, double num_pnt,
                    double *num1=nullptr, double *num2=nullptr,
                    double *num3=nullptr, double *num4=nullptr, double *num5=nullptr);
    void movec(double ref, double *num1, double *num2);
    void Stop();
    void settcp(int on);
    void SetVelocity(double v);
    sdk_info RobotInfo();
    //////////////////////////////////////
    void SetRobotConf(int index,const char *_ip,int _port);
//    void SetRobotConf2(int index, const char *_ip,int _port,const char *_ip2,int _port2);
//    void RobotConnect();
    bool RobotConnect();
    void RobotDisconnect();
    void ControlBoxDigitalOut(int out);
//    DIInput ControlBoxDigitalInput();
    int ControlBoxDigitalIn();
    bool IsConnected();
}

#endif // SDK_H*/

