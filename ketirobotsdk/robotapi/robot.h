#ifndef ROBOT_H
#define ROBOT_H


#include <iostream>
#include <vector>

#include "../libcustom/tcpclient2.h"
#include "../robotconf.h"
#include <unistd.h>

using namespace std;


class robot
{
public:

    int test_value=0;
    int robot_num=0;

    robot();
     ~robot();
    virtual void InitSocket(TCPClient *sock1,TCPClient *sock2,ReciveData *Info,int cord_type);
    virtual void MoveL(WayPoints *data);
    virtual void MoveJ(WayPoints *data, double acc);
    virtual void MoveJ(WayPoints *data);
    virtual void MoveB(WayPoints *data);
    virtual void MoveC(WayPoints *data);

    virtual void Stop();
    virtual void SetVelocity(double v);
    virtual void CobotInit();
    virtual bool WaitMove();

    virtual bool RobotConnect(std::string ip, int port, ReciveData *Info);
    virtual void RobotDisconnect();

    virtual void RobotInfo();
    virtual void settcp(bool on);
    virtual bool IsConnected();
    virtual void Pause();
    virtual void Resume();
//    virtual void SetTCP();



    virtual void ControlBoxDigitalOut(int out);
    virtual array<bool,8> ControlBoxDigitalInput(void);
//    virtual vector<bool> ControlBoxDigitalIn(void);
    virtual int ControlBoxDigitalIn(void);
    virtual void set_speed_acc_j(double v,double a);
    void testf(int i);
    void check();
    TCPClient *CMDSocket;
private:


};


#endif // ROBOT_H
