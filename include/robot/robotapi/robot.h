#ifndef ROBOT_H
#define ROBOT_H


#include <iostream>
#include <vector>

#include "../libcustom/tcpclient.h"
#include "../robotconf.h"
#include <unistd.h>

using namespace std;


class robot
{
public:

    robot();
     ~robot();
    virtual void InitSocket(TCPClient *sock1,TCPClient *sock2,ReciveData *Info,int cord_type);
    virtual void MoveL(WayPoints *data);
    virtual void MoveJ(WayPoints *data);
    virtual void MoveB(WayPoints *data);
    virtual void MoveC(WayPoints *data);

    virtual void Stop();
    virtual void SetVelocity(double v);
    virtual void CobotInit();
    virtual bool WaitMove();

    virtual void RobotInfo();
    virtual void settcp(bool on);
//    virtual void SetTCP();



    virtual void ControlBoxDigitalOut(int out);
    virtual array<bool,8> ControlBoxDigitalInput(void);
//    virtual vector<bool> ControlBoxDigitalIn(void);
    virtual double ControlBoxDigitalIn(void);
    TCPClient *CMDSocket;
private:


};


#endif // ROBOT_H
