#ifndef M1013_H
#define M1013_H

//#include "qstring.h"


#include <iostream>
#include "../robotconf.h"
#include "../libcustom/tcpclient.h"
#include "robot.h"



class m1013 : public robot
{
    TCPClient *m1013CmdTCP;
    RConf Conf;
public:
    m1013();

    ReciveData *sdata;
    WayPoints pos={};
    double vel_l=250,vel_j=14.3;


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
    void InitSocket(TCPClient *cmdsock,ReciveData *Info,int cord_type);

};

#endif // M1013_H
