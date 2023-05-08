#ifndef M1013_H
#define M1013_H

#include "qstring.h"
#include <QtNetwork>

#include <iostream>
#include "../robotconf.h"



class m1013
{
    QTcpSocket *m1013CmdTCP;

public:
    m1013();

    ReciveData *sdata;
    WayPoints pos={};

    void RobotInfo();
    void InitSocket(QTcpSocket *cmdsock,ReciveData *Info,int cord_type);

};

#endif // M1013_H
