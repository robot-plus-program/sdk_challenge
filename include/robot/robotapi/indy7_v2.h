#ifndef indy7_H
#define indy7_H

//#include "qstring.h"
#include "../libcustom/tcpclient.h"

#include <iostream>
#include "../robotconf.h"
#include "robot.h"

#pragma pack(push,1)
struct 	DCPheader// 56byte
{
    char robotName[20];
    char robotVersion[12];
    unsigned char stepInfo;
    unsigned char sof;		//source of Frame
    int invokeId;
    int dataSize;
    uint32_t status;	//4byte
    char reserved[6];	//6byte
    int cmdId;
};

union DCPData
{
    double doubleData[25];
    int intData[50];
    char charData[200];
};
struct DCPInfo
{
    DCPheader header;
    DCPData data;
};
struct ExtDCPheader// 56byte
{
    char robotName[20];
    char robotVersion[12];
    unsigned char stepInfo;
    unsigned char sof;		//source of Frame
    int invokeId;
    int dataSize;
    uint32_t status;	//4byte
    char reserved[6];	//6byte
    int cmdId;
};
union ExtDCPData
{
    double doubleData[1*480000];
    int intData[4*480000];
    char charData[8*480000];
};
struct ExtDCPInfo
{
    ExtDCPheader header;
    ExtDCPData data;
};

#pragma pack(pop)

union DCPMsg{

    char byte[256];
    DCPInfo DCP;

};

union ExtDCP
{
    char byte[3840058];
    ExtDCPInfo value;
};

//char buf[8*480000];

class indy7 : public robot
{

    int invoke;
    DCPMsg msg={};
    ExtDCP Extmsg={};
    DCPheader RecvHeader={};

    RConf Conf;

    TCPClient *IndyTCP;


public:

    indy7();
    ~indy7();

    ReciveData *RecvData={};
    void InitSocket(TCPClient *sock1,TCPClient *sock2,ReciveData *Info,int cord_type);
    void InitSocket(TCPClient *sock,ReciveData *Info,int cord_type);
    void MoveL(WayPoints *data);
    void MoveJ(WayPoints *data);
    void MoveB(WayPoints *data);

    void RobotInfo();


    void JointTaskTo(double j1, double j2, double j3, double j4, double j5, double j6);
    void moveTaskTo(double x, double y,double z,double u,double v,double w);
    void AddTaskWaypointSet(double x, double y,double z,double u,double v,double w ,double r=0);
    void ClearTaskWaypointSet();
    void ExecuteTaskWaypointSet();
    void SetBrakeOn();
    void SetBrakeOff();
    void SetServoOn();
    void SetServoOff();
    void JointMoveTo();
    void SetTatskMoveBaseMode(int TaskMode);

    void MoveTCP(float x, float y, float z, float rx, float ry, float rz, float spd=300, float acc=100);

    void ext_trajmove(WayPoints *data);

    void GetTaskPos();
    void GetJointPos();
    void RobotFinishMotion();
    void RobotIsBusy();
    void SetDefaultTCP(cord TCPcord={});

};

#endif // indy7_H
