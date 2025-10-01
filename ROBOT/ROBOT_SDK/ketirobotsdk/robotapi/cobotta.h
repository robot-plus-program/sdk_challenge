#ifndef COBOTTA_H
#define COBOTTA_H

//#include "qstring.h"


#include <iostream>
#include "../robotconf.h"
#include "robot.h"

#include "densoapi/include/bCAPClient/bcap_client.h"

#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <algorithm>

class cobotta : public robot
{
    RConf Conf;
public:
    cobotta();

    int fd;
    HRESULT hr, hrOpen, hrCtrl, hrRobot;
	uint32_t hCtrl, hRobot;

    ReciveData *RecvData;
    WayPoints pos={};
    bool connected;

    void InitSocket(TCPClient *sock1,TCPClient *sock2,ReciveData *Info,int cord_type);
    void MoveL(WayPoints *data);
    void MoveJ(WayPoints *data);
    void MoveB(WayPoints *data);
    void MoveC(WayPoints *data);
    void RobotInfo();
	bool RobotConnect(std::string ip, int port, ReciveData *Info);
	void RobotDisconnect();
    bool IsConnected(){return connected;}

    void GripperGrip();
    void GripperRelease();
    void GripperWidth(double *width);

    void Stop();
    void SetVelocity(double v);
	void settcp(bool on);
	bool WaitMove();

	void ControlBoxDigitalOut(int out);
	int ControlBoxDigitalIn(void);

private:
	VARIANT vntParam, vntRet;
	BSTR bstrCommand, bstrOption;

	void OpenClient(std::string ip, int port);
	void ServiceStart();
	void ControllerConnect();
	void ClearError();
	void ClearErrorInit();
	void Preparation();
	void ControllerGetRobot();
	void TakeArm();
	void GiveArm();
	void MotorOnOff(int32_t flag);
};

extern cobotta cobo;

#endif // COBOTTA_H
