#ifndef M1013_H
#define M1013_H

//#include "qstring.h"


#include <iostream>
#include "../robotconf.h"
#include "../libcustom/tcpclient2.h"
#include "robot.h"

#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>

#include "doosanapi/include/DRFLEx.h"
using namespace DRAFramework;

#undef NDEBUG
#include <assert.h>

extern CDRFLEx Drfl;

extern void OnTpInitializingCompleted();
extern void OnHommingCompleted();
extern void OnProgramStopped(const PROGRAM_STOP_CAUSE);
extern void OnMonitoringDataCB(const LPMONITORING_DATA pData);
extern void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData);
extern void OnMonitoringCtrlIOCB(const LPMONITORING_CTRLIO pData);
extern void OnMonitoringCtrlIOExCB(const LPMONITORING_CTRLIO_EX pData);
extern void OnMonitoringStateCB(const ROBOT_STATE eState);
extern void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl);
extern void OnLogAlarm(LPLOG_ALARM tLog);
extern void OnTpPopup(LPMESSAGE_POPUP tPopup);
extern void OnTpLog(const char *strLog);
extern void OnTpProgress(LPMESSAGE_PROGRESS tProgress);
extern void OnTpGetuserInput(LPMESSAGE_INPUT tInput);
extern void OnRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData);
extern void OnDisConnected();

extern bool g_bHasControlAuthority;
extern bool g_TpInitailizingComplted;
extern bool g_mStat;
extern bool g_Stop;
extern bool moving;

class m1013 : public robot
{
    TCPClient *m1013CmdTCP;
    RConf Conf;
public:
    m1013();

    ReciveData *sdata;
    WayPoints pos={};
	double vel_l=50, vel_j=120.0f;
    double acc_l=75,acc_j=180;

    void InitSocket(TCPClient *sock1,TCPClient *sock2,ReciveData *Info,int cord_type);
    void MoveL(WayPoints *data);
    void MoveJ(WayPoints *data, double acc=-1);
    void MoveJ(WayPoints *data);
    void MoveB(WayPoints *data);
    void MoveC(WayPoints *data);
    void RobotInfo();
	bool RobotConnect(std::string ip, int port, ReciveData *Info);
	void RobotDisconnect();

    void Stop();
    void SetVelocity(double v);
	void settcp(bool on);
	bool WaitMove();

	void ControlBoxDigitalOut(int out);
	int ControlBoxDigitalIn(void);
    void set_speed_acc_j(double v,double a);

	void RobotComplianceCtrlOn(float stpx = 3000, float stpy = 3000, float stpz = 3000, float strx = 200, float stry = 200, float strz = 200);
	void RobotComplianceCtrlOff();
};

#endif // M1013_H
