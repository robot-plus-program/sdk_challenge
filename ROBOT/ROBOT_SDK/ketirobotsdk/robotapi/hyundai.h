#ifndef HYUNDAI_H
#define HYUNDAI_H

#include <string>
#include "../robotconf.h"
#include "../libcustom/tcpclient2.h"
#include "robot.h"
#include <iostream>
#include <curl/curl.h>
#include <curl/easy.h>
#include <curl/curlver.h>
#include <curl/urlapi.h>


#include <vector>
#include <sstream>

class hyundai : public robot
{
    CURL* curl;
    // CURLcode res;
    string base_url ;
    RConf Conf;
    double spd=100;

public:
    hyundai();

    void MoveL(WayPoints *data);
    // void MoveJ(WayPoints *data, double acc);
    void MoveJ(WayPoints *data);
    // void MoveB(WayPoints *data);
    void MoveC(WayPoints *data);
    // void Stop();
    void SetVelocity(double v);
    // void CobotInit();
    // bool WaitMove();
    bool RobotConnect(std::string ip, int port, ReciveData *Info);
    // void RobotDisconnect();
    void RobotInfo();
    // void settcp(bool on);
    // bool IsConnected();
    // void Pause();
    // void Resume();
};
size_t writeCallback(void* contents, size_t size,size_t nmemb,std::string* output);
const bool ExampleMethodPost();
std::string getJsonValue(const std::string& json, const std::string& key);
std::vector<int> getJsonArray(const std::string& json, const std::string& key);
#endif // HYUNDAI_H
