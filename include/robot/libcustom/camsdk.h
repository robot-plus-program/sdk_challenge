#ifndef WITHCAMSDK_H
#define WITHCAMSDK_H

#include "../libcustom/tcpclient.h"

#include <stdio.h>
#pragma pack(push,1)
struct sInfo
{
   int State;
   double PoseData[16];

};

union Info
{
    char buf[16*8+4];
    sInfo Data;
};

#pragma pack(pop)
class CamSDK
{

public:
    Info SendData, ReadData;
    CamSDK();
    ~CamSDK();
    void Connect(const string& _ip, uint16_t _port,int _timeout=-1);
    void Disconect();

    void SendMsg(char Msg[]);
    void SendMsg(char *Msg,int len);
    char *ReadMsg();
private:
    TCPClient *CAMSDKSocket;
};

#endif // WITHCAMSDK_H
