#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#define _XOPEN_SOURCE_EXTENDED 1

#include <iostream>
#include <chrono>
#include <thread>

#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <arpa/inet.h>


#include <netinet/tcp.h>
#include <arpa/inet.h>


const int MAXCONNECTIONS = 5;
const int MAXWAITBUFSIZE = 4096;
const int MAXSENDBUFSIZE = 1024;
const int MAXRECEIVEBUFSIZE = 4096;
//const int MAXRECEIVEBUFSIZE = 8*480000;
const int RECVBUFSIZE = 509;

using namespace std;

enum TCPCmd{
    Idle,
    Connection,
    Disconnction,
    Write,
    Read
};

class TCPClient
{


public:
    TCPClient();
    ~TCPClient();

//    void Connect(const string& _ip, uint16_t _port,int _timeout=-1);
    int Connect(const string& _ip, uint16_t _port,int _timeout=-1);
    void Disconnect();
    void Write(char* _buf,int _len = -1);

//    void Read();
    char* Read();

    void state();
    bool IsConnected();

    bool comm_thread_run;
//    char bufSend[MAXSENDBUFSIZE] = {0,};
    char *bufSend;
    int len;
    std::chrono::duration<double,std::milli> dt,timeout;

     TCPCmd Cmd;
     bool writeinfo=false;
     long recvByteLen;
private:
    long sendByteLen;
    long byteLen;
    unsigned int curLen;
    int SockFD;
    char *ptrRecvBufIndx = nullptr;
    char buf[MAXRECEIVEBUFSIZE] = {0,};
    char bufWait[MAXWAITBUFSIZE] = {0,};

    int dataLen = 0;

    int ret=-1;
    pthread_t comm_rx_thread;

    sockaddr_in server_addr;

    uint16_t port;
    string ip;

//    TCPCmd Cmd;

    bool server_connected=false;

    void Connect();
protected:

    void Write();
    static void* run(void* arg);
};

#endif // TCPCLIENT_H
