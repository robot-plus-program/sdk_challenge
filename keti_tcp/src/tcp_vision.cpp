#include <ros/ros.h>
#include <keti_msgs/VisionState.h>
#include <keti_msgs/VisionComm.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sstream>
#include <stdlib.h>

#include <pthread.h>

const int MAXCONNECTIONS = 5;
const int MAXWAITBUFSIZE = 4096;
const int MAXSENDBUFSIZE = 10;
const int MAXRECEIVEBUFSIZE = 4096;
const int RECVBUFSIZE = 1024;
const int SENDBUFSIZE = 1;

static bool connected = false;
static std::vector<double> value;
static std::string server_ip;
static int server_port;

static long sendByteLen;
static long recvByteLen;
static long byteLen;
static unsigned int curLen;
static int serverSockFD;
static char *ptrRecvBufIndx = nullptr;
static char buf[RECVBUFSIZE] = {0,};
static char bufWait[MAXWAITBUFSIZE] = {0,};
static char bufSend[MAXSENDBUFSIZE] = {0,};
static sockaddr_in server_addr;

using namespace std;

static void* vision_comm_func(void *arg){

    ros::Rate loop_rate(1);

    int dataLen = 0;
    
    connected = false;

    while(ros::ok()){
        if(!connected){
            serverSockFD = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if(serverSockFD < 0){
                cout << endl << "socket create error" << endl;
            }

            int on = 1;
            if(setsockopt(serverSockFD, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
                cout << endl << "set option curLen = 0; error!!" << endl;
            }
            if(setsockopt(serverSockFD, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
                cout << endl << "set option curLen = 0; error!!" << endl;
            }

            memset(&server_addr, 0x00, sizeof(sockaddr_in));

            // std::cout << "ip : " << server_ip << std::endl;
            // std::cout << "port : " << server_port << std::endl;

            server_addr.sin_addr.s_addr = inet_addr(server_ip.c_str());
            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(server_port);

            curLen = 0;
            memset(bufWait, 0, MAXWAITBUFSIZE);
            ptrRecvBufIndx = bufWait;
            int ret = connect(serverSockFD, (struct sockaddr*)&server_addr, sizeof(server_addr));

            if(ret < 0){
                ROS_INFO("connect error");
                connected = false;
            }
            else{
                ROS_INFO("connect success");
                connected = true;
            }
        }

        // while(connected){
        //     byteLen = recv(serverSockFD, buf, RECVBUFSIZE, 0);
        //     if(byteLen > 0){
        //         if(byteLen == RECVBUFSIZE){
        //             for(int i = 0; i < byteLen; i++){
        //                 printf("%d\t", buf[i]);
        //             }
        //             printf("\n");
        //         }
        //     }
        //     else{
        //         connected = false;
        //         ROS_INFO("Disconnected vision comm");
        //         break;
        //     }
        // }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return nullptr;
}

bool vision_comm(keti_msgs::VisionComm::Request &req, keti_msgs::VisionComm::Response &res){
    ROS_INFO("Receive request");
    if(connected){
        bufSend[0] = '1';
        sendByteLen = send(serverSockFD, bufSend, 1, 0);
        if(sendByteLen >= 0){
            byteLen = recv(serverSockFD, buf, RECVBUFSIZE, 0);
            if(byteLen > 0){
                printf("\n");
                for(uint8_t i = 0; i < byteLen; i++){
                    printf("%d ", buf[i]);
                }
                printf("\n");

                res.value.clear();
                double data = 0;
                int num = byteLen/8;

                for(int i = 0; i < num; i++){
                    memcpy(&data, buf + 8*i, 8);
                    res.value.push_back(data);
                }
                ROS_INFO("vision data : %f, %f, %f", res.value[0], res.value[1], res.value[2]);
            }
            else{
                connected = false;
                res.value.clear();
                ROS_INFO("Disconnected vision comm");
            }
        }
        else{
            connected = false;
            res.value.clear();
            ROS_INFO("Disconnected vision comm");
        }
    }
    res.connected = connected;

    return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "tcp_vision_node");

	ros::NodeHandle nh;
    ros::Publisher pubTcpVision = nh.advertise<keti_msgs::VisionState>("keti_vision_state",1);
    ros::ServiceServer srvVisionRequest  = nh.advertiseService("keti_vision_comm", vision_comm);

    std::string ip;
    int port = 0;
    nh.getParam("ip", ip);
    nh.getParam("port", port);
    ROS_INFO("server ip : %s, server port : %d", ip.c_str(), port);

    server_ip = ip;
    server_port = port;

    ros::Rate loop_rate(10);

    keti_msgs::VisionState msg;
    msg.connected = 0;

    pthread_t vision_comm_thread;
    pthread_create(&vision_comm_thread, NULL, vision_comm_func, NULL);

    value.assign(0, 6);

    while(ros::ok()){
        msg.connected = connected;

        pubTcpVision.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
} 

