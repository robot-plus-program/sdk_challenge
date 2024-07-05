#include "robot_server.h"

RobotServer::RobotServer(ros::NodeHandle nh, int server_port) : port(server_port){
    subRobotState = nh.subscribe("/keti_robot_state", 1, &RobotServer::RobotStateCallback, this);
    subGripperState = nh.subscribe("/keti_gripper_state", 1, &RobotServer::GripperStateCallback, this);

    robotState.current_joint.assign(6, 0);
    robotState.current_T_matrix.assign(16, 0);
}

RobotServer::~RobotServer(){
    delete acRobot;
}

void RobotServer::start(){
    if(port == 5000){
        acRobot = new actionlib::SimpleActionClient<keti_robot_control::RobotMoveAction>("robot_move_action", true);

        ROS_INFO("wait robot action server...");
        
        if(acRobot->waitForServer())
        {
            ROS_INFO("connected robot action server");
        }
        else{
            ROS_ERROR("robot action server error");
        }
    }
    if(port == 5002){
        acGripper = new actionlib::SimpleActionClient<keti_robot_control::GripperMoveAction>("gripper_move_action", true);

        ROS_INFO("wait gripper action server...");

        if(acGripper->waitForServer())
        {
            ROS_INFO("connected gripper action server");
        }
        else{
            ROS_ERROR("gripper action server error");
        }
    }
    memset(systemStat.idata, 0, 116*4);
    systemStat.sdata.robot_state = 1;

    if(port == 5000){
        pthread_create(&cmd_thread, NULL, cmd_func, this);
    }
    else if(port == 5001){
        pthread_create(&data_thread, NULL, data_func, this);
    }
    else if(port == 5002){
        pthread_create(&gripper_thread, NULL, gripper_func2, this);
    }
}

void *RobotServer::cmd_func(void *arg){
    RobotServer *robotServer = static_cast<RobotServer*>(arg);

    robotServer->initSocket();

    while(ros::ok()){
        robotServer->connectSocket();

        while(robotServer->connected){
            memset(robotServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
            unsigned long lenRecv = recv(robotServer->clientSockFD, robotServer->bufRecv, MAXRECEIVEBUFSIZE, 0);
            if(lenRecv > 0){
                robotServer->strRecv.resize(lenRecv);
                std::copy(robotServer->bufRecv, robotServer->bufRecv + lenRecv, begin(robotServer->strRecv));
                ROS_INFO("strRecv : %s", robotServer->strRecv.c_str());

                std::string strCmd(robotServer->strRecv.substr(0, 4));
                if(strCmd.compare("halt") == 0){
                    ROS_INFO("cmd stop!");
                    robotServer->robotCmd.cmd = 0;
                    // robotServer->acRobot->cancelGoal();
                    // robotServer->acRobot->sendGoal(robotServer->goalRobot, 
                    //     boost::bind(&RobotServer::RobotMoveResultCallback, robotServer, _1, _2),
                    //     boost::bind(&RobotServer::RobotMoveActiveCallback, robotServer),
                    //     boost::bind(&RobotServer::RobotMoveFeedbackCallback, robotServer, _1));
                    // robot_func();
                    pthread_create(&robotServer->robot_thread, NULL, robot_func, robotServer);
                }

                std::string strCmd1(robotServer->strRecv.substr(0, 6));
                if(strCmd1.compare("move_j") == 0){
                    std::istringstream iss(robotServer->strRecv.substr(11, robotServer->strRecv.find(']')));
                    std::string buffer;
                    std::vector<std::string> result;
                    while(getline(iss, buffer, ',')){
                        result.push_back(buffer);
                    }

                    for(unsigned int i = 0; i < 6; i++){
                        robotServer->robotCmd.joint[i] = atof(result[i].c_str());
                        robotServer->robotCmd.joint[i] *= DEG2RAD;
                        robotServer->robotCmd.joint[i] += offset[i];
                    }

                    ROS_INFO("cmd joint : %f, %f, %f, %f, %f, %f", robotServer->robotCmd.joint[0], robotServer->robotCmd.joint[1], robotServer->robotCmd.joint[2], robotServer->robotCmd.joint[3], robotServer->robotCmd.joint[4], robotServer->robotCmd.joint[5]);
                    robotServer->robotCmd.cmd = 1;

                    // robot_func();
                    pthread_create(&robotServer->robot_thread, NULL, robot_func, robotServer);
                }

                std::string strCmd2(robotServer->strRecv.substr(0, 6));
                if(strCmd2.compare("move_l") == 0){
                    std::istringstream iss(robotServer->strRecv.substr(11, robotServer->strRecv.find(']')));
                    std::string buffer;
                    std::vector<std::string> result;
                    while(getline(iss, buffer, ',')){
                        result.push_back(buffer);
                    }

                    for(unsigned int i = 0; i < 3; i++){
                        robotServer->robotCmd.pose[i] = atof(result[i].c_str());
                        robotServer->robotCmd.pose[i] *= 0.001;
                        robotServer->robotCmd.pose[i + 3] = atof(result[i + 3].c_str());
                        robotServer->robotCmd.pose[i + 3] *= DEG2RAD;
                    }
                    ROS_INFO("cmd pose : %f, %f, %f, %f, %f, %f", robotServer->robotCmd.pose[0], robotServer->robotCmd.pose[1], robotServer->robotCmd.pose[2], robotServer->robotCmd.pose[3], robotServer->robotCmd.pose[4], robotServer->robotCmd.pose[5]);

                    double mat[9] = {0,};
                    double ang_x = robotServer->robotCmd.pose[3], ang_y = robotServer->robotCmd.pose[4], ang_z = robotServer->robotCmd.pose[5];
                    double Rz[9] = {cos(ang_z), -sin(ang_z), 0, sin(ang_z), cos(ang_z), 0, 0, 0, 1};
                    double Ry[9] = {cos(ang_y), 0, sin(ang_y), 0, 1, 0, -sin(ang_y), 0, cos(ang_y)};
                    double Rx[9] = {1, 0, 0, 0, cos(ang_x), -sin(ang_x), 0, sin(ang_x), cos(ang_x)};
                    double Rzy[9] = {0,}, temp = 0;
                    for(unsigned int i = 0; i < 3; i++){
                        for(unsigned int j = 0; j < 3; j++){
                            temp = 0;
                            for(unsigned int k = 0; k < 3; k++){
                                temp += Rz[i*3 + k]*Ry[k*3 + j];
                            }
                            Rzy[i*3 + j] = temp;
                        }
                    }

                    for(unsigned int i = 0; i < 3; i++){
                        for(unsigned int j = 0; j < 3; j++){
                            temp = 0;
                            for(unsigned int k = 0; k < 3; k++){
                                temp += Rzy[i*3 + k]*Rx[k*3 + j];
                            }
                            mat[i*3 + j] = temp;
                        }
                    }

                    memset(robotServer->robotCmd.mat, 0, sizeof(double)*16);
                    robotServer->robotCmd.mat[3] = robotServer->robotCmd.pose[0];
                    robotServer->robotCmd.mat[7] = robotServer->robotCmd.pose[1];
                    robotServer->robotCmd.mat[11] = robotServer->robotCmd.pose[2];
                    for(unsigned int i = 0; i < 3; i++){
                        for(unsigned int j = 0; j < 3; j++){
                            robotServer->robotCmd.mat[i*4 + j] = mat[i*3 + j];
                        }
                    }
                    robotServer->robotCmd.mat[15] = 1;

                    robotServer->robotCmd.cmd = 2;
                    pthread_create(&robotServer->robot_thread, NULL, robot_func, robotServer);
                }
            
                std::string strCmd3(robotServer->strRecv.substr(0, 7));
                if(strCmd3.compare("move_pb") == 0){
                    if(lenRecv < 20){
                        memset(robotServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
                        lenRecv = recv(robotServer->clientSockFD, robotServer->bufRecv, MAXRECEIVEBUFSIZE, 0);
                        robotServer->strRecv.resize(lenRecv);
                        std::copy(robotServer->bufRecv, robotServer->bufRecv + lenRecv, begin(robotServer->strRecv));
                        // ROS_INFO("[Re]robotServer->strRecv : %s", robotServer->strRecv.c_str());
                    }

                    std::size_t indx, indx2;
                    std::vector<std::string> strPnts;
                    std::string temp;

                    indx = robotServer->strRecv.find('[');
                    indx2 = robotServer->strRecv.find(']');
                    int len = indx2 - indx - 1;
                    while(indx != std::string::npos && indx2 != std::string::npos){
                        temp = robotServer->strRecv.substr(indx + 1, len);
                        strPnts.push_back(temp);

                        indx = robotServer->strRecv.find('[', indx + len - 1);
                        indx2 = robotServer->strRecv.find(']', indx + len - 1);

                        len = indx2 - indx - 1;
                    }

                    for(unsigned int num = 0; num < strPnts.size(); num++){
                        // ROS_INFO("pnt%d : %s", num, strPnts[num].c_str());

                        std::istringstream iss(strPnts[num]);
                        std::string buffer;
                        std::vector<std::string> result;
                        while(getline(iss, buffer, ',')){
                            result.push_back(buffer);
                        }

                        for(unsigned int i = 0; i < 3; i++){
                            robotServer->robotCmd.pose[6*num + i] = atof(result[i].c_str());
                            robotServer->robotCmd.pose[6*num + i] *= 0.001;
                            robotServer->robotCmd.pose[6*num + i + 3] = atof(result[i + 3].c_str());
                            robotServer->robotCmd.pose[6*num + i + 3] *= DEG2RAD;
                        }

                        ROS_INFO("cmd pose : %f, %f, %f, %f, %f, %f", 
                            robotServer->robotCmd.pose[6*num + 0], robotServer->robotCmd.pose[6*num + 1], robotServer->robotCmd.pose[6*num + 2], 
                            robotServer->robotCmd.pose[6*num + 3], robotServer->robotCmd.pose[6*num + 4], robotServer->robotCmd.pose[6*num + 5]);

                        double mat[9] = {0,};
                        double ang_x = robotServer->robotCmd.pose[6*num + 3], ang_y = robotServer->robotCmd.pose[6*num + 4], ang_z = robotServer->robotCmd.pose[6*num + 5];
                        double Rz[9] = {cos(ang_z), -sin(ang_z), 0, sin(ang_z), cos(ang_z), 0, 0, 0, 1};
                        double Ry[9] = {cos(ang_y), 0, sin(ang_y), 0, 1, 0, -sin(ang_y), 0, cos(ang_y)};
                        double Rx[9] = {1, 0, 0, 0, cos(ang_x), -sin(ang_x), 0, sin(ang_x), cos(ang_x)};
                        double Rzy[9] = {0,}, temp = 0;
                        for(unsigned int i = 0; i < 3; i++){
                            for(unsigned int j = 0; j < 3; j++){
                                temp = 0;
                                for(unsigned int k = 0; k < 3; k++){
                                    temp += Rz[i*3 + k]*Ry[k*3 + j];
                                }
                                Rzy[i*3 + j] = temp;
                            }
                        }

                        for(unsigned int i = 0; i < 3; i++){
                            for(unsigned int j = 0; j < 3; j++){
                                temp = 0;
                                for(unsigned int k = 0; k < 3; k++){
                                    temp += Rzy[i*3 + k]*Rx[k*3 + j];
                                }
                                mat[i*3 + j] = temp;
                            }
                        }

                        double cmd_mat[16] = {0,};
                        cmd_mat[0*4 + 3] = robotServer->robotCmd.pose[6*num + 0];
                        cmd_mat[1*4 + 3] = robotServer->robotCmd.pose[6*num + 1];
                        cmd_mat[2*4 + 3] = robotServer->robotCmd.pose[6*num + 2];
                        for(unsigned int i = 0; i < 3; i++){
                            for(unsigned int j = 0; j < 3; j++){
                                cmd_mat[i*4 + j] = mat[i*3 + j];
                            }
                        }
                        cmd_mat[15] = 1;

                        for(unsigned int i = 0; i < 16; i++){
                            robotServer->robotCmd.mat[16*num + i] = cmd_mat[i];
                        }

                        // std::cout << "cmd T matrix " << num + 1 << " : " << std::endl;
                        // for(unsigned int i = 0; i < 16; i++){
                        //     std::cout << robotCmd.mat[16*num + i] << "\t";
                        // }
                        // std::cout << std::endl;
                    }
                    robotServer->robotCmd.num = strPnts.size();
                    robotServer->robotCmd.cmd = 3;

                    pthread_create(&robotServer->robot_thread, NULL, robot_func, robotServer);
                }
            }
            else{
                robotServer->connected = false;
            }
        }
    }

    return nullptr;
}

void* RobotServer::robot_func(void *arg){
    RobotServer *robotServer = static_cast<RobotServer*>(arg);
    if(robotServer->robotCmd.cmd == 0){
        robotServer->acRobot->cancelAllGoals();
    }
    else if(robotServer->robotCmd.cmd == 1){
        robotServer->goalRobot.cmd = robotServer->robotCmd.cmd;
        robotServer->goalRobot.num = 1;
        robotServer->goalRobot.value.clear();
        for(unsigned int i = 0; i < 6; i++){
            robotServer->goalRobot.value.push_back(robotServer->robotCmd.joint[i]);
        }
    }
    else if(robotServer->robotCmd.cmd == 2){
        robotServer->goalRobot.cmd = robotServer->robotCmd.cmd;
        robotServer->goalRobot.num = 1;
        robotServer->goalRobot.value.clear();
        for(unsigned int i = 0; i < 16; i++){
            robotServer->goalRobot.value.push_back(robotServer->robotCmd.mat[i]);
        }
    }
    else if(robotServer->robotCmd.cmd == 3){
        robotServer->goalRobot.cmd = robotServer->robotCmd.cmd;
        robotServer->goalRobot.num = robotServer->robotCmd.num;
        robotServer->goalRobot.value.clear();
        for(unsigned int i = 0; i < 16*robotServer->robotCmd.num; i++){
            robotServer->goalRobot.value.push_back(robotServer->robotCmd.mat[i]);
        }
        // for(unsigned int num = 0; num < robotServer->robotCmd.num; num++){
        //     std::cout << "cmd T matrix " << num + 1 << " : " << std::endl;
        //     for (unsigned int i = 0; i < 16; i++)
        //     {
        //         std::cout << robotServer->goalRobot.value[16 * num + i] << "\t";
        //     }
        //     std::cout << std::endl;
        // }
    }
    robotServer->acRobot->sendGoal(robotServer->goalRobot, 
        boost::bind(&RobotServer::RobotMoveResultCallback, robotServer, _1, _2),
        boost::bind(&RobotServer::RobotMoveActiveCallback, robotServer),
        boost::bind(&RobotServer::RobotMoveFeedbackCallback, robotServer, _1));
    // robotServer->acRobot->waitForResult();
    robotServer->robotCmd.cmd = -1;

    return nullptr;
}

void *RobotServer::data_func(void *arg){
    RobotServer *robotServer = static_cast<RobotServer*>(arg);

    robotServer->initSocket();

    while(ros::ok()){
        robotServer->connectSocket();

        while(robotServer->connected){
            memset(robotServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
            unsigned long lenRecv = recv(robotServer->clientSockFD, robotServer->bufRecv, MAXRECEIVEBUFSIZE, 0);
            if(lenRecv > 0){
                robotServer->strRecv.resize(lenRecv);
                std::copy(robotServer->bufRecv, robotServer->bufRecv + lenRecv, begin(robotServer->strRecv));
//                ROS_INFO("%ld", lenRecv);
//                ROS_INFO("%s", robotServer->strRecv.c_str());
//                for(unsigned int i = 0; i < lenRecv; i++){
//                  ROS_INFO("%d", robotServer->strRecv.at(i));
//                }
                if(robotServer->strRecv.compare("reqdata") == 0){
                    robotServer->systemStat.sdata.header[0] = 0x24;
                    robotServer->systemStat.sdata.header[1] = 116;
                    robotServer->systemStat.sdata.header[2] = 116>>8;
                    robotServer->systemStat.sdata.header[3] = 0x03;

                    robotServer->systemStat.sdata.robot_state = robotServer->robotState.state;
                    robotServer->systemStat.sdata.program_mode = 0;
                    robotServer->systemStat.sdata.init_state_info = 6;

                    for(unsigned int i = 0; i < 6; i++){
                        robotServer->systemStat.sdata.jnt_ang[i] = static_cast<float>(robotServer->robotState.current_joint[i]*RAD2DEG);
                    }
                    // ROS_INFO("current joint1 : %f, %f, %f, %f, %f, %f", robotServer->robotState.current_joint[0], robotServer->robotState.current_joint[1], robotServer->robotState.current_joint[2], robotServer->robotState.current_joint[3], robotServer->robotState.current_joint[4], robotServer->robotState.current_joint[5]);
                    // ROS_INFO("current joint2 : %f, %f, %f, %f, %f, %f", robotServer->systemStat.sdata.jnt_ang[0], robotServer->systemStat.sdata.jnt_ang[1], robotServer->systemStat.sdata.jnt_ang[2], robotServer->systemStat.sdata.jnt_ang[3], robotServer->systemStat.sdata.jnt_ang[4], robotServer->systemStat.sdata.jnt_ang[5]);

                    robotServer->systemStat.sdata.tcp_pos[0] = static_cast<float>(robotServer->robotState.current_T_matrix[3])*1000;
                    robotServer->systemStat.sdata.tcp_pos[1] = static_cast<float>(robotServer->robotState.current_T_matrix[7])*1000;
                    robotServer->systemStat.sdata.tcp_pos[2] = static_cast<float>(robotServer->robotState.current_T_matrix[11])*1000;

                    robotServer->systemStat.sdata.tcp_pos[3] = static_cast<float>(atan2(robotServer->robotState.current_T_matrix[2 * 4 + 1], robotServer->robotState.current_T_matrix[2 * 4 + 2])*RAD2DEG);
                    robotServer->systemStat.sdata.tcp_pos[4] = static_cast<float>(asin(-robotServer->robotState.current_T_matrix[2 * 4 + 0])*RAD2DEG);
                    robotServer->systemStat.sdata.tcp_pos[5] = static_cast<float>(atan2(robotServer->robotState.current_T_matrix[1 * 4 + 0], robotServer->robotState.current_T_matrix[0 * 4 + 0])*RAD2DEG);

                    unsigned long lenSend = send(robotServer->clientSockFD, robotServer->systemStat.idata, 116*4, 0);
                }
            }
            else{
                robotServer->connected = false;
            }
        }
    }

    return nullptr;
}

void *RobotServer::gripper_func2(void *arg){
  RobotServer *robotServer = static_cast<RobotServer*>(arg);

  uint16_t StatusWord, Diagnosis, Position;
  uint16_t ControlWord, BasePosition, ShiftPosition, TeachPosition, WorkPosition;
  uint8_t DeviceMode, GripForce, DriveVelocity;

  robotServer->initSocket();

  while(ros::ok()){
    robotServer->connectSocket();

    StatusWord = Error|PLCActive|MovementComplete|MotorON|HomingPositionOK|AtBaseposition;
    Diagnosis = 774;
    Position = 100;
    ControlWord = 4;

    while(robotServer->connected){
      memset(robotServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
      unsigned long lenRecv = recv(robotServer->clientSockFD, robotServer->bufRecv, MAXRECEIVEBUFSIZE, 0);
      if(lenRecv > 0){
        robotServer->strRecv.resize(lenRecv);
        std::copy(robotServer->bufRecv, robotServer->bufRecv + lenRecv, begin(robotServer->strRecv));
//        ROS_INFO("lenRecv : %ld", lenRecv);
//        for(unsigned int i = 0; i < lenRecv; i++){
//          ROS_INFO("%d", robotServer->strRecv.at(i));
//        }

        Position = robotServer->gripperState.width*4000/(0.04);
//        ROS_INFO("gripper width : %d", Position);

        if(robotServer->gripperState.state == 1){
            StatusWord &= ~InMotion;
            StatusWord |= MovementComplete;
            if(Position < 300){
                StatusWord |= AtBaseposition;
                StatusWord &= ~AtWorkposition;
            }
            else{
                StatusWord |= AtWorkposition;
                StatusWord &= ~AtBaseposition;
            }
        }
        else if(robotServer->gripperState.state == 2){
            StatusWord |= InMotion;
            StatusWord &= ~MovementComplete;
            StatusWord &= ~AtBaseposition;
            StatusWord &= ~AtWorkposition;
        }

//        ROS_INFO("StatusWord : %d", StatusWord);

        if(robotServer->bufRecv[7] == 0x03){
          robotServer->bufRecv[5] = 0x09;
          robotServer->bufRecv[8] = 0x06;

          robotServer->bufRecv[9] = StatusWord >> 8;
          robotServer->bufRecv[10] = (uint8_t)StatusWord;

          robotServer->bufRecv[11] = Diagnosis >> 8;
          robotServer->bufRecv[12] = (uint8_t)Diagnosis;

          robotServer->bufRecv[13] = Position >> 8;
          robotServer->bufRecv[14] = (uint8_t)Position;

          unsigned long lenSend = send(robotServer->clientSockFD, robotServer->bufRecv, 15, 0);
        }
        else if(robotServer->bufRecv[7] == 0x10){
            int indx = 7 + 6;

            ControlWord = robotServer->bufRecv[indx++]*256;
            ControlWord += robotServer->bufRecv[indx++];

            DeviceMode = robotServer->bufRecv[indx++];
            indx += 3;

            GripForce = robotServer->bufRecv[indx++];
            DriveVelocity = robotServer->bufRecv[indx++];

            BasePosition = robotServer->bufRecv[indx++]*256;
            BasePosition += robotServer->bufRecv[indx++];

            ShiftPosition = robotServer->bufRecv[indx++]*256;
            ShiftPosition += robotServer->bufRecv[indx++];

            TeachPosition = robotServer->bufRecv[indx++]*256;
            TeachPosition += robotServer->bufRecv[indx++];

            WorkPosition = robotServer->bufRecv[indx++]*256;
            WorkPosition += robotServer->bufRecv[indx++];

//             std::cout << "ControlWord : " << ControlWord << std::endl;
//             std::cout << "DeviceMode : " << (int)DeviceMode << std::endl;
//             std::cout << "GripForce : " << (int)GripForce << std::endl;
//             std::cout << "DriveVelocity : " << (int)DriveVelocity << std::endl;
//             std::cout << "BasePosition : " << BasePosition << std::endl;
//             std::cout << "ShiftPosition : " << ShiftPosition << std::endl;
//             std::cout << "TeachPosition : " << TeachPosition << std::endl;
//             std::cout << "WorkPosition : " << WorkPosition << std::endl;

            if(ControlWord == 1){
                StatusWord |= DataTransferOK;
                StatusWord &= ~Error;
                Diagnosis = 0;
            }
            else if(ControlWord == 0){
                StatusWord &= ~DataTransferOK;
            }
            else if(ControlWord == 512){
                robotServer->goalGripper.cmd = 1;
                robotServer->acGripper->sendGoal(robotServer->goalGripper,
                    boost::bind(&RobotServer::GripperMoveResultCallback, robotServer, _1, _2),
                    boost::bind(&RobotServer::GripperMoveActiveCallback, robotServer),
                    boost::bind(&RobotServer::GripperMoveFeedbackCallback, robotServer, _1));
                // robotServer->acGripper->waitForResult();
            }
            else if(ControlWord == 256){
                robotServer->goalGripper.cmd = 2;
                robotServer->acGripper->sendGoal(robotServer->goalGripper,
                    boost::bind(&RobotServer::GripperMoveResultCallback, robotServer, _1, _2),
                    boost::bind(&RobotServer::GripperMoveActiveCallback, robotServer),
                    boost::bind(&RobotServer::GripperMoveFeedbackCallback, robotServer, _1));
                // robotServer->acGripper->waitForResult();
            }
            else if(ControlWord == 4){
                StatusWord &= ~MoveWorkpositionFlag;
                StatusWord &= ~MoveBasepositionFlag;
            }
        }
        usleep(1000);

      }
      else{
        robotServer->connected = false;
      }
    }
  }

  return nullptr;
}

void *RobotServer::gripper_func(void *arg){
    RobotServer *robotServer = static_cast<RobotServer*>(arg);

    int s = -1;
    modbus_t *ctx;
    int rc;
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    int header_length;
    std::string ip_or_device;

    ip_or_device = "0.0.0.0";

    ctx = modbus_new_tcp(ip_or_device.c_str(), robotServer->port);

    header_length = modbus_get_header_length(ctx);

    std::cout << "header_length : " << header_length << std::endl;

    uint16_t StatusWord, Diagnosis, Position;
    uint16_t ControlWord, BasePosition, ShiftPosition, TeachPosition, WorkPosition;
    uint8_t DeviceMode, GripForce, DriveVelocity;

    while(ros::ok()){

        ROS_INFO("modbus server running waiting. waiting client...");
        s = modbus_tcp_listen(ctx, 1);
        modbus_tcp_accept(ctx, &s);
        StatusWord = Error|PLCActive|MovementComplete|MotorON|HomingPositionOK|AtBaseposition;
        Diagnosis = 774;
        Position = 100;
        ControlWord = 4;

        while(true){
            rc = modbus_receive(ctx, query);
            // ROS_INFO("rc : %d", rc);

            if (rc == -1) {
                break;
            }

            Position = robotServer->gripperState.width*4000/(0.04);

            if(robotServer->gripperState.state == 1){
                StatusWord &= ~InMotion;
                StatusWord |= MovementComplete;
                if(Position < 300){
                    StatusWord |= AtBaseposition;
                    StatusWord &= ~AtWorkposition;
                }
                else{
                    StatusWord |= AtWorkposition;
                    StatusWord &= ~AtBaseposition;
                }
            }
            else if(robotServer->gripperState.state == 2){
                StatusWord |= InMotion;
                StatusWord &= ~MovementComplete;
                StatusWord &= ~AtBaseposition;
                StatusWord &= ~AtWorkposition;
            }

            if(query[header_length] == 0x03){
                query[5] = 0x09;
                query[8] = 0x06;

                query[9] = StatusWord >> 8;
                query[10] = (uint8_t)StatusWord;

                query[11] = Diagnosis >> 8;
                query[12] = (uint8_t)Diagnosis;

                query[13] = Position >> 8;
                query[14] = (uint8_t)Position;

                int w_s = modbus_get_socket(ctx);
                rc = send(w_s, (const char*) query, 15, MSG_NOSIGNAL);

                if (rc == -1) {
                    break;
                }
            }
            else if(query[header_length] == 0x10){
                int indx = header_length + 6;

                ControlWord = query[indx++]*256;
                ControlWord += query[indx++];

                DeviceMode = query[indx++];
                indx += 3;

                GripForce = query[indx++];
                DriveVelocity = query[indx++];

                BasePosition = query[indx++]*256;
                BasePosition += query[indx++];

                ShiftPosition = query[indx++]*256;
                ShiftPosition += query[indx++];

                TeachPosition = query[indx++]*256;
                TeachPosition += query[indx++];

                WorkPosition = query[indx++]*256;
                WorkPosition += query[indx++];

                // std::cout << "ControlWord : " << ControlWord << std::endl;
                // std::cout << "DeviceMode : " << (int)DeviceMode << std::endl;
                // std::cout << "GripForce : " << (int)GripForce << std::endl;
                // std::cout << "DriveVelocity : " << (int)DriveVelocity << std::endl;
                // std::cout << "BasePosition : " << BasePosition << std::endl;
                // std::cout << "ShiftPosition : " << ShiftPosition << std::endl;
                // std::cout << "TeachPosition : " << TeachPosition << std::endl;
                // std::cout << "WorkPosition : " << WorkPosition << std::endl;

                if(ControlWord == 1){
                    StatusWord |= DataTransferOK;
                    StatusWord &= ~Error;
                    Diagnosis = 0;
                }
                else if(ControlWord == 0){
                    StatusWord &= ~DataTransferOK;
                }
                else if(ControlWord == 512){
                    robotServer->goalGripper.cmd = 1;
                    robotServer->acGripper->sendGoal(robotServer->goalGripper, 
                        boost::bind(&RobotServer::GripperMoveResultCallback, robotServer, _1, _2),
                        boost::bind(&RobotServer::GripperMoveActiveCallback, robotServer),
                        boost::bind(&RobotServer::GripperMoveFeedbackCallback, robotServer, _1));
                    // robotServer->acGripper->waitForResult();
                }
                else if(ControlWord == 256){
                    robotServer->goalGripper.cmd = 2;
                    robotServer->acGripper->sendGoal(robotServer->goalGripper, 
                        boost::bind(&RobotServer::GripperMoveResultCallback, robotServer, _1, _2),
                        boost::bind(&RobotServer::GripperMoveActiveCallback, robotServer),
                        boost::bind(&RobotServer::GripperMoveFeedbackCallback, robotServer, _1));
                    // robotServer->acGripper->waitForResult();
                }
                else if(ControlWord == 4){
                    StatusWord &= ~MoveWorkpositionFlag;
                    StatusWord &= ~MoveBasepositionFlag;
                }
            }

            usleep(10000);
        }
        ROS_INFO("modbus disconnected");

        modbus_close(ctx);
        modbus_free(ctx);
    }

    return nullptr;
}

void RobotServer::RobotMoveResultCallback(const actionlib::SimpleClientGoalState &state, const keti_robot_control::RobotMoveResultConstPtr &result){
    // ROS_INFO("Robot Move Complete");
}

void RobotServer::RobotMoveFeedbackCallback(const keti_robot_control::RobotMoveFeedbackConstPtr &feedback){
    // ROS_INFO("Robot Move Feedback");
}

void RobotServer::RobotMoveActiveCallback(){
    // ROS_INFO("Robot Goal just went active");
    // systemStat.sdata.robot_state = 1;
}

void RobotServer::GripperMoveResultCallback(const actionlib::SimpleClientGoalState &state, const keti_robot_control::GripperMoveResultConstPtr &result)
{
    // ROS_INFO("Robot Move Complete");
}

void RobotServer::GripperMoveFeedbackCallback(const keti_robot_control::GripperMoveFeedbackConstPtr &feedback)
{
    // ROS_INFO("Robot Move Feedback");
}

void RobotServer::GripperMoveActiveCallback()
{
    // ROS_INFO("Robot Goal just went active");
}

void RobotServer::RobotStateCallback(const keti_robot_control::RobotState &msg){
    // ROS_INFO("robot_state : %d", msg.state);
    // ROS_INFO("current joint(before) : %f, %f, %f, %f, %f, %f", msg.current_joint[0], msg.current_joint[1], msg.current_joint[2], msg.current_joint[3], msg.current_joint[4], msg.current_joint[5]);
    // ROS_INFO("current T matrix : ");
    // ROS_INFO("%f, %f, %f, %f", msg.current_T_matrix[0], msg.current_T_matrix[1], msg.current_T_matrix[2], msg.current_T_matrix[3]);
    // ROS_INFO("%f, %f, %f, %f", msg.current_T_matrix[4], msg.current_T_matrix[5], msg.current_T_matrix[6], msg.current_T_matrix[7]);
    // ROS_INFO("%f, %f, %f, %f", msg.current_T_matrix[8], msg.current_T_matrix[9], msg.current_T_matrix[10], msg.current_T_matrix[11]);
    // ROS_INFO("%f, %f, %f, %f", msg.current_T_matrix[12], msg.current_T_matrix[13], msg.current_T_matrix[14], msg.current_T_matrix[15]);

    robotState.state = msg.state;
    for(unsigned int i = 0; i < 6; i++){
        robotState.current_joint[i] = msg.current_joint[i] - offset[i];
    }
    // memcpy(robotState.current_joint.data(), msg.current_joint.data(), sizeof(double)*6);
    memcpy(robotState.current_T_matrix.data(), msg.current_T_matrix.data(), sizeof(double)*16);

    // ROS_INFO("current joint(after) : %f, %f, %f, %f, %f, %f", robotState.current_joint[0], robotState.current_joint[1], robotState.current_joint[2], robotState.current_joint[3], robotState.current_joint[4], robotState.current_joint[5]);
}

void RobotServer::GripperStateCallback(const keti_robot_control::GripperState &msg){
    // ROS_INFO("gripper_state : %d", msg.state);
    // ROS_INFO("width : %f", msg.width);
    gripperState.state = msg.state;
    gripperState.width = msg.width;
}

void RobotServer::RobotMoveStateCallback(const moveit_msgs::ExecuteTrajectoryActionFeedback &feedback){
    ROS_INFO("%d, %s", feedback.status.status, feedback.status.text.c_str());
    if(feedback.status.status == 1){
        // systemStat.sdata.robot_state = 2;
    }
    else if(feedback.status.status == 3){
        // systemStat.sdata.robot_state = 1;
    }
}

void RobotServer::initSocket(){
    listenSockFD = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(listenSockFD < 0){
        // cout << endl << "socket create error" << endl;
        ROS_ERROR("socket create error");
        ros::shutdown();
        return;
    }

    int on = 1;
    if(setsockopt(listenSockFD, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
        // cout << endl << "set option curLen = 0; error!!" << endl;
        ROS_ERROR("set optio cuLen = 0; error!!");
        return;
    }
    if(setsockopt(listenSockFD, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
        // cout << endl << "set option curLen = 0; error!!" << endl;
        ROS_ERROR("set option curLen = 0; error!!");
        return;
    }

    // server_addr.sin_addr.s_addr = inet_addr("192.168.135.201");
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    ROS_INFO("server binded");
    ROS_INFO("address : %s", inet_ntoa(server_addr.sin_addr));
    ROS_INFO("port : %d", ntohs(server_addr.sin_port));

    if(bind(listenSockFD, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) < 0){
        ROS_ERROR("bind error");
        ros::shutdown();
        // cout << endl << "bind error" << endl;
        return;
    }
}

void RobotServer::connectSocket(){
    // cout << "server running waiting. waiting client..." << endl;
    ROS_INFO("server running waiting. waiting client...");

    if(listen(listenSockFD, MAXCONNECTIONS) < 0){
        // cout << endl << "listen error" << endl;
        ROS_ERROR("listen error");
        ros::shutdown();
    }
    int clientAddrSize = sizeof(client_addr);

    memset(bufWait, 0, MAXWAITBUFSIZE);
    ptrRecvBufIndx = bufWait;
    clientSockFD = accept(listenSockFD, reinterpret_cast<struct sockaddr*>(&client_addr), reinterpret_cast<socklen_t*>(&clientAddrSize));

    if(clientSockFD < 0){
        // cout << endl << "accept error" << endl;
        ROS_ERROR("accept error");
        ros::shutdown();
    }

    // cout << "(" << port << ") " << "client accepted" << endl;
    // cout << "(" << port << ") " << "address : " << inet_ntoa(client_addr.sin_addr) << endl;
    // cout << "(" << port << ") " << "port : " << ntohs(client_addr.sin_port) << endl;
    ROS_INFO("(%d)client accepted", port);
    ROS_INFO("(%d)address : %s", port, inet_ntoa(client_addr.sin_addr));
    ROS_INFO("(%d)port : %d", port, ntohs(client_addr.sin_port));

    connected = true;
}
