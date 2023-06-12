#include "zimmergripper.h"

ZimmerGripper::ZimmerGripper()
{
    connected = false;
    memset(reg_read, 0, sizeof(uint16_t)*NUM_RECV_REG);
    memset(reg_write, 0, sizeof(uint16_t)*NUM_SEND_REG);
    send_flag = false;
    comm_step = 0;
    grip_flag = false;
    init_flag = false;
    gripper_velocity = 50;
    gripper_force = 50;
    debug = false;
    gripper_pos = 0;
}

ZimmerGripper::~ZimmerGripper()
{
    if(connected){
        modbus_close(mb);
        modbus_free(mb);
        if (debug)
            cout << "modbus close complete" << endl;
    }
}

void ZimmerGripper::connect(std::string ip, int port)
{
//    cout << "ip : " << ip << ", port : " << port << endl;
    mb = modbus_new_tcp(ip.c_str(), port);
    int ret = modbus_connect(mb);
    if(ret == 0){
        connected = true;
        cout << "connected gripper" << endl;

        pthread_create(&comm_thread, nullptr, comm_func, this);
    }
    else{
        connected = false;
        cout << "not connected gripper" << endl;
    }
}

void ZimmerGripper::disconnect(){
    comm_thread_run = false;
    pthread_join(comm_thread, nullptr);

    modbus_close(mb);
    modbus_free(mb);
    connected = false;
    if (debug)
        cout << "modbus close complete" << endl;
}

void *ZimmerGripper::comm_func(void *arg){
    ZimmerGripper *gripper = static_cast<ZimmerGripper*>(arg);

    gripper->comm_thread_run = true;

    while(gripper->comm_thread_run && gripper->connected){
        modbus_read_registers(gripper->mb, ADDR_RECV, NUM_RECV_REG, gripper->reg_read);
//        cout << bool(gripper->reg_read[0]&0x8000) << " ";
//        cout << bool(gripper->reg_read[0]&0x4000) << " ";
//        cout << bool(gripper->reg_read[0]&0x2000) << " ";
//        cout << bool(gripper->reg_read[0]&0x1000) << "  "; // Data transfer ok

//        cout << bool(gripper->reg_read[0]&0x0800) << " ";
//        cout << bool(gripper->reg_read[0]&0x0400) << " ";
//        cout << bool(gripper->reg_read[0]&0x0200) << " ";
//        cout << bool(gripper->reg_read[0]&0x0100) << "  ";

//        cout << bool(gripper->reg_read[0]&0x0080) << " ";
//        cout << bool(gripper->reg_read[0]&0x0040) << " ";
//        cout << bool(gripper->reg_read[0]&0x0020) << " ";
//        cout << bool(gripper->reg_read[0]&0x0010) << "  ";

//        cout << bool(gripper->reg_read[0]&0x0008) << " "; // Movement complete
//        cout << bool(gripper->reg_read[0]&0x0004) << " "; // In motion
//        cout << bool(gripper->reg_read[0]&0x0002) << " ";
//        cout << bool(gripper->reg_read[0]&0x0001) << endl;

        // cout << gripper->reg_read[0] << ", " << gripper->reg_read[1] << ", " << gripper->reg_read[2] << endl;
        gripper->gripper_pos = gripper->reg_read[2];

        if(gripper->send_flag){
            switch(gripper->comm_step){
                case 0:
                {
                    if(gripper->reg_read[0]&PLCActive){
                        if (gripper->debug)
                            cout << "PLC active bit check complete" << endl;
                        modbus_write_registers(gripper->mb, ADDR_SEND, NUM_SEND_REG, gripper->reg_write);
                        gripper->comm_step++;
                    }
                    break;
                }
                case 1:
                {
                    if(gripper->reg_read[0]&DataTransferOK && gripper->reg_read[0]&MotorON){
                        if (gripper->debug)
                            cout << "Data transfer ok bit & motor on bit check complete" << endl;
                        gripper->reg_write[0] = 0;
                        modbus_write_registers(gripper->mb, ADDR_SEND, NUM_SEND_REG, gripper->reg_write);
                        gripper->comm_step++;
                    }
                    break;
                }
                case 2:
                {
                    if(!(gripper->reg_read[0]&DataTransferOK)){
                        if (gripper->debug)
                            cout << "Handshake is done" << endl;
                        gripper->reg_write[0] = 1;
                        gripper->reg_write[1] = 85*256 + 0;
                        modbus_write_registers(gripper->mb, ADDR_SEND, NUM_SEND_REG, gripper->reg_write);
                        gripper->comm_step++;
                    }
                    break;
                }
                case 3:
                {
                    if(gripper->reg_read[0]&DataTransferOK){
                        if (gripper->debug)
                            cout << "Data transfer ok bit check complete" << endl;
                        gripper->reg_write[0] = 0;
                        modbus_write_registers(gripper->mb, ADDR_SEND, NUM_SEND_REG, gripper->reg_write);
                        gripper->comm_step++;
//                        gripper->send_flag = false;
                        gripper->init_flag = true;
                    }
                    break;
                }
                case 4:
                {
                    if(!(gripper->reg_read[0]&DataTransferOK)){
                        if(gripper->grip_flag){
//                            if(!(gripper->reg_read[0]&AtWorkposition))
                            {
                                if (gripper->debug)
                                    cout << "grip move to workposition" << endl;
                                gripper->reg_write[0] = 512;
                            }
                        }
                        else{
//                            if(!(gripper->reg_read[0]&AtBaseposition))
                            {
                                if (gripper->debug)
                                    cout << "grip move to baseposition" << endl;
                                gripper->reg_write[0] = 256;
                            }
                        }

                        modbus_write_registers(gripper->mb, ADDR_SEND, NUM_SEND_REG, gripper->reg_write);
                        gripper->comm_step++;
                    }
                    break;
                }
                case 5:
                {
                    if(gripper->reg_read[0]&InMotion && !(gripper->reg_read[0]&MovementComplete)){
                        gripper->comm_step++;
                    }
                    break;
                }
                case 6:
                {
                    if(!(gripper->reg_read[0]&InMotion) && gripper->reg_read[0]&MovementComplete){
                        if (gripper->debug)
                            cout << "move complete" << endl;
                        gripper->reg_write[0] = 4;
                        modbus_write_registers(gripper->mb, ADDR_SEND, NUM_SEND_REG, gripper->reg_write);
                        gripper->comm_step++;
                        // gripper->send_flag = false;
                    }
                    break;
                }
                case 7:
                {
                    if(!(gripper->reg_read[0]&MoveWorkpositionFlag) && !(gripper->reg_read[0]&MoveBasepositionFlag)){
                        gripper->comm_step = -1;
                        gripper->send_flag = false;
                    }
                    break;
                }
                default:
                {
                    break;
                }
            }
        //    cout << endl << "step : " << gripper->comm_step << endl;
        }

        usleep(10000);
    }

    memset(gripper->reg_read, 0, sizeof(uint16_t)*NUM_RECV_REG);
    if (gripper->debug)
        cout << "gripper modbus disconnected" << endl;

    return nullptr;
}

void ZimmerGripper::get_write_reg(uint16_t reg[])
{
    memcpy(reg, reg_write, sizeof(uint16_t)*NUM_SEND_REG);
}

void ZimmerGripper::get_read_reg(uint16_t reg[])
{
    memcpy(reg, reg_read, sizeof(uint16_t)*NUM_RECV_REG);
}

void ZimmerGripper::gripper_init()
{
    reg_write[0] = 1;
    reg_write[1] = 3*256 + 0;
    reg_write[2] = 50;
    reg_write[3] = gripper_force*256 + gripper_velocity;
    reg_write[4] = 100;
    reg_write[5] = 2000;
    reg_write[7] = 4000;

    init_flag = false;
    comm_step = 0;
    send_flag = true;
    while(!init_flag){
        usleep(1000);
    }
    comm_step = -1;
}

void ZimmerGripper::gripper_grip(bool sync)
{
    if(init_flag){
        reg_write[0] = 1;
        reg_write[1] = 3*256 + 0;
        reg_write[2] = 50;
        reg_write[3] = gripper_force*256 + gripper_velocity;
        reg_write[4] = 100;
        reg_write[5] = 2000;
        reg_write[7] = 4000;

        comm_step = 0;
        send_flag = true;
        grip_flag = true;
        if(sync){
            while(send_flag){
                usleep(1000);
            }
        }
    }
}

void ZimmerGripper::gripper_release(bool sync)
{
    if(init_flag){
        reg_write[0] = 1;
        reg_write[1] = 3*256 + 0;
        reg_write[2] = 50;
        reg_write[3] = gripper_force*256 + gripper_velocity;
        reg_write[4] = 100;
        reg_write[5] = 2000;
        reg_write[7] = 4000;

        comm_step = 0;
        send_flag = true;
        grip_flag = false;
        if(sync){
            while(send_flag){
                usleep(1000);
            }
        }
    }
}

void ZimmerGripper::gripper_custom(uint16_t position, uint8_t velocity, uint8_t force, bool sync)
{
    if(init_flag){
        reg_write[0] = 1;
        reg_write[1] = 3*256 + 0;
        reg_write[2] = 50;

        reg_write[3] = force*256 + velocity;

        if(position > reg_read[2]){
            reg_write[4] = 100;
            reg_write[5] = position - 100;
            reg_write[7] = position;
            grip_flag = true;
        }
        else{
            reg_write[4] = position;
            reg_write[5] = 2000;
            reg_write[7] = 4000;
            grip_flag = false;
        }

        comm_step = 0;
        send_flag = true;

        if(sync){
            while(send_flag){
                usleep(1000);
            }
        }

        if (debug) {
            cout << "base position : " << reg_write[4] << endl;
            cout << "shift position : " << reg_write[5] << endl;;
            cout << "work position : " << reg_write[7] << endl;;
        }
    }
}

void ZimmerGripper::gripper_opt(uint8_t velocity, uint8_t force)
{
    gripper_velocity = velocity;
    gripper_force = force;
}
