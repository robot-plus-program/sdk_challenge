#ifndef ZIMMERGRIPPER_H
#define ZIMMERGRIPPER_H

#include <modbus/modbus.h>
#include <pthread.h>
#include <unistd.h>
#include <memory.h>
#include <iostream>

// Output data word 0 - 0x0801 (ControlWord)
// Output data word 1 - 0x0802 (DeviceMode, Workpiece No)
// Output data word 2 - 0x0803 (Reserve, PositionTolerance)
// Output data word 3 - 0x0804 (GripForce, DriveVelocity)
// Output data word 4 - 0x0805 (BasePosition)
// Output data word 5 - 0x0806 (ShiftPosition)
// Output data word 6 - 0x0807 (TeachPosition)
// Output data word 7 - 0x0808 (WorkPosition)

// Input data word 0 - 0x0002 (StatusWord)
// Input data word 1 - 0x0003 (Diagnosis)
// Input data word 2 - 0x0004 (ActualPosition)

const uint8_t NUM_RECV_REG = 3;
const uint8_t NUM_SEND_REG = 8;
const uint32_t ADDR_RECV = 0x0001;//0x0002;
const uint32_t ADDR_SEND = 0x0801;//0x0801;

const uint16_t Error = 0x8000;
const uint16_t MoveWorkpositionFlag = 0x4000;
const uint16_t MoveBasepositionFlag = 0x2000;
const uint16_t DataTransferOK = 0x1000;
const uint16_t AtUndefinedPosition = 0x0800;
const uint16_t AtWorkposition = 0x0400;
const uint16_t AtBaseposition = 0x0100;
const uint16_t PLCActive = 0x0040;
const uint16_t MovementComplete = 0x0008;
const uint16_t InMotion = 0x0004;
const uint16_t MotorON = 0x0002;
const uint16_t HomingPositionOK = 0x0001;

using namespace std;

class ZimmerGripper
{
public:
    ZimmerGripper();
    ~ZimmerGripper();

    void connect(std::string ip, int port);
    void disconnect();
    bool isConnected(){return connected;}
    static void *comm_func(void *arg);
    void get_write_reg(uint16_t reg[NUM_SEND_REG]);
    void get_read_reg(uint16_t reg[NUM_RECV_REG]);
    void gripper_init();
    void gripper_grip(bool sync = true);
    void gripper_release(bool sync = true);
    void gripper_custom(uint16_t position, uint8_t velocity, uint8_t force, bool sync = true);
	void gripper_opt(uint8_t velocity, uint8_t force);
	void gripper_jog_enable();
	void gripper_jog_plus();
	void gripper_jog_minus();
	void gripper_homing();
	void gripper_stop();
	void set_inner();
	void set_outer();
    double gripper_cur_pos(){return grip_distance;};

private:
    modbus_t *mb;
    bool connected;
    pthread_t comm_thread;
    bool comm_thread_run;
    uint16_t reg_read[NUM_RECV_REG], reg_write[NUM_SEND_REG];
    bool send_flag;
    int comm_step;
    bool grip_flag, init_flag;
    uint8_t gripper_velocity, gripper_force;
	int mode_indx = 0;
	int mode[2] = {85, 95};
    double grip_distance;
    bool debug;
    int m_port;
};

// extern "C"{
//     ZimmerGripper* SetGripper();
//     void connect(ZimmerGripper* pthis, const char * ip, int port);
//     void disconnect(ZimmerGripper* pthis);
//     void gripper_init(ZimmerGripper* pthis);
//     void gripper_grip(ZimmerGripper* pthis);
//     void gripper_release(ZimmerGripper* pthis);
//     double gripper_cur_pos(ZimmerGripper* pthis);
//     // void gripper_set_inner(ZimmerGripper* pthis);
//     // void gripper_set_outer(ZimmerGripper* pthis);
// }

#endif // ZIMMERGRIPPER_H
