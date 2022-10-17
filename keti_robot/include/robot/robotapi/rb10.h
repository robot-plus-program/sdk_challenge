#ifndef rb10_H
#define rb10_H

#include "qstring.h"
#include <QtNetwork>
#include "../robotconf.h"



#include <iostream>

#define MAX_MC              6
#define RT_TIMER_PERIOD_MS  2
#define COMMAND_CANID           0x01


typedef union{
    struct{
        unsigned    FET:1;	 	// FET ON   //
        unsigned    RUN:1;		// Control ON
        unsigned    INIT:1;     // Init Process Passed  //
        unsigned    MOD:1;		// Control Mode
        unsigned    FRC:1;		// Friction Compensation //
        unsigned    BAT:1;      // Low Battery //
        unsigned    CALIB:1;    // Calibration Mode //
        unsigned    MT_ERR:1;   // Multi-Turn Error //

        unsigned    JAM:1;		// JAM Error
        unsigned    CUR:1;		// Over Current Error
        unsigned    BIG:1;		// Big Position Error
        unsigned    INP:1;      // Big Input Error
        unsigned    FLT:1;		// FET Driver Fault Error //
        unsigned    TMP:1;      // Temperature Error //
        unsigned    PS1:1;		// Position Limit Error (Lower) ////
        unsigned    PS2:1;		// Position Limit Error (Upper)

        unsigned    rsvd:8;

        unsigned    CAN:1;
        unsigned    rsvd2:7;
    }b;
    unsigned char B[4];
}mSTAT;


enum RCR_INIT_ERR_FLAG{
    INIT_STAT_ERR_CLEAR = 0,
    INIT_STAT_ERR_NO_SMPS,
    INIT_STAT_ERR_NO_EMG_SW,
    INIT_STAT_ERR_GPIF_PSW,
    INIT_STAT_ERR_GPIF_FET,
    INIT_STAT_ERR_MC_CAN_CHECK,
    INIT_STAT_ERR_MC_FIND_HOME,
    INIT_STAT_ERR_COL_OFFSET,
};

enum RCR_INIT_INFO{
    INIT_STAT_INFO_NOACT = 0,
    INIT_STAT_INFO_VOLTAGE_CHECK,
    INIT_STAT_INFO_DEVICE_CHECK,
    INIT_STAT_INFO_FIND_HOME,
    INIT_STAT_INFO_VARIABLE_CHECK,
    INIT_STAT_INFO_COLLISION_ON,
    INIT_STAT_INFO_INIT_DONE
};


#define MAX_SHARED_DATA 116
#define MAX_CONFIG_DATA 24

typedef struct{
    char    shared_data_name[64];
    int     shared_data_type;
}shared_data_info;
//extern shared_data_info sd_info_stat[MAX_SHARED_DATA];
//extern shared_data_info sd_info_config[MAX_CONFIG_DATA];

typedef union{
    struct{
        char    header[4];
        // 0
        float   time;                   // time [sec]
        float   jnt_ref[6];             // joint reference [deg]
        float   jnt_ang[6];             // joint encoder value [deg]
        float   cur[6];                 // joint current value [mA]
        // 19
        float   tcp_ref[6];             // calculated tool center point from reference [mm, deg]
        float   tcp_pos[6];             // calculated tool center point from encoder [mm, deg]
        // 31
        float   analog_in[4];           // analog input value of control box [V]
        float   analog_out[4];          // analog output value of control box [V]
        int     digital_in[16];         // digital input value of control box [0 or 1]
        int     digital_out[16];        // digital input value of control box [0 or 1]
        // 71
        float   temperature_mc[6];      // board temperature of each joint [celcius]
        // 77
        int     task_pc;                // (ignore)
        int     task_repeat;            // (ignore)
        int     task_run_id;            // (ignore)
        int     task_run_num;           // (ignore)
        float   task_run_time;          // (ignore)
        int     task_state;             // (ignore)
        // 83
        float   default_speed;          // overriding speed [0~1]
        int     robot_state;            // state of robot motion [1:idle  2:paused or stopped by accident  3: moving]
        int     power_state;            // power state
        // 86
        float   tcp_target[6];          // (ignore)
        int     jnt_info[6];            // joint information (look mSTAT)
        // 98
        int     collision_detect_onoff; // collision detect onoff [0:off  1:on]
        int     is_freedrive_mode;      // current freedrive status [0:off  1:on]
        int     program_mode;           // current program mode [0:real mode  1:simulation mode]
        // 101
        int     init_state_info;        // status information of robot initialization process
        int     init_error;             // error code of robot initialization process
        // 103
        float   tfb_analog_in[2];       // analog input value of tool flange board [V]
        int     tfb_digital_in[2];      // digital input value of tool flange board [0 or 1]
        int     tfb_digital_out[2];     // digital output value of tool flange board [0 or 1]
        float   tfb_voltage_out;        // reference voltage of tool flange board [0, 12, 24]
        // 110
        int     op_stat_collision_occur;
        int     op_stat_sos_flag;
        int     op_stat_self_collision;
        int     op_stat_soft_estop_occur;
        int     op_stat_ems_flag;
        // 115
    }sdata;
    float fdata[MAX_SHARED_DATA];
    int idata[MAX_SHARED_DATA];
}systemSTAT;

typedef union{
    struct{
        char    header[4];
        // 0
        float   sensitivity;            // collision threshold [0~1]
        float   work_x_min;             // (ignore)
        float   work_x_max;             // (ignore)
        float   work_y_min;             // (ignore)
        float   work_y_max;             // (ignore)
        float   work_z_min;             // (ignore)
        float   work_z_max;             // (ignore)
        float   mount_rotate[3];        // direction of gravity [normalized vector]
        // 10
        float   toolbox_size[3];        // virtual collision box size of tool [mm]
        float   toolbox_center_pos[3];  // virtual collision box position of tool [mm]
        float   tool_mass;              // tool mass [kg]
        float   tool_mass_center_pos[3];// center of mass position of tool [mm]
        float   tool_ee_pos[3];         // tool position [mm]
        // 23
    }sdata;
    float fdata[MAX_CONFIG_DATA];
}systemCONFIG;



typedef struct{
    char    header[4];
    char    type;
    char    msg[1000];
    int     len;
}systemPOPUP;

extern int      __IS_PODO_WORKING__;

extern int      _NO_OF_MC;
extern systemSTAT   *sys_status;
extern systemCONFIG *sys_config;
extern systemPOPUP  *sys_popup;
extern systemPOPUP  *sys_custom_alarm;



// Utility ----------------------------
#define LOG_FAIL    "\033[1;31m"
#define LOG_GOOD    "\033[1;32m"
#define LOG_WARN    "\033[1;33m"
#define LOG_NORMAL  "\033[0m"


class rb10
{
public:
    rb10();

    QTcpSocket *cmdSocket;
    QTcpSocket *dataSocket;
    systemSTAT systemStat;
    systemCONFIG systemConfig;
    systemPOPUP  systemPopup;

    ReciveData *Uidata;

    int cmdConfirmFlag;
    int moveCmdFlag;
    int moveCmdCnt;

    void InitSocket(QTcpSocket *sock1,QTcpSocket *sock2,ReciveData *Info,int cord_type);

    void print(QString value);
    void RobotInfo();

    // Cobot Control API -------------------
    // <CobotInit>
    // : initialize Cobot
    void CobotInit();

    // <MoveJoint>
    // : move to target posture in joint coordinate
    // joint1~joint6 : target joint angle in deg unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveJoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd = -1, float acc = -1);

    // <MoveTCP>
    // : move to target posture in cartesian coordinate
    // x, y, z : target TCP(tool center point) position in mm unit
    // rx, ry, rz : target TCP orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveTCP(float x, float y, float z, float rx, float ry, float rz, float spd = -1, float acc = -1);

    // <MoveCircle_ThreePoint>
    // : move current position to final position while it follows circle trajectory
    // : the circle trajectory is derived from current, first, and final position
    // type : 0 - try to follow both input position and orientation
    //        1 - orientation will be fixed to current orientation
    //        2 - orientation will be changed perpendicularly starting from current orientation
    // x1, y1, z1 : first position in mm unit
    // rx1, ry1, rz1 : first orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // x2, y2, z2 : final position in mm unit
    // rx2, ry2, rz2 : final orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveCircle_ThreePoint(int type, float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd = -1, float acc = -1);

    // <MoveCircle_Axis>
    // : move current position to final position while it follows circle trajectory
    // : the circle trajectory is derived from current position, center position, axis, and rotation angle
    // type : 0 - try to follow both input position and orientation
    //        1 - orientation will be fixed to current orientation
    //        2 - orientation will be changed perpendicularly starting from current orientation
    // cx, cy, cz : center position in mm unit
    // ax, ay, az : axis representation (norminal)
    // rot_angle: rotation angle in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveCircle_Axis(int type, float cx, float cy, float cz, float ax, float ay, float az, float rot_angle, float spd, float acc);

    // <MoveJointBlend_Clear>
    // : clear joint blend list
    void MoveJointBlend_Clear();

    // <MoveJointBlend_AddPoint>
    // : add point to the joint blend list
    // : only the last point's 'vel' and 'acc' will be applied
    // joint1~joint6 : target joint angle in deg unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveJointBlend_AddPoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd = -1, float acc = -1);

    // <MoveJointBlend_MovePoint>
    // : start to move all the points in the joint blend list
    void MoveJointBlend_MovePoint();

    // <MoveTCPBlend_Clear>
    // : clear TCP blend list
    void MoveTCPBlend_Clear();

    // <MoveTCPBlend_AddPoint>
    // : add point to the TCP blend list
    // : only the last point's 'vel' and 'acc' will be applied
    // radius : blend distance in mm unit
    // x, y, z : target TCP(tool center point) position in mm unit
    // rx, ry, rz : target TCP orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveTCPBlend_AddPoint(float radius, float x, float y, float z, float rx, float ry, float rz, float spd = -1, float acc = -1);

    // <MoveTCPBlend_MovePoint>
    // : start to move all the points in the joint blend list
    void MoveTCPBlend_MovePoint();

    // <ControlBoxDigitalOut>
    // control digital out ports in control box
    // d0~d15 : digital out value (0 or 1)s
    void ControlBoxDigitalOut(int d0=0, int d1=0, int d2=0, int d3=0, int d4=0, int d5=0, int d6=0, int d7=0, int d8=0, int d9=0, int d10=0, int d11=0, int d12=0, int d13=0, int d14=0, int d15=0);

    // <ControlBoxAnalogOut>
    // control analog out ports in control box
    // a0~a3 : analog out value in voltage unit (0~10)
    void ControlBoxAnalogOut(float a0, float a1, float a2, float a3);

    // <ToolOut>
    // control digital out ports and voltage level in tool flange board
    // volt : reference voltage of tool flange board in voltage unit(0, 12, 24)
    // d0, d1 : digital out value (0 or 1)
    void ToolOut(int volt, int d0, int d1);

    // <ProgramMode_Real>
    // change to 'real robot' mode -- robot will move
    void ProgramMode_Real();

    // <ProgramMode_Simulation>
    // change to 'simulation' mode -- robot will not move except teaching
    void ProgramMode_Simulation();

    // <BaseSpeedChange>
    // change base speed -- base speed will be multiplied to motion velocity
    // spd : normalized base speed (0~1)
    void BaseSpeedChange(float spd);

    // <MotionPause>
    // pause the current motion
    void MotionPause();

    // <MotionHalt>
    // halt the current motion
    // !! CAUTION : user would better escape the motion sequence
    //            : if not, the next motion will be activated immediately
    void MotionHalt();

    // <MotionResume>
    // resume the paused motion
    void MotionResume();

    // <CollisionResume>
    // resume the motion which is paused due to external collision
    void CollisionResume();
    // -------------------------------------

    void MoveTCPitpl_Clear();
    void MoveTCPitpl_AddPoint(float radius, float x, float y, float z, float rx, float ry, float rz, float spd = -1, float acc = -1);
    void MoveTCPitpl_MovePoint();
    void SetTCP(cord TCPcord={});

private:
    int initFlag;
};

#endif // rb10_H
