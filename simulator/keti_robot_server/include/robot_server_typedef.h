#include <iostream>
#include <vector>

#define MAX_SHARED_DATA 116

typedef union{
    struct{
        char header[4];
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

typedef struct{
    double joint[6];
    double pose[6*5];
    double mat[16*5];
    int cmd;
    int num;
    double x, y, z, w;
}RobotCmd;

typedef struct{
    int state;
    std::vector<double> current_joint;
    std::vector<double> current_T_matrix;
}RobotState;

typedef struct{
    int state;
    double width;
}GripperState;

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