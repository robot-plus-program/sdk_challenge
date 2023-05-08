#ifndef ROBOTCONF_H
#define ROBOTCONF_H

#define rpy_deg 0
#define rpy_rad 1
#define zyz_deg 2
#define zyz_rad 3

#define rpy 0
#define zyz 1

#include <math.h>

#include <iostream>
using namespace std;
//class RobotConf
//{
//public:
//    RobotConf();
//};

//using namespace std;

typedef struct {
    double R[9];
    double position[3];
} TMatrixInfo;

typedef struct {
    double value[6];
} JointInfo;

typedef struct {
    double NUM_PNT;
    double blend;
    TMatrixInfo Pnt[100];
    JointInfo Jnt[100];

} WayPoints;
typedef struct{
    double value[6];
} cord;
//union JntData{
//    float FJnt[6];
//    double DJnt[6];

//};

typedef struct
{
    double Jnt[6];
    double TCPpos[6];
    double RobotState;
} ReciveData;

extern int rottype;

class RConf
{
public:
    RConf() ;

//    int rot_type=rpy_deg;

    void InverseRot(double *R,double *rx,double *ry,double *rz);
    void Angle2Rot(double A,double B, double C);
    TMatrixInfo Pos2Rot(ReciveData PosData,double *coor);
    TMatrixInfo Pos2Rot(ReciveData PosData,int rottype);
    TMatrixInfo Pos2Rot(ReciveData PosData);
    ReciveData Rot2Pos(TMatrixInfo matrix);
};


#endif // ROBOTCONF_H

