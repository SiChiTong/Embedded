#ifndef __tf_4wd_H__
#define __tf_4wd_H__

#include "robot_wheel_config.h"
#include "math.h"

#if ROBOT_WHEEL_MODEL == 4

class TF_4WD
{
public:
    TF_4WD();
    //set Global Speed
    void Global_Speed_Set(float* Global_Speed , float* Expect_Motor_Speed ,
                          float Global_Coordinat_Z);
    //set 3WD Robot Speed
    void Robot_Speed_Set(float* Robot_Speed , float* Expect_Motor_Speed);

    //get Robot Global Coordinate
    void Get_Global_Coordinate(float* D_Len_M_Filter , float* Mesure_Global_Coordinate);

    //get Robot Robot Coordinate
    void Get_Robot_Coordinate(float* D_Len_M_Filter , float* Mesure_Robot_Coordinate);

private:
    void Robot_To_Motor_tf(float *Robot , float *Motor , float robot_r);
    void Motor_To_Robot_tf(float *Motor , float *Robot , float robot_r);
    void Global_To_Robot_tf( float *Global , float *Robot , float R_theta);
    void Robot_To_Global_tf(float *Robot , float *Global ,float R_theta);
    void Global_To_Motor_tf(float *Global , float *Motor ,float R_theta ,float robot_r);
    void Motor_To_Global_tf(float *Motor , float *Global ,float R_theta ,float robot_r);
};

extern TF_4WD tf_4wd;

#endif

#endif  //#ifndef __TF_4WD_H__
