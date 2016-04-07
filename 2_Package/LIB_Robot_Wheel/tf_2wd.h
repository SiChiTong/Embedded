#ifndef TF_2WD_H
#define TF_2WD_H

#include "robot_wheel_config.h"
#include "math.h"

#if ROBOT_WHEEL_MODEL == 2

class TF_2WD
{
public:
    TF_2WD()
    {
        robot_wheel_radius_=0;
        robot_body_radius_=0;
    }

    void set_robot_wheel_radius(float wheel_r){robot_wheel_radius_ = wheel_r;}
    void set_robot_body_radius(float body_r){robot_body_radius_ = body_r;}
    float get_robot_wheel_radius(void){return robot_wheel_radius_;}
    float get_robot_body_radius(void){return robot_body_radius_ ;}

    //set Global Speed
    void globalSpeedSet(float* expect_global_speed , float* expect_motor_speed ,
                        float global_coordinat_z);
    //set 3WD Robot Speed
    void robotSpeedSet(float* expect_robot_speed , float* expect_motor_speed);
    //get Robot Global Coordinate
    void getGlobalCoordinate(float* d_motor_len_filter , float* measure_global_coordinate);
    //get Robot Robot Coordinate
    void getRobotCoordinate(float* d_motor_len_filter , float* measure_robot_coordinate);

private:
    float robot_wheel_radius_;
    float robot_body_radius_;

    void robotToMotorTF(float* robot , float* motor , float robot_r);
    void motorToRobotTF(float* motor , float* robot , float robot_r);
    void globalToRobotTF( float* global , float* robot , float R_theta);
    void robotToGlobalTF(float* robot , float* global ,float R_theta);
    void globalToMotorTF(float* global , float* motor ,float R_theta ,float robot_r);
    void motorToGlobalTF(float* motor , float* global ,float R_theta ,float robot_r);
};

extern TF_2WD tf_2w;

#endif

#endif  // #ifndef TF_2WD_H

