#ifndef PID_H
#define PID_H

#include "main.h"
#include "stdint.h"
#include "struct_typedef.h"

typedef struct
{
  
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;
	
	fp32 deadband;
    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;


extern void PID_init(pid_type_def *pid, const float PID[3], float max_out, float max_iout,float deadbond);


extern float PID_calc(pid_type_def *pid, float ref, float set);


extern void PID_clear(pid_type_def *pid);



typedef struct
{
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出
		float deadband;
	
    float ref;
    float fdb;
	float lastfdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;



extern void PID_init(pid_type_def *pid,  const fp32 PID[3],fp32 deadband,fp32 max_out, fp32 max_iout);

extern void PID_Init(PidTypeDef *pid,  const float PID[6]);

extern float PID_Calc(PidTypeDef *pid,  float ref,float fdb);


extern void PID_angleloop_init(pid_type_def *pid,  const fp32 PID[3],fp32 deadband ,fp32 max_out, fp32 max_iout);

extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern fp32 PID_angleloop_calc(pid_type_def *pid, fp32 ref, fp32 set);


#endif
