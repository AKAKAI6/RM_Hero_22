#include "pid.h"
#include "main.h"
#include "math.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_init(pid_type_def *pid,  const fp32 PID[3],fp32 deadband,fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
   
	pid->deadband = deadband;
	
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
   
	if(fabs(pid->error[0])< pid->deadband ) //死区
			pid->error[0]=0;
	
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
   
    
    return pid->out;
}



float PID_Calc(PidTypeDef *pid,  float ref,float fdb)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->ref = ref;
    pid->fdb = fdb;
    pid->error[0] = ref - fdb;
		
		if(pid->error[0]> -pid->deadband && pid->error[0]<pid->deadband) //死区
			pid->error[0]=0;

 
	pid->Pout = pid->Kp * pid->error[0];
	pid->Iout += pid->Ki * pid->error[0];
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Dbuf[0];
	LimitMax(pid->Iout, pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	LimitMax(pid->out, pid->max_out);
//		if(pid->error[0]>-pid->deadband&&pid->deadband)//死区执行积分分离操作
//			pid->out=pid->Pout+pid->Dout;
//			else
//				pid->out=pid->Pout+pid->Iout+pid->Dout;
//		
    return pid->out;
}

void PID_Init(PidTypeDef *pid,  const float PID[6])
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = PID[3];
    pid->max_iout = PID[4]; 
		pid->deadband= PID[5];
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

void PID_angleloop_init(pid_type_def *pid,  const fp32 PID[3], fp32 deadband,fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
	
	pid->deadband = deadband;
    
	pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 PID_angleloop_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
	
	
	if(fabs(pid->error[0])< pid->deadband ) //死区
			pid->error[0]=0;
	
	
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    
    
    return pid->out;
}
