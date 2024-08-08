#include "gimbal.h"
#include "stdio.h"
#include "remote_control.h"
#include "chassis_task.h"
#include "pid.h"
#include "main.h"
#include "can_comm.h"
#include "CAN_receive.h"
#include "Vision_Task.h"
#include <math.h>

shoot_task_t rc_shoot;

shoot_t shoot;

gimbal_p_t gimbal_p;
gimbal_y_t gimbal_y;
gimbal_r_t gimbal_r;



extern RC_GET_t rc_sent;
extern tray_mode_t tray_mode;

extern VISION_GET_t vision_sent;

extern VISION_MODE vision_mode;

int Gimbal_Precision_Mode = 0;

int Vision_Precision_Mode = 0;

gimbal_move_t gimbal_move;
fp32 PITCH_ch,watch_pitch_set,FRIC_ch;
int flag_fric = 2,num = 1;
fp32 pit_x,pit_y,speed_a;//0.51,0.51,10000.0f

float speed_add = 4.0f,gim_flag;
fp32 PITCH_set = 0,FRIC_set = 0;
float speed_f_set,fric_targetspeed;
float speed_add,fric_flag;

void gimbal_rc_to_ctrl_vector(gimbal_move_t *gimbal_rc_to_ctrl_vector,fp32 *FRIC,fp32 *PITCH_SET)
{
	
	static fp32 PITCH_set,FRIC_set, PITCH_set_v;
	
	PITCH_ch = gimbal_rc_to_ctrl_vector->gimbal_RC->rc.ch[1];
	FRIC_ch = gimbal_rc_to_ctrl_vector->gimbal_RC->rc.ch[4];

	//摩擦轮控制

    if(flag_fric == 0 && FRIC_ch < -200.0f)
    {
        num++;
		flag_fric = 1;
    }
	if(FRIC_ch > -200.0f)
	{
		flag_fric = 0;
	}
    if(num%2 != 0)
      {  FRIC_set = 0.0f;
		  fric_flag = 0.0f;
        
      }
    if(num%2 == 0)
      { 
		  FRIC_set = 6000.0f;
        fric_flag = 1.0f;
      }
	 
	  if(vision_mode == VISION_OFF)
	  {
		   if(PITCH_ch > 330.0f)
			{
				
				PITCH_set +=4.0f ;
				
				
				if(PITCH_set>1100.0f)
					PITCH_set = 1100.0f;

			}
			if(PITCH_ch < -330.0f)
			{

				PITCH_set -= 4.0f ;
				if(PITCH_set<8.0f)
					PITCH_set = 8.0f;
				
			}
	  
	  
		watch_pitch_set = PITCH_set;}
	  if(vision_mode == VISION_ON)
	  { 
		  
		   if(PITCH_ch > 330.0f)
			{
				
				PITCH_set_v += 0.1f ;
				
				
				if(PITCH_set_v > 40.0f)
					PITCH_set_v = 40.0f;

			}
			if(PITCH_ch < -330.0f)
			{

				PITCH_set_v -= 0.1f ;
				if(PITCH_set_v < -20.0f)
					PITCH_set_v = -20.0f;
				
			}
		  
		  watch_pitch_set = PITCH_set_v;
		 
	  }
	
	*PITCH_SET = watch_pitch_set;
    *FRIC = FRIC_set;
	 
}
fp32 deadband_p_anglepid_out(fp32 anglepid_out)
{
    if(fabs(anglepid_out)>DEADBAND_p_ANGLEPID_OUT)
		return 0;
    else
        return anglepid_out;
}

float fric_w,pit_w;
void gimbal_rc_to_set(gimbal_move_t *gimbal_rc_set)
{
    
    fp32 PITCH_SET,FRIC_SET;
    gimbal_rc_to_ctrl_vector(gimbal_rc_set,&FRIC_SET,&PITCH_SET);//遥控器数值传递包含数值映射
	
	fric_w = FRIC_SET;
	pit_w = PITCH_SET;
    gimbal_rc_set->fric_speedset = FRIC_SET;
    gimbal_rc_set->angleset_pitch = PITCH_SET;
	
}

fp32 angle_target,speed_x;

float imu_angleset;
float  pit_swing = -20;
float debug_p_vel, debug_p_cur;
float left_fric_set,left_fric_rpm,right_fric_set ,right_fric_rpm , watch_l_cur;
void gimbal_ctrl_loop(gimbal_move_t *gimbal_ctrl_loop)
{
    //YAW轴和pitch轴的角度与速度更新
    
    gimbal_ctrl_loop->angle_pitch = gimbal_ctrl_loop->motor_gimbal[1].gimbal_motor_measure->real_angle;
    gimbal_ctrl_loop->speed_pitch = gimbal_ctrl_loop->motor_gimbal[1].gimbal_motor_measure->speed_rpm;
	
	gimbal_ctrl_loop->imu_angle_pitch = gimbal_p.IMU_actualangle;
	gimbal_ctrl_loop->imu_speed_pitch = gimbal_p.IMU_actualspeed;
   //pitch轴pid控制
    if(vision_mode == VISION_OFF)
	{	
		PID_angleloop_calc(&gimbal_ctrl_loop->motor_angle_pid[MOTOR_PID], gimbal_ctrl_loop->angle_pitch, gimbal_ctrl_loop->angleset_pitch);
		PID_calc(&gimbal_ctrl_loop->motor_speed_pid[MOTOR_PID], gimbal_ctrl_loop->speed_pitch, gimbal_ctrl_loop->motor_angle_pid[MOTOR_PID].out);
		
		//限位部分防止pitch部分损坏
		if(gimbal_ctrl_loop->angleset_pitch > 1100.0f)
		{
			gimbal_ctrl_loop->angleset_pitch = 1100.0f;
			gimbal_ctrl_loop->motor_speed_pid[MOTOR_PID].out = 0.0f;
		}
		
		if(gimbal_ctrl_loop->angleset_pitch < 8.0f)
		{
			gimbal_ctrl_loop->angleset_pitch = 8.0f;
			gimbal_ctrl_loop->motor_speed_pid[MOTOR_PID].out = 0.0f;
		}
		gimbal_ctrl_loop->motor_gimbal[1].give_current =(int16_t) gimbal_ctrl_loop->motor_speed_pid[MOTOR_PID].out;
	}
	
	if(vision_mode == VISION_ON)
	{
		PID_angleloop_calc(&gimbal_ctrl_loop->imu_angle_pid[IMU_PID],  gimbal_ctrl_loop->imu_angle_pitch, vision_sent.pitch.target_angle);
//		gimbal_ctrl_loop->imu_anglepid_out = pit_targetspeed ;
		gimbal_ctrl_loop->imu_anglepid_out = gimbal_ctrl_loop->imu_angle_pid[IMU_PID].out;
		PID_calc(&gimbal_ctrl_loop->imu_speed_pid[IMU_PID], gimbal_ctrl_loop->speed_pitch, gimbal_ctrl_loop->imu_anglepid_out);
		debug_p_vel = gimbal_ctrl_loop->speed_pitch;
		
		//限位部分防止pitch机械损坏
		if(vision_sent.pitch.target_angle > 45.0f)
		{
			vision_sent.pitch.target_angle = 45.0f;
		}
		
		if(vision_sent.pitch.target_angle < 0.0f)
		{
			vision_sent.pitch.target_angle = 0.0f;
		}
		gimbal_ctrl_loop->motor_gimbal[1].give_current = (int16_t) (gimbal_ctrl_loop->imu_speed_pid[IMU_PID].out + 924.0f );
		debug_p_cur = gimbal_ctrl_loop->motor_gimbal[1].give_current;
	}
	
    //FRIC速度环控制
    
	PID_calc(&gimbal_ctrl_loop->motor_speed_pid[LEFT_fric],		gimbal_ctrl_loop->motor_gimbal[LEFT_fric].gimbal_motor_measure->speed_rpm,	-gimbal_ctrl_loop->fric_speedset);
    PID_calc(&gimbal_ctrl_loop->motor_speed_pid[RIGHT_fric],	gimbal_ctrl_loop->motor_gimbal[RIGHT_fric].gimbal_motor_measure->speed_rpm,	gimbal_ctrl_loop->fric_speedset);
    //观察值
//		speed_f_set = gimbal_ctrl_loop->fric_speedset;

	left_fric_set = -gimbal_ctrl_loop->fric_speedset;
	right_fric_set = gimbal_ctrl_loop->fric_speedset;
	
	left_fric_rpm = gimbal_ctrl_loop->motor_gimbal[LEFT_fric].gimbal_motor_measure->speed_rpm;
	right_fric_rpm = gimbal_ctrl_loop->motor_gimbal[RIGHT_fric].gimbal_motor_measure->speed_rpm;
    
	//	watch_l_cur = shoot.motor_gimbal[left_f].gimbal_motor_measure->given_current;
	watch_l_cur = gimbal_ctrl_loop->motor_gimbal[LEFT_fric].gimbal_motor_measure->given_current;

	
	//赋值电流值
    
	for(int i = 2;i<4;i++)
    {
        gimbal_ctrl_loop->motor_gimbal[i].give_current =(int16_t) gimbal_ctrl_loop->motor_speed_pid[i].out;

    }
}


void gimbal_init(gimbal_move_t *gimbal_init)
{
    
    gimbal_init->gimbal_RC = get_remote_control_point();//获取遥控器数据指针

    //速度环
   
	fp32 SHOOT_SPPED_PID[4] = {SHOOT_SPEED_KP,SHOOT_SPEED_KI,SHOOT_SPEED_KD};//编码器
	
    fp32 PITCH_SPEED_PID[4] = {PITCH_SPEED_KP,PITCH_SPEED_KI,PITCH_SPEED_KD};//编码器
	
    fp32 PITCH_ANGLE_PID[4] = {PITCH_ANGLE_KP,PITCH_ANGLE_KI,PITCH_ANGLE_KD};//编码器
	
	//imu
	fp32 IMU_ANGLE_PID[4] = {IMU_ANGLE_KP,IMU_ANGLE_KI,IMU_ANGLE_KD};//IMU
	fp32 IMU_SPEED_PID[4] = {IMU_SPEED_KP,IMU_SPEED_KI,IMU_SPEED_KD};//IMU

	
   
    for (int i = 0; i < 4; i++)//获取云台电机数据,1为PITCH，2为SHOOT
    {
        gimbal_init->motor_gimbal[i].gimbal_motor_measure = get_gimbal_motor_measure_point(i);
    }
	//gimbaltask，shoot中的獲取电机指针
	gimbal_p.motor_gimbal[GIMBAL_P_PID].gimbal_motor_measure = get_gimbal_motor_measure_point(1);
	shoot.motor_gimbal[left_f].gimbal_motor_measure = get_gimbal_motor_measure_point(2);
    shoot.motor_gimbal[right_f].gimbal_motor_measure = get_gimbal_motor_measure_point(3);
    //PITCH速度环
	
   //编码器
	PID_init(&gimbal_init->motor_speed_pid[MOTOR_PID],  PITCH_SPEED_PID,  0.2f ,  PITCH_SPEED_PID_MAX_OUT,  PITCH_SPEED_PID_MAX_IOUT);
	PID_angleloop_init(&gimbal_init->motor_angle_pid[MOTOR_PID],  PITCH_ANGLE_PID, 0.009f,  PITCH_ANGLE_PID_MAX_OUT,  PITCH_ANGLE_PID_MAX_IOUT);

	PID_init(&gimbal_p.motor_speed_pid[GIMBAL_P_PID],  PITCH_SPEED_PID,  0.2f ,  PITCH_SPEED_PID_MAX_OUT,  PITCH_SPEED_PID_MAX_IOUT);
	PID_angleloop_init(&gimbal_p.motor_angle_pid[GIMBAL_P_PID],  PITCH_ANGLE_PID,  0.07f,  PITCH_ANGLE_PID_MAX_OUT, PITCH_ANGLE_PID_MAX_IOUT);

	
	//imu
	 PID_init(&gimbal_init->imu_speed_pid[IMU_PID],  IMU_SPEED_PID,  0.2f ,  IMU_SPEED_PID_MAX_OUT,  IMU_SPEED_PID_MAX_IOUT);
	 PID_angleloop_init(&gimbal_init->imu_angle_pid[IMU_PID],  IMU_ANGLE_PID,  0.07f,  IMU_ANGLE_PID_MAX_OUT, IMU_ANGLE_PID_MAX_IOUT);
	
	PID_init(&gimbal_p.imu_speed_pid[IMU_PID],  IMU_SPEED_PID,  0.2f ,  IMU_SPEED_PID_MAX_OUT,  IMU_SPEED_PID_MAX_IOUT);
    PID_angleloop_init(&gimbal_p.imu_angle_pid[IMU_PID],  IMU_ANGLE_PID,  0.07f,  IMU_ANGLE_PID_MAX_OUT,  IMU_ANGLE_PID_MAX_IOUT);

	//FRIC1 2的速度pid初始化
	PID_init(&gimbal_init->motor_speed_pid[LEFT_fric],  SHOOT_SPPED_PID, 0.01f , SHOOT_SPEED_PID_MAX_OUT, SHOOT_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_init->motor_speed_pid[RIGHT_fric],  SHOOT_SPPED_PID, 0.01f , SHOOT_SPEED_PID_MAX_OUT, SHOOT_SPEED_PID_MAX_IOUT);
	//shoot任務中的初始化
	PID_init(&shoot.motor_speed_pid[left_f],  SHOOT_SPPED_PID, 0.01f , SHOOT_SPEED_PID_MAX_OUT, SHOOT_SPEED_PID_MAX_IOUT);
	PID_init(&shoot.motor_speed_pid[right_f],  SHOOT_SPPED_PID, 0.01f , SHOOT_SPEED_PID_MAX_OUT, SHOOT_SPEED_PID_MAX_IOUT);
	
	

}


float PITCH_set_g,add_speed_p;
float watch_yaw_ch;\
float pit_ch_rc;
void gimbal_task(void)
{			
			rc_sent.pitch.target_angle = rc_sent.pitch.target_angle*1500.0f;
		if(rc_sent.pitch.target_angle <  660.0f && rc_sent.pitch.target_angle > -660.0f)
			pit_ch_rc = rc_sent.pitch.target_angle;
	
		gimbal_p.angle_pitch = gimbal_p.motor_gimbal[GIMBAL_P_PID].gimbal_motor_measure->real_angle;
		gimbal_p.speed_pitch = gimbal_p.motor_gimbal[GIMBAL_P_PID].gimbal_motor_measure->speed_rpm;
		
		gimbal_p.imu_angle_pit = gimbal_p.IMU_actualangle;
	
		 if(pit_ch_rc > 10.0)
		{
			PITCH_set_g -= 10.0f ;
			if(PITCH_set_g<8.0f)
				PITCH_set_g = 8.0f;
			
		}
		if(pit_ch_rc < -10.0)
		{

			PITCH_set_g +=10.0f ;
			if(PITCH_set_g>1100.0f)
				PITCH_set_g = 1100.0f;
		
		}
		
		
		
		if(vision_mode == VISION_OFF)
		{
			
			gimbal_p.angleset_pitch = PITCH_set_g;
		
		
			PID_angleloop_calc(&gimbal_p.motor_angle_pid[GIMBAL_P_PID], gimbal_p.angle_pitch, gimbal_p.angleset_pitch);
			
			PID_calc(&gimbal_p.motor_speed_pid[GIMBAL_P_PID], gimbal_p.speed_pitch, gimbal_p.motor_angle_pid[GIMBAL_P_PID].out);
			
				if(gimbal_p.angleset_pitch > 1100.0f)
			{
				gimbal_p.angleset_pitch = 1100.0f;
				gimbal_p.motor_speed_pid[GIMBAL_P_PID].out = 0.0f;
			}
			
			if(gimbal_p.angleset_pitch < 8.0f)
			{
				gimbal_p.angleset_pitch = 8.0f;
				gimbal_p.motor_speed_pid[GIMBAL_P_PID].out = 0.0f;
			}
			gimbal_p.motor_gimbal[GIMBAL_P_PID].give_current =(int16_t) gimbal_p.motor_speed_pid[GIMBAL_P_PID].out;
		}
		
		if(vision_mode == VISION_ON)
		{
			
			
			
			//			PITCH_set_g = 0;2为-，3为+
			
			PID_angleloop_calc(&gimbal_p.imu_angle_pid[IMU_PID], gimbal_p.imu_angle_pit ,vision_sent.pitch.target_angle);
			
			PID_calc(&gimbal_p.imu_speed_pid[IMU_PID], gimbal_p.speed_pitch, gimbal_p.imu_angle_pid[IMU_PID].out);
			
			if(vision_sent.pitch.target_angle < -20.0f)
			{
				gimbal_p.angleset_pitch = -20.0f;
				
			}
			
			if(vision_sent.pitch.target_angle > 35.0f)
			{
				gimbal_p.angleset_pitch = 35.0f;
				
			}
			gimbal_p.motor_gimbal[GIMBAL_P_PID].give_current = (int16_t) gimbal_p.imu_speed_pid[IMU_PID].out;

		}
		
		if(vision_mode == VISION_OFF)
		{
			rc_sent.yaw.target_angle = rc_sent.yaw.target_angle*1500.0f;
			watch_yaw_ch=rc_sent.yaw.target_angle;
		}
		if(vision_mode == VISION_ON)
			rc_sent.yaw.target_angle = vision_sent.yaw.target_angle;
}
float sent_tray,fric_open_flag,watch_l_cur;
float shoot_l_rpm;
void shoot_task(void)
{	
	
	shoot.fric_speedset = rc_shoot.left_fric.target_speed; 
	
	PID_calc(&shoot.motor_speed_pid[left_f],  shoot.motor_gimbal[left_f].gimbal_motor_measure->speed_rpm,   -rc_shoot.left_fric.target_speed);//left，负转速
	PID_calc(&shoot.motor_speed_pid[right_f],  shoot.motor_gimbal[right_f].gimbal_motor_measure->speed_rpm,   rc_shoot.left_fric.target_speed);//riget，正转速
    
	if(shoot.motor_gimbal[left_f].gimbal_motor_measure->speed_rpm < -60.0f &&  shoot.motor_gimbal[right_f].gimbal_motor_measure->speed_rpm > 60.0f)
		fric_open_flag = 1;
	else
		fric_open_flag = 0;
	
	speed_f_set = -rc_shoot.left_fric.target_speed;
//	left_fric_set = -speed_f_set;
//	right_fric_set = speed_f_set;
//	
//	watch_l_cur = shoot.motor_gimbal[left_f].gimbal_motor_measure->given_current;
//	watch_r_cur = shoot.motor_gimbal[right_f].gimbal_motor_measure->given_current;

//	
	shoot_l_rpm = shoot.motor_gimbal[left_f].gimbal_motor_measure->speed_rpm;
//	right_fric_rpm = shoot.motor_gimbal[right_f].gimbal_motor_measure->speed_rpm;

	shoot.motor_gimbal[left_f].give_current = (int16_t) shoot.motor_speed_pid[left_f].out;
	shoot.motor_gimbal[right_f].give_current = (int16_t) shoot.motor_speed_pid[right_f].out;
//	if(gimbal_p.IMU_actualangle > 1.0f)
		sent_tray = tray_mode.tray*550;
//	else
//		sent_tray = 0.0f;
}

void set_pit_offset(void)
{
	if(gimbal_p.IMU_actualangle>-6.0f)
	{
			CAN_CMD_GIMBAL(-2000.0f,
							0.0f,
							0.0f);
	}
//	if(gimbal_p.IMU_actualangle<-20.0f)
//	{
//			CAN_CMD_GIMBAL(-3000.0f,
//							0.0f,
//						0.0f);
//	}
}	


