#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"


//遥控器通道值换算到角度的比例
#define CH1_TO_YAW_RATIO 0.54545454f
#define CH2_TO_PITCH_RATIO 2.46515151f

//角度环输出死区
#define DEADBAND_ANGLEPID_OUT 30.0f
//pitch的角度环输出死区
#define DEADBAND_p_ANGLEPID_OUT 3000.0f

//速度环kpkikd
#define PITCH_SPEED_KP 14.0f//12000.0f
#define PITCH_SPEED_KI 0.05f//500.0f
#define PITCH_SPEED_KD 7.0f//5000.0f
#define PITCH_SPEED_PID_MAX_OUT 12000.0f//3000.0f
#define PITCH_SPEED_PID_MAX_IOUT 6000.0f//700.0f


#define SHOOT_SPEED_KP 16.0f//12000.0f
#define SHOOT_SPEED_KI 1.2f//500.0f
#define SHOOT_SPEED_KD 0.24f//5000.0f
#define SHOOT_SPEED_PID_MAX_OUT 15000.0f//3000.0f
#define SHOOT_SPEED_PID_MAX_IOUT 4000.0f//700.0f



#define IMU_SPEED_KP 35.0f
#define IMU_SPEED_KI 8.0f
#define IMU_SPEED_KD 2.0f
#define IMU_SPEED_PID_MAX_OUT 15000.0f
#define IMU_SPEED_PID_MAX_IOUT 10000.0f

#define IMU_ANGLE_KP 18.0f
#define IMU_ANGLE_KI 0.0f
#define IMU_ANGLE_KD 0.0f
#define IMU_ANGLE_PID_MAX_OUT 210.0f
#define IMU_ANGLE_PID_MAX_IOUT 0.0f


#define FRIC_SPEED_PID_MAX_OUT 16000.0f

//角度环kpkikd
#define PITCH_ANGLE_KP 4.0f
#define PITCH_ANGLE_KI 0.0f
#define PITCH_ANGLE_KD 0.0f
#define PITCH_ANGLE_PID_MAX_IOUT 1500.0f
#define PITCH_ANGLE_PID_MAX_OUT 1500.0f



#define SHOOT_FRIC_HIGH_SPEED 6000.0f

#define LEFT_fric 2
#define RIGHT_fric 3

#define left_f 0
#define right_f 1

#define IMU_PID 0
#define MOTOR_PID 1

#define GIMBAL_P_PID 0

typedef struct
{
  const motor_measure_t *gimbal_motor_measure;//云台电机反馈参数
  int16_t give_current;
  int16_t give_voltage;
} gimbal_motor_t;

typedef struct
{
    const RC_ctrl_t *gimbal_RC;               //云台使用的遥控器指针
  
	pid_type_def imu_angle_pid[4];
	
	pid_type_def imu_speed_pid[4];
  
	
	gimbal_motor_t motor_gimbal[5];          //云台电机数据
  
	pid_type_def motor_speed_pid[4];             //云台电机速度pid
 
	pid_type_def motor_angle_pid[4];          //云台电机角度pid

  fp32 angleset_yaw;
  fp32 angleset_pitch;
  fp32 angleset_shoot;

  fp32 angle_yaw;
  fp32 angle_pitch;
  fp32 angle_shoot;
  
  fp32 speed_yaw;
  fp32 speed_pitch;
  fp32 speed_shoot;
	
  fp32 offset_P;//pitch零点
  fp32 offset_y;//yaw零点

  fp32 fric_speedset;
  
  fp32 imu_angle_pitch;
  fp32 imu_speed_pitch;
  
  fp32 imu_anglepid_out;
} gimbal_move_t;


typedef struct
{
	fp32 IMU_actualangle;
	pid_type_def imu_pid[4];

}gimbal_y_t;

typedef struct
{
	fp32 IMU_actualangle;
	pid_type_def imu_pid[4];

}gimbal_r_t;

typedef struct
{
	fp32 IMU_actualangle;
	
	fp32 IMU_actualspeed;
	
	pid_type_def imu_angle_pid[4];
	
	const RC_ctrl_t *gimbal_RC;               
  
  gimbal_motor_t motor_gimbal[1];          
  
	pid_type_def motor_speed_pid[1];
	
	pid_type_def imu_speed_pid[1];  	
 
	pid_type_def motor_angle_pid[1];          
	
	 fp32 angleset_pitch;
	
	 fp32 angle_pitch;
	
	fp32 speed_pitch;
	
	fp32 imu_angle_pit;
	
}gimbal_p_t;

typedef struct
{
	 gimbal_motor_t motor_gimbal[2];          
  
	pid_type_def motor_speed_pid[2];             
 
	pid_type_def motor_angle_pid[2];      
	
	 fp32 speed_shoot;
	
	fp32 fric_speedset;

}shoot_t;


typedef struct{
	
	float target_speed;
	float actual_speed;
	
	float target_angle;
	float actual_angle;
	
	int last_shoot_flag;
	
	float set_currunt;
	
	float last_angle;			//上一次机械转子角度
	int rounds;				    //转过的圈数
	int total_angle;			//总共转过的角度
	int last_speed;       		//上一次真实转速
	int record_begin_angle_status;//是否记录了电机初始角度 0代表没有记录，1代表记录成功
	int begin_angle;            //电机初始角度
	
	PidTypeDef speed_pid;
	PidTypeDef angle_pid;
}trigger_t;

typedef struct
{
	float off_set;
	pid_type_def pit_speed_pid[2];             


}pit_offset_t;


typedef struct{
	
	float target_speed;
	float actual_speed;
	
	float set_currunt;
	

	PidTypeDef speed_pid;
}fric_t;

typedef struct{
	
	fric_t left_fric;
	fric_t right_fric;
	
	trigger_t trigger;
	fp32 tirgg_flag;
}shoot_task_t;




extern void gimbal_init(gimbal_move_t *gimbal_init);

extern void gimbal_ctrl_loop(gimbal_move_t *gimbal_crtl_loop);
extern void gimbal_rc_to_set(gimbal_move_t *gimbal_rc_set);

extern gimbal_move_t gimbal_move;

extern void gimbal_rc_to_ctrl_vector(gimbal_move_t *gimbal_rc_to_ctrl_vector,fp32 *FRIC,fp32 *PITCH_SET);

extern fp32 deadband_anglepid_out(fp32 anglepid_out);


extern void IMU_trans(void);

extern void fric_task(void);

extern void pitch_task(void);

extern void shoot_task(void);

extern void gimbal_task(void);

extern void set_pit_offset(void);

#endif
