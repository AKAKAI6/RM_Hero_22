
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h" 



//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1

//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0

//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2



//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_RATIO 0.006f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_RATIO 0.005f

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_RATIO 0.000002f

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_RATIO 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f


//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

//电机到中心的距离
#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//m3508转化成底盘速度(m/s)的比例，
#define CHASSIS_MOTOR_RPM_TO_VECTOR_RATIO 0.000415809748903494517209f


//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f

//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f

//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

//Wz的控制比例，防止自旋过快
#define CHASSIS_CAR_CENTER 0.18f

//底盘电机速度环PID
#define M3508_MOTOR_SPEED_PID_KP 15000.0f
#define M3508_MOTOR_SPEED_PID_KI 10.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//拨弹盘电机角度环PID
#define TRAY_MOTOR_ANGLE_PID_KP 5.0f
#define TRAY_MOTOR_ANGLE_PID_KI 0.0f
#define TRAY_MOTOR_ANGLE_PID_KD 0.0f
#define TRAY_MOTOR_ANGLE_PID_MAX_IOUT 1500.0f
#define TRAY_MOTOR_ANGLE_PID_MAX_OUT 1500.0f
//拨弹盘电机速度环pid
#define TRAY_MOTOR_SPEED_PID_KP 5.0f
#define TRAY_MOTOR_SPEED_PID_KI 0.8f
#define TRAY_MOTOR_SPEED_PID_KD 0.0f
#define TRAY_MOTOR_SPEED_PID_MAX_OUT 5000.0f
#define TRAY_MOTOR_SPEED_PID_MAX_IOUT 3000.0f

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
  
	

  chassis_motor_t motor_chassis[5];          //底盘电机数据
  
	pid_type_def motor_speed_pid[6];             //底盘电机速度pid
 
	pid_type_def motor_angle_pid[5];          //底盘电机角度pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //使用一阶低通滤波减缓设定值

  fp32 vx;                          //底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                          //底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                          //底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s即遥控器的转化数据
  fp32 vy_set;                      //底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s          

  fp32 vx_max_speed;  //前进方向最大速度 单位m/s
  fp32 vx_min_speed;  //后退方向最大速度 单位m/s
  fp32 vy_max_speed;  //左方向最大速度 单位m/s
  fp32 vy_min_speed;  //右方向最大速度 单位m/s
  
  fp32 tray_flag;
  fp32 tray_loopset;//拨弹盘
  fp32 tray_angleset;
  fp32 t_angle;
} chassis_move_t;



extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

extern chassis_move_t chassis_move;
void chassis_init(chassis_move_t *chassis_move_init);

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
#define _ARMABI_PURE __declspec(__nothrow) __attribute__((const))
extern _ARMABI_PURE double fabs(double /*x*/);

void chassis_set_contorl(chassis_move_t *chassis_move_control);

void chassis_feedback_update(chassis_move_t *chassis_move_update);

void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

void tray_control_loop(chassis_move_t *chassis_move_control_loop);

#endif
