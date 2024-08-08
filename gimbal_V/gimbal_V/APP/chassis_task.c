
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "stdio.h"
#include "main.h"
#include "bsp_can.h"
#include "tim.h"

//底盘运动数据
chassis_move_t chassis_move;

void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 5; i++)
    {
        
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_RATIO * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }
    //更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

}


void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    wheel_speed[0] = -vx_set - vy_set - CHASSIS_CAR_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set - CHASSIS_CAR_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set - CHASSIS_CAR_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set - CHASSIS_CAR_CENTER * wz_set;
}



fp32 temp_speedtarget;//速度观察值


void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    
    // 麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    // 计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

   
    // 计算pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
	
	// 赋值电流值
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}


void tray_control_loop(chassis_move_t *chassis_move_control_loop)//拨弹盘双闭环控制
{

	fp32 tray_set = 0.0f;
	chassis_tray_control_set(&tray_set, chassis_move_control_loop);//遥控器数值的传递
   	
	if(tray_set>50.0f)
	{	
		chassis_move_control_loop->tray_flag = 2;
	}
	else if(chassis_move_control_loop->tray_flag == 2)
	{
		
        chassis_move_control_loop->tray_loopset++;
		chassis_move_control_loop->tray_flag = 0;
	
    }
    //角度设置
	chassis_move_control_loop->tray_angleset = chassis_move_control_loop->tray_loopset*2.0f;
	
	chassis_move_control_loop->motor_angle_pid[0].out = PID_angleloop_calc(&chassis_move_control_loop->motor_angle_pid[0],chassis_move_control_loop->motor_chassis[4].chassis_motor_measure->real_angle,chassis_move_control_loop->tray_angleset);
	//角度环输出死区控制	
	if(fabs(chassis_move_control_loop->motor_angle_pid[0].out)<100.0f)
		chassis_move_control_loop->motor_angle_pid[0].out = 0;
	//PID计算，电流值赋值	
    PID_calc(&chassis_move_control_loop->motor_speed_pid[4], chassis_move_control_loop->motor_chassis[4].chassis_motor_measure->speed_rpm,chassis_move_control_loop->motor_angle_pid[0].out);
	chassis_move_control_loop->motor_chassis[4].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[4].out);

}


void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    // 底盘速度环pid值
    const static fp32 motor_speed_pid[4] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
	//拨弹盘速度环pid值
	const static fp32 tray_speed_pid[4] = {TRAY_MOTOR_SPEED_PID_KP, TRAY_MOTOR_SPEED_PID_KI, TRAY_MOTOR_SPEED_PID_KD};
	//拨弹盘角度环pid值
    const static fp32 tray_angle_pid[4] = {TRAY_MOTOR_ANGLE_PID_KP, TRAY_MOTOR_ANGLE_PID_KI, TRAY_MOTOR_ANGLE_PID_KD};
    
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

  
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();

    //获取底盘电机数据指针，初始化底盘和拨弹盘电机的PID 
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i],  motor_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
    }
	
	//拨弹盘电机pid初始化
	chassis_move_init->motor_chassis[4].chassis_motor_measure = get_chassis_motor_measure_point(4);
     
    //速度环
	PID_init(&chassis_move_init->motor_speed_pid[4],  tray_speed_pid, TRAY_MOTOR_SPEED_PID_MAX_OUT,TRAY_MOTOR_SPEED_PID_MAX_IOUT);
	 
    //角度环
	PID_angleloop_init(&chassis_move_init->motor_angle_pid[0], tray_angle_pid, 10000,TRAY_MOTOR_ANGLE_PID_MAX_OUT, TRAY_MOTOR_ANGLE_PID_MAX_IOUT);
	
   
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
		chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
		
		chassis_move_control->wz_set = angle_set;
    
	chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	

}


