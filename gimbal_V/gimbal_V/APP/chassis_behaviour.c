  
#include "chassis_behaviour.h"
#include "stdio.h"
#include "chassis_task.h"



 
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_RATIO * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

static void chassis_tray_control(fp32 *tray_set,chassis_move_t *chassis_move_rc_to_vector)
{
    if (tray_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *tray_set =  chassis_move_rc_to_vector->chassis_RC->rc.ch[4];
}


void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);//模式选择
    
}

void chassis_tray_control_set(fp32 *tray_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (tray_set == NULL ||  chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    chassis_tray_control(tray_set, chassis_move_rc_to_vector);
    
}


void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)//遙控器的值传给底盘控制
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
   
    // 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_RATIO;//3.6m/s左右
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_RATIO;

    
    *vx_set = vx_set_channel;
    *vy_set = vy_set_channel;
}
