
#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "struct_typedef.h"
#include "chassis_task.h"



//拨弹盘电机控制
extern void chassis_tray_control_set(fp32 *tray_set, chassis_move_t *chassis_move_rc_to_vector);
//vx_set, 通常控制纵向移动.vy_set, 通常控制横向移动.wz_set, 通常控制旋转运动. chassis_move_rc_to_vector,  包括底盘所有信息.
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
