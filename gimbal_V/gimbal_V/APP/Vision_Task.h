#ifndef __VISION_TASK_H
#define __VISION_TASK_H

typedef enum
{
	VISION_ON = 0,
	VISION_OFF,
	
}VISION_MODE;

typedef struct
{
	float target_angle;
	

}GIMBAL_VI_t;

typedef struct
{
	float target_x;
	float target_y;
	float target_z;

}CHASSIS_VI_t;

//上位机获取的信息
typedef struct{
	GIMBAL_VI_t yaw;
	GIMBAL_VI_t pitch;
	CHASSIS_VI_t chassis;
}VISION_GET_t;

void Vision_Task(void);


#endif
