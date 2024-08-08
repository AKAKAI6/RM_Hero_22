
#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    
    CAN_YAW_MOTOR_ID = 0x20B,
    CAN_PIT_MOTOR_ID = 0x203,
    CAN_FRIC_1_MOTOR_ID = 0x201,
	CAN_FRIC_2_MOTOR_ID = 0x202,
    

} can_id_e;


typedef struct
{
    uint16_t ecd;   //转子机械角度   [0,8191]      
    int16_t  speed_rpm;//转子转速
    int16_t  given_current;//转子电流
    uint8_t  temperate;//温度
    int16_t  last_ecd;//上个角度
	
	int16_t round_cnt;//圈数
	float total_angle;//总角度
	float real_angle;//量纲化角度
	
	int16_t angle_initflag;//初始角度标签
	int16_t angle_initflag_2;//初始角度标签
	
	float ecd_offset;//零点
	
	float angle_offset;//零点
	
	float speed_rad;
	
} motor_measure_t;


extern void update_angle(motor_measure_t *motor_setangle);

extern void CAN_CMD_GIMBAL(int16_t TRAY,int16_t PITCH,int16_t SHOOT);

extern void CAN_CMD_YAW(int16_t YAW);

extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern void  get_motor_measure(motor_measure_t *ptr,uint8_t *data); 

extern void CAN_CMD_PIT(int16_t PITCH);//pitch控制

extern void CAN_CMD_F1F2(int16_t FRIC_1,int16_t FRIC_2);//shoot控制


extern const motor_measure_t *get_gimbal_motor_measure_point(uint8_t i);

extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif
