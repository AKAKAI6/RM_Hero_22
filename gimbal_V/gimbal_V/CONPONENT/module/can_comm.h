#ifndef __CAN_COMM_H
#define __CAN_COMM_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "gimbal.h"

typedef struct
{
	
	float x_target;
	float y_target;
	float z_target;
	float yaw_target;
	
}chassis_tx_t;


typedef struct
{
	float tray;
	float mode_sr;
	float mode_sl;
}tray_mode_t;

extern void CAN_COMM_XYZ(int16_t X,int16_t Y,int16_t Z);//X，Y，Z，IMU_Y
extern void CAN_COMM_T_MODE(int16_t TRAY,int16_t s1,int16_t s2);//拨弹盘，模式选择

extern void CANTX_IMU_UPPER(int16_t YAW,int16_t PIT);

extern void CANTX_MODE_P(int16_t s0,int16_t s1,int16_t IMU_P,int16_t IMU_Y);//????,???0x002


//extern void CANTX_XYZ_IMU(RC_ctrl_t *rc_data,int16_t IMU_Y);
void CAN_COMM_T(int16_t TRAY);//拨弹盘，模式选择

extern void CAN_COMM_Y(int16_t YAW);
extern void CanTx_Fricflag( int16_t flag , int16_t vision_yawtarget , int16_t invert_flag);



extern void CAN_COMM_UNLOAD(int16_t unload,int16_t num);
extern void CAN_C_TO_105_UNLOAD(RC_ctrl_t *rc_data,int16_t num);
//extern int mode_select(RC_ctrl_t *rc_data);

extern void CAN_C_TO_105_INA(RC_ctrl_t *rc_data);
extern void CAN_COMM_INABILITY(int16_t INA);

extern void CAN_COMM_num(int16_t num);

extern void CANTX_YAW_TARGET(int16_t YAW_TARGET);


#endif
