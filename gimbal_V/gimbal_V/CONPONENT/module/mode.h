#ifndef __MODE_H
#define __MODE_H
#include "remote_control.h"

#define R 0
#define L 1

typedef enum
{
  
    HANGING = 0, //云台随动
	FOLLOW,       //小陀螺
	TOP_ANGLE,   //无力状态
	ZERO_FORCE,
	
} MODE_t;


typedef enum
{
	UPPER_ON,
    UPPER_OFF,
} V_MODE_t;

extern void mode_select(RC_ctrl_t *rc_data);

extern  void mode_control(void);


#endif
