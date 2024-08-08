#include "stm32f4xx.h"    
#include "RC_task.h"
#include "remote_control.h"
#include "gimbal.h"
#include "math.h"
#include "Vision_Task.h"        
#include "can_comm.h"
#include "mode.h"

#include "can_receive.h"

extern shoot_task_t rc_shoot;

extern RC_ctrl_t rc_data;

extern shoot_t shoot;

extern tray_mode_t tray_mode;

extern VISION_MODE vision_mode;


int vision_switch_flag=0;//视觉开关的标识符，用来实现拨一下开关视觉，再播一下开关视觉的功能 
static int deadline_judge(uint8_t a);
float rc_cali(int max,int min, int RC_actual, int RC_max, int RC_mid, int RC_min,int deadline);
int F_flag=0;//判断F是否开启
int frie_first_flag=0;//判断第一次发单
int Fric_Switch_Flag = 0;
int Shoot_Num = 0;
int MOUSE_pre_left_cnt=0;	
//计时变量
int Chassis_Spin_Delay_Cnt					= 0;
int Gimbal_Reverse_Bottom_Delay_Cnt			= 0;
int Chassis_Reverse_Bottom_Delay_Cnt		= 0;
int Gimbal_Precision_Mode_Delay_Cnt			= 0;
int Chassis_TurnAround_Delay_Cnt			= 0;
int Fric_Switch_Delay_Cnt					= 0;
int Pitch_Calibration_Delay_Cnt				= 0;

KEY_CONTROL KEY_MODE=KEY_OFF;
extern int Sent_dataC;
//修改外部模式变量
extern int Gimbal_Precision_Mode;
extern int Last_Gimbal_Precision_Mode;
extern int Gimbal_Precision_Activated_Flag;
extern int Gimbal_Precision_Inactivated_Flag;
extern int Vision_Precision_Mode;


void control_mode_judge(void)
{
	if(rc_data.rc.ch[0]!=0||rc_data.rc.ch[1]!=0||rc_data.rc.ch[2]!=0||rc_data.rc.ch[3]!=0||rc_data.rc.ch[4]!=0)
		KEY_MODE=KEY_OFF;
	if(KEY_board||MOUSE_x||MOUSE_y||MOUSE_z)
		KEY_MODE=KEY_ON;
}
int shoot_true=0,shoot_true_cnt=0;






int R_flag=0,R_cnt=0,flag_cnt = 2;
int mode_s1,mode_s2;
int shoot_test = 0;
int mouse_left_flag = 2;
float invert_flag =  0;
void key_control_data(void)
{	

	if(R_flag==1)
		R_cnt++;
	if(R_cnt>=1000) {R_flag=0;R_cnt=0;}
		rc_sent.x_speed=0;
		rc_sent.y_speed=0;
	//Shift键低速
	if(KEY_board & KEY_PRESSED_OFFSET_SHIFT)
	{
		if(KEY_board & KEY_PRESSED_OFFSET_W)
			rc_sent.x_speed=-110.0f;
		if(KEY_board & KEY_PRESSED_OFFSET_S)
			rc_sent.x_speed=110.0f;
		if(KEY_board & KEY_PRESSED_OFFSET_A)
			rc_sent.y_speed=110.0f;
		if(KEY_board & KEY_PRESSED_OFFSET_D)
			rc_sent.y_speed=-110.0f;
			rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_RUN,KEY_YAW_ANGLE_MINN_RUN,MOUSE_x,KEY_MAXX,KEY_MINN);
		
			rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_RUN,KEY_PITCH_ANGLE_MINN_RUN,MOUSE_y,KEY_MAXX,KEY_MINN);
		
	}
	else
	{
		if(KEY_board & KEY_PRESSED_OFFSET_W)
			rc_sent.x_speed=KEY_X_SPEED_MAXX;
		if(KEY_board & KEY_PRESSED_OFFSET_S)
			rc_sent.x_speed=KEY_X_SPEED_MINN;
		if(KEY_board & KEY_PRESSED_OFFSET_A)
			rc_sent.y_speed=KEY_Y_SPEED_MINN;
		if(KEY_board & KEY_PRESSED_OFFSET_D)
			rc_sent.y_speed=KEY_Y_SPEED_MAXX;
			rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,KEY_MAXX,KEY_MINN);
			
			rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_ON,KEY_PITCH_ANGLE_MINN_ON,MOUSE_y,KEY_MAXX,KEY_MINN);
	}
	
	//Ctrl+R键底盘随动开关
	if(KEY_board & KEY_PRESSED_OFFSET_R)
	{
		if (KEY_board & KEY_PRESSED_OFFSET_CTRL)
		{
			if (Chassis_Spin_Delay_Cnt == 0)
			{
				Chassis_Spin_Delay_Cnt = 100;
				
//				tray_mode.mode_s0 = 3;//随动
				tray_mode.mode_sl = 3;
			}
		}
	}
	//R键小陀螺模式开关
	if(KEY_board & KEY_PRESSED_OFFSET_R)
	{
		if (Chassis_Spin_Delay_Cnt == 0)
		{
			Chassis_Spin_Delay_Cnt = 100;
			
//				tray_mode.mode_s0 = 3;//小陀螺
				tray_mode.mode_sl = 2;
			
		}
	}
	

	//摩擦轮开关
	if (KEY_board & KEY_PRESSED_OFFSET_G)
	{
		if (Fric_Switch_Delay_Cnt==0)
			
		{
			if (Fric_Switch_Flag == 0) Fric_Switch_Flag=1;
			else if (Fric_Switch_Flag == 1) Fric_Switch_Flag=0;
			
			rc_shoot.left_fric.target_speed = 6000.0f*Fric_Switch_Flag;
		   
			
			Fric_Switch_Delay_Cnt = 100;
		}
	}
	//发射及发射延迟
	if(MOUSE_pre_left == 1 && mouse_left_flag ==2 &&  Fric_Switch_Flag == 1)
	{	
		tray_mode.tray = 1;
		mouse_left_flag = 1;
	}
	if(MOUSE_pre_left==0)
	{
		mouse_left_flag = 2;
		tray_mode.tray = 0;
	}
				
	
//	if(KEY_PRESSED_OFFSET_B&KEY_board)//掉头
//	{
////		if (Gimbal_Precision_Mode_Delay_Cnt == 0) 
////		{
////			invert_flag = 1 ;
////			Gimbal_Precision_Mode_Delay_Cnt = 100;
////		
////		}
//	}
	//视觉模式
	if(MOUSE_pre_right)
	{
		Vision_Precision_Mode = 1;
	}
	if(MOUSE_pre_right != 1)
	{
		Vision_Precision_Mode = 0;
	}
	if(Vision_Precision_Mode==1)
		{
			vision_mode = VISION_ON;
			
//			tray_mode.mode_s1 = 1;//自瞄
			tray_mode.mode_sr = 1;
		}
		
	if(Vision_Precision_Mode==0)
	{
		
		vision_mode = VISION_OFF;
		tray_mode.mode_sr = 3;

	}
	
	
 
	//计时用
	if (Chassis_Spin_Delay_Cnt > 0)				Chassis_Spin_Delay_Cnt--;
	if (Gimbal_Precision_Mode_Delay_Cnt > 0)	Gimbal_Precision_Mode_Delay_Cnt--;
	if (Gimbal_Reverse_Bottom_Delay_Cnt > 0)	Gimbal_Reverse_Bottom_Delay_Cnt--; 
	if (Chassis_Reverse_Bottom_Delay_Cnt > 0)	Chassis_Reverse_Bottom_Delay_Cnt--;
	if (Chassis_TurnAround_Delay_Cnt>0)			Chassis_TurnAround_Delay_Cnt--;
	if (Fric_Switch_Delay_Cnt>0)				Fric_Switch_Delay_Cnt--;
	if (Pitch_Calibration_Delay_Cnt>0)			Pitch_Calibration_Delay_Cnt--;
}
/**
	* @brief       幅度判断函数
	* @param[a]		想要判断的数值
	* @retvel      1（不超限幅），0（超限幅）
*/
static int deadline_judge(uint8_t a)
{
	if(abs(a-RC_MIDD)<=DEADLINE) return 1;
	else return 0;
}

/**
	* @brief       			遥控器映射到实际速度或者实际角度
	* @param[max]		    需要映射的最大值
	* @param[min]		    需要映射的最小值
	* @param[RCactual]	遥控器通道实际值
	* @param[RCmax]		  遥控器通道最大值
	* @param[RCmid]		  遥控器通道中值
	* @param[RCmin]		  遥控器通道最小值
	* @param[deadline]	遥控器死区
	* @retvel      1（不超限幅），0（超限幅）
*/
float rc_cali(int max,int min, int RC_actual, int RC_max, int RC_mid, int RC_min, int deadline)
{
	float value;
	if((RC_actual-RC_mid)>=deadline)
	{
		value=(float)(RC_actual-RC_mid-deadline)/(RC_max-RC_min);
		value*=max;
	}
	else if((RC_actual-RC_mid)<-deadline)
	{
		value=(float)(RC_actual-RC_mid+deadline)/(RC_max-RC_min);
		value*=max;
	}
	else
		value=0;
	return value;
}




