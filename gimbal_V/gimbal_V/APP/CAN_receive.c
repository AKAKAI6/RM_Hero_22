
#include "CAN_receive.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//电机反馈值的换算
void  get_motor_measure(motor_measure_t *ptr,uint8_t *data)                                   
{                                                               
    
	(ptr)->last_ecd = (ptr)->ecd;                                   
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            
	(ptr)->speed_rpm  = (int16_t)((data)[2]<<8 | (data)[3]);     							
	(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  
    (ptr)->temperate = (data)[6];  
	(ptr)->speed_rad = (ptr)->speed_rpm*6.0f;

}

//电机转动角度更新
void update_angle(motor_measure_t *motor_setangle)
{		
	if(motor_setangle->angle_initflag)
	{
		if(motor_setangle->ecd - motor_setangle->last_ecd > 4096)	//当前电机反馈角度-上次反馈角度超过半圈
		{																									//由于角度值为0-8292：想获得角度的积累，要用圈计数来辅助（当前反馈-上次反馈值）才能达到目的
			motor_setangle->round_cnt --;																	
		}
		if(motor_setangle->ecd - motor_setangle->last_ecd < -4096)	
		{
			motor_setangle->round_cnt ++;														//++--正好不同情况下凑整圈
		}
	}
	else		//只执行一次
	{
		motor_setangle->ecd_offset = motor_setangle->ecd;		//第一次得到的角度反馈赋给encoder_offset（零点）
		
		motor_setangle->total_angle = motor_setangle->round_cnt*8192 + motor_setangle->ecd - motor_setangle->ecd_offset;	
		motor_setangle->real_angle = (float)motor_setangle->total_angle/8192.0f * 360.0f;	
		
		motor_setangle->angle_offset = motor_setangle->real_angle;
		motor_setangle->angle_initflag = 1;
	}
	
	motor_setangle->total_angle = motor_setangle->round_cnt*8192 + motor_setangle->ecd - motor_setangle->ecd_offset;	
	motor_setangle->real_angle = (float)motor_setangle->total_angle/8192.0f * 360.0f;				//量纲转换
	
}	



static motor_measure_t motor_gimbal[7];

static CAN_TxHeaderTypeDef  gmibal_tx_message;

static CAN_TxHeaderTypeDef  pit_tx_message;

static CAN_TxHeaderTypeDef  f1f2_tx_message;


static uint8_t              gimbal_can_send_data[8];

static uint8_t              pit_can_send_data[8];

static uint8_t              f1f2_can_send_data[8];


//接受电机数据
int rxup_flag = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];


	if(hcan->Instance == CAN1)
	{	
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);

		switch (rx_header.StdId)
		{
			

			case CAN_PIT_MOTOR_ID:{get_motor_measure(&motor_gimbal[1], rx_data);
					update_angle(&motor_gimbal[1]);break;}//0x203
			
			case CAN_FRIC_1_MOTOR_ID:{get_motor_measure(&motor_gimbal[2], rx_data);
					update_angle(&motor_gimbal[2]);break;}//0x201
			
			case CAN_FRIC_2_MOTOR_ID:{get_motor_measure(&motor_gimbal[3], rx_data);
					update_angle(&motor_gimbal[3]);break;}//0x202
		   
			
			default:
			{
				break;
			}
		 }
	}
	if(hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);

		switch (rx_header.StdId)
		{
			case 0x010 :{rxup_flag = 6;break;}
		}
	}
}


int watch_can_2;

void CAN_CMD_PIT(int16_t PITCH)//tray，pitch轴，shoot控制
{
    uint32_t send_mail_box;
    pit_tx_message.StdId = 0x200;
    pit_tx_message.IDE = CAN_ID_STD;
    pit_tx_message.RTR = CAN_RTR_DATA;
    pit_tx_message.DLC = 0x08;
    pit_can_send_data[0] = 0;
    pit_can_send_data[1] = 0;
    pit_can_send_data[2] = 0;
    pit_can_send_data[3] = 0;
    pit_can_send_data[4] = PITCH >> 8;
    pit_can_send_data[5] = PITCH;
    pit_can_send_data[6] = 0;
    pit_can_send_data[7] = 0;
	
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &pit_tx_message, pit_can_send_data, &send_mail_box);
}

void CAN_CMD_F1F2(int16_t FRIC_1,int16_t FRIC_2)//tray，pitch轴，shoot控制
{
    uint32_t send_mail_box;
    f1f2_tx_message.StdId = 0x200;
    f1f2_tx_message.IDE = CAN_ID_STD;
    f1f2_tx_message.RTR = CAN_RTR_DATA;
    f1f2_tx_message.DLC = 0x08;
    f1f2_can_send_data[0] = FRIC_1>>8;
    f1f2_can_send_data[1] = FRIC_1;
    f1f2_can_send_data[2] = FRIC_2>>8;
    f1f2_can_send_data[3] = FRIC_2;
    f1f2_can_send_data[4] = 0;
    f1f2_can_send_data[5] = 0;
    f1f2_can_send_data[6] = 0;
    f1f2_can_send_data[7] = 0;
	
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &f1f2_tx_message, f1f2_can_send_data, &send_mail_box);
}

void CAN_CMD_GIMBAL(int16_t PITCH,int16_t FRIC_1,int16_t FRIC_2)//tray，pitch轴，shoot控制
{
    uint32_t send_mail_box;
    gmibal_tx_message.StdId = 0x200;
    gmibal_tx_message.IDE = CAN_ID_STD;
    gmibal_tx_message.RTR = CAN_RTR_DATA;
    gmibal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = FRIC_1>>8;
    gimbal_can_send_data[1] = FRIC_1;
    gimbal_can_send_data[2] = FRIC_2>>8;
    gimbal_can_send_data[3] = FRIC_2;
    gimbal_can_send_data[4] = PITCH >> 8;
    gimbal_can_send_data[5] = PITCH;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
	
    watch_can_2 = HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gmibal_tx_message, gimbal_can_send_data, &send_mail_box);
}


//返回云台电机指针
const motor_measure_t *get_gimbal_motor_measure_point(uint8_t i)
{
    return &motor_gimbal[i ];
}
