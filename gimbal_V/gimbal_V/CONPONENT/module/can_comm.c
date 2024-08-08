#include "can_comm.h"
#include "CAN_receive.h"
#include "main.h"
#include "remote_control.h"
#include "gimbal.h"

extern RC_ctrl_t rc_data;

extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan1;
chassis_tx_t chassis_tx;

tray_mode_t tray_mode;

static CAN_TxHeaderTypeDef  XYZ_tx_message;
static CAN_TxHeaderTypeDef  TY_tx_message;
static CAN_TxHeaderTypeDef  UNLOAD_tx_message;
static CAN_TxHeaderTypeDef  INABILITY_tx_message;
static CAN_TxHeaderTypeDef  NUM_tx_message;
static CAN_TxHeaderTypeDef  IMU_tx_message;
static CAN_TxHeaderTypeDef  MODE_tx_message;
static CAN_TxHeaderTypeDef  YAW_TAARGET_tx_message;

static CAN_TxHeaderTypeDef  Y_tx_message;

static CAN_TxHeaderTypeDef Fricflag_tx_message;

static uint8_t  XYZ_can_send_data[8];
static uint8_t  TY_can_send_data[8];
static uint8_t  IMU_can_send_data[8];
static uint8_t  MODE_can_send_data[8];


static uint8_t  Fricflag_can_send_data[8];
static uint8_t  Y_can_send_data[8];



int16_t  w_IMU_Y;
void CAN_COMM_XYZ(int16_t X,int16_t Y,int16_t Z)//X，Y，Z，IMU_Y
{
    uint32_t send_mail_box;
    XYZ_tx_message.StdId = 0x001;
    XYZ_tx_message.IDE = CAN_ID_STD;
    XYZ_tx_message.RTR = CAN_RTR_DATA;
    XYZ_tx_message.DLC = 0x08;
	
	
	
    XYZ_can_send_data[0] = X >> 8;
    XYZ_can_send_data[1] = X;
    XYZ_can_send_data[2] = Y >> 8;
    XYZ_can_send_data[3] = Y;
    XYZ_can_send_data[4] = Z >> 8;
    XYZ_can_send_data[5] = Z;
    XYZ_can_send_data[6] = 0;
    XYZ_can_send_data[7] = 0;
	
	
	
    HAL_CAN_AddTxMessage(&hcan2, &XYZ_tx_message, XYZ_can_send_data, &send_mail_box);
}

void CAN_COMM_T(int16_t TRAY)//拨弹盘，模式选择
{
    uint32_t send_mail_box;
    TY_tx_message.StdId = 0x002;
    TY_tx_message.IDE = CAN_ID_STD;
    TY_tx_message.RTR = CAN_RTR_DATA;
    TY_tx_message.DLC = 0x08;
    TY_can_send_data[0] = TRAY >> 8;
    TY_can_send_data[1] = TRAY;
    TY_can_send_data[2] = 0;
    TY_can_send_data[3] = 0;
    TY_can_send_data[4] = 0;
    TY_can_send_data[5] = 0;
    TY_can_send_data[6] = 0;
    TY_can_send_data[7] = 0;
	
    HAL_CAN_AddTxMessage(&hcan2, &TY_tx_message, TY_can_send_data, &send_mail_box);

}



void CANTX_MODE_P(int16_t s0,int16_t s1,int16_t IMU_P,int16_t IMU_Y)//模式发送,原来是0x002
{
	uint32_t send_mail_box;
    MODE_tx_message.StdId = 0x006;
    MODE_tx_message.IDE = CAN_ID_STD;
    MODE_tx_message.RTR = CAN_RTR_DATA;
    MODE_tx_message.DLC = 0x08;
    MODE_can_send_data[0] =  s0;
    MODE_can_send_data[1] = s1;
    MODE_can_send_data[2] = IMU_P & 0xff;
    MODE_can_send_data[3] = IMU_P >> 8;
    MODE_can_send_data[4] = IMU_Y& 0xff;
    MODE_can_send_data[5] = IMU_Y>> 8;
    MODE_can_send_data[6] = 0;
    MODE_can_send_data[7] = 0;
	
    HAL_CAN_AddTxMessage(&hcan2, &MODE_tx_message, MODE_can_send_data, &send_mail_box);

}


void CanTx_Fricflag( int16_t flag , int16_t vision_yawtarget , int16_t invert_flag)
{
	uint32_t send_mail_box;
    Fricflag_tx_message.StdId = 0x007;
    Fricflag_tx_message.IDE = CAN_ID_STD;
    Fricflag_tx_message.RTR = CAN_RTR_DATA;
    Fricflag_tx_message.DLC = 0x08;
    Fricflag_can_send_data[0] = flag;
    Fricflag_can_send_data[1] = vision_yawtarget & 0xff;
    Fricflag_can_send_data[2] = vision_yawtarget >> 8;
    Fricflag_can_send_data[3] = invert_flag;
    Fricflag_can_send_data[4] = 0;
    Fricflag_can_send_data[5] = 0;
    Fricflag_can_send_data[6] = 0;
    Fricflag_can_send_data[7] = 0;
	
    HAL_CAN_AddTxMessage(&hcan2, &Fricflag_tx_message, Fricflag_can_send_data, &send_mail_box);


}





//保留上下位机通信
//void CANTX_IMU_UPPER(int16_t YAW,int16_t PIT)//上位机通信
//{
//    uint32_t send_mail_box;
//    NUM_tx_message.StdId = 0x011;
//    NUM_tx_message.IDE = CAN_ID_STD;
//    NUM_tx_message.RTR = CAN_RTR_DATA;
//    NUM_tx_message.DLC = 0x08;
//	NUM_tx_message.TransmitGlobalTime=DISABLE;
//    IMU_can_send_data[0] = YAW << 8;
//    IMU_can_send_data[1] = YAW;
//    IMU_can_send_data[2] = PIT <<8;
//    IMU_can_send_data[3] = PIT;
//    IMU_can_send_data[4] = 0;
//    IMU_can_send_data[5] = 0;
//    IMU_can_send_data[6] = 0;
//    IMU_can_send_data[7] = 0;
//	
//	see_imu_1 = IMU_can_send_data[0];
//	see_imu_2 = IMU_can_send_data[1];
//    watch_can_comm = HAL_CAN_AddTxMessage(&hcan1, &IMU_tx_message, IMU_can_send_data, &send_mail_box);

//}



fp32 x,y,z;
int16_t num_IMU ;
//void CANTX_XYZ_IMU(RC_ctrl_t *rc_data,int16_t IMU_Y)
//{
//	
//	
//	x = -rc_data->rc.ch[3];
//	y = -rc_data->rc.ch[2];
//	z = rc_data->rc.ch[0];

//	num_IMU = (int16_t)IMU_Y;
//	CAN_COMM_XYZ_IMU(x,y,z,num_IMU);
//}

fp32 yaw,tray,s1,s2;




