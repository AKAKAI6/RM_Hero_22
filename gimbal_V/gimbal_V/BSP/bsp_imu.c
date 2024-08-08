#include "bsp_imu.h"
#include "bsp_imu_pwm.h"

#include "bmi088driver.h"
#include "ist8310driver.h"
#include  "pid.h"
#include "delay.h"
#include "MahonyAHRS.h"
#include "gimbal.h"

#include "bsp_spi.h"

#include "math.h"

#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm¸ø¶¨

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

extern SPI_HandleTypeDef hspi1;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};



volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;


bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];



static uint8_t first_temperate;
//static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static const fp32 imu_temp_PID[5] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD,TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT};
//static pid_type_def imu_temp_pid;
static PidTypeDef imu_temp_pid;
//¼ÓËÙ¶È¼ÆµÍÍ¨ÂË²¨
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
static void BMI_update(fp32 *yaw, fp32 *pitch);



fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};

fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.Å·À­½Ç µ¥Î» rad

void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);


extern gimbal_y_t gimbal_y;
extern gimbal_p_t gimbal_p;
extern gimbal_r_t gimbal_r;

/**
  * @brief          ¼ÆËãÍÓÂÝÒÇÁãÆ¯
  * @param[out]     gyro_offset:¼ÆËãÁãÆ¯
  * @param[in]      gyro:½ÇËÙ¶ÈÊý¾Ý
  * @param[out]     offset_time_count: ×Ô¶¯¼Ó1
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
        gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
        gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
        (*offset_time_count)++;
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          Ð£×¼ÍÓÂÝÒÇ
  * @param[out]     ÍÓÂÝÒÇµÄ±ÈÀýÒò×Ó£¬1.0fÎªÄ¬ÈÏÖµ£¬²»ÐÞ¸Ä
  * @param[out]     ÍÓÂÝÒÇµÄÁãÆ¯£¬²É¼¯ÍÓÂÝÒÇµÄ¾²Ö¹µÄÊä³ö×÷Îªoffset
  * @param[out]     ÍÓÂÝÒÇµÄÊ±¿Ì£¬Ã¿´ÎÔÚgyro_offsetµ÷ÓÃ»á¼Ó1,
  * @retval         none
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}
/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift, 
  * @retval         none
  */
/**
  * @brief          Ð£×¼ÍÓÂÝÒÇÉèÖÃ£¬½«´Óflash»òÕßÆäËûµØ·½´«ÈëÐ£×¼Öµ
  * @param[in]      ÍÓÂÝÒÇµÄ±ÈÀýÒò×Ó£¬1.0fÎªÄ¬ÈÏÖµ£¬²»ÐÞ¸Ä
  * @param[in]      ÍÓÂÝÒÇµÄÁãÆ¯
  * @retval         none
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

/**
  * @brief          ¿ØÖÆbmi088µÄÎÂ¶È
  * @param[in]      temp:bmi088µÄÎÂ¶È
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
  * @brief          ¸ù¾Ýimu_update_flagµÄÖµ¿ªÆôSPI DMA
  * @param[in]      temp:bmi088µÄÎÂ¶È
  * @retval         none
  */
static void imu_cmd_spi_dma(void);

/**
  * @brief          Ðý×ªÍÓÂÝÒÇ,¼ÓËÙ¶È¼ÆºÍ´ÅÁ¦¼Æ,²¢¼ÆËãÁãÆ¯,ÒòÎªÉè±¸ÓÐ²»Í¬°²×°·½Ê½
  * @param[out]     gyro: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
  * @param[out]     accel: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
  * @param[out]     mag: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
  * @param[in]      bmi088: ÍÓÂÝÒÇºÍ¼ÓËÙ¶È¼ÆÊý¾Ý
  * @param[in]      ist8310: ´ÅÁ¦¼ÆÊý¾Ý
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}


/**
  * @brief          »ñÈ¡½ÇËÙ¶È,0:xÖá, 1:yÖá, 2:rollÖá µ¥Î» rad/s
  * @param[in]      none
  * @retval         INS_gyroµÄÖ¸Õë
  */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == INT1_ACCEL_Pin)
	{
			accel_update_flag |= 1 << IMU_DR_SHFITS;
			accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
			if(imu_start_dma_flag)
			{
					imu_cmd_spi_dma();
			}
	}
	else if(GPIO_Pin == INT1_GYRO_Pin)
	{
			gyro_update_flag |= 1 << IMU_DR_SHFITS;
			if(imu_start_dma_flag)
			{
					imu_cmd_spi_dma();
			}
	}
//	else if (GPIO_Pin == IST8310_DRDY_Pin)
//	{
//		 mag_update_flag |= 1 << IMU_DR_SHFITS;  

//		if(mag_update_flag &= 1 << IMU_DR_SHFITS)
//		{
//				mag_update_flag &= ~(1<< IMU_DR_SHFITS);
//				mag_update_flag |= (1 << IMU_SPI_SHFITS);

//				ist8310_read_mag(ist8310_real_data.mag);
//		}
//	}

}

/**
  * @brief          imuÈÎÎñ, ³õÊ¼»¯ bmi088, ist8310, ¼ÆËãÅ·À­½Ç
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
static int INS_TASK_CNT=0;



void INS_task(void)
{

  INS_TASK_CNT++;
	if(INS_TASK_CNT>25) {imu_temp_control(bmi088_real_data.temp);INS_TASK_CNT=0;}
  if(gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
  {
      gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
      BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
  }

  if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
  {
      accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
  }

  if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
  {
     accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
     BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
     imu_temp_control(bmi088_real_data.temp);
  }
       imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);


        //¼ÓËÙ¶È¼ÆµÍÍ¨ÂË²¨
        //accel low-pass filter
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];
  
        	//½ÇËÙ¶ÈÁãÖµÐÞÕý£¬Ä¿µÄÊÇ½¨Á¢ËÀÇø£¬mpu²¨ÎÆ¸¡¶¯
		bmi088_real_data.gyro[0] = fabs(bmi088_real_data.gyro[0])<GIMBAL_GYRO_X_ZERO_CORRECT ? 0.0f : bmi088_real_data.gyro[0];
		bmi088_real_data.gyro[1] = fabs(bmi088_real_data.gyro[1])<GIMBAL_GYRO_Y_ZERO_CORRECT ? 0.0f : bmi088_real_data.gyro[1];
		bmi088_real_data.gyro[2] = fabs(bmi088_real_data.gyro[2])<GIMBAL_GYRO_Z_ZERO_CORRECT ? 0.0f : bmi088_real_data.gyro[2];
        
		AHRS_update(INS_quat,0.001f, bmi088_real_data.gyro, bmi088_real_data.accel, ist8310_real_data.mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
		
		gimbal_y.IMU_actualangle = INS_angle[0] / (2 * 3.141590f) * 360.0f;
		gimbal_p.IMU_actualangle = 1.00f*INS_angle[2]/(2*3.141590f)*360.0f;//ç”±äºŽå®‰è£…ä½ç½®é—®é¢˜å¯¼è‡´på’Œyçš„æ•°å€¼ç›¸å
		gimbal_p.IMU_actualspeed = 1.00f*INS_gyro[1];
		gimbal_r.IMU_actualangle = INS_angle[1]/(2*3.141590f)*360.0f;
		
}
	
    


void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
	MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 0.0f, 0.0f, 0.0f);
}
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}
uint8_t s;
void IMU_Init(void)
{  
    while(BMI088_init())
    {
        delay_ms(100);
    }
//    while(ist8310_init())
//    {
//        delay_ms(100);
//    }
	
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    
	PID_Init(&imu_temp_pid,imu_temp_PID);
	
	AHRS_init(INS_quat, bmi088_real_data.accel, ist8310_real_data.mag);
	
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;

	 //rotate and zero drift 
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
	                                                                                                                                                
	accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
}

/**
  * @brief          ¿ØÖÆbmi088µÄÎÂ¶È
  * @param[in]      temp:bmi088µÄÎÂ¶È
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
		    PID_Calc(&imu_temp_pid, 45.0f, temp);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //ÔÚÃ»ÓÐ´ïµ½ÉèÖÃµÄÎÂ¶È£¬Ò»Ö±×î´ó¹¦ÂÊ¼ÓÈÈ
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 50)
            {
                //´ïµ½ÉèÖÃÎÂ¶È£¬½«»ý·ÖÏîÉèÖÃÎªÒ»°ë×î´ó¹¦ÂÊ£¬¼ÓËÙÊÕÁ²
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 6.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**
  * @brief          ¸ù¾Ýimu_update_flagµÄÖµ¿ªÆôSPI DMA
  * @param[in]      temp:bmi088µÄÎÂ¶È
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{

        //¿ªÆôÍÓÂÝÒÇµÄDMA´«Êä
        if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
            return;
        }
        //¿ªÆô¼ÓËÙ¶È¼ÆµÄDMA´«Êä
        if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            return;
        }
        



        if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
}


void DMA2_Stream2_IRQHandler(void)
{
    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //ÍÓÂÝÒÇ¶ÁÈ¡Íê±Ï
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            
        }

        //accel read over
        //¼ÓËÙ¶È¼Æ¶ÁÈ¡Íê±Ï
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //ÎÂ¶È¶ÁÈ¡Íê±Ï
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);

        }
    }
}
