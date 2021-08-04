#include "openvio_task.h"
#include "openvio_def.h"
#include "openvio.h"

#include "cambus.h"
#include "mt9v034.h"
#include "ov7725.h"

#include "dcmi.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "mpu6050.h"
#include "camera.h"
#include "sd_card.h"

#include "lcd_init.h"

extern USBD_HandleTypeDef hUsbDeviceHS;
extern int frame_count;
extern int line_cnt;

DMA_BUFFER uint8_t mpu6000_data[14];

struct OPENVIO_STATUS vio_status;

#define REQUEST_CAMERA_START 0xA0
#define REQUEST_CAMERA_STOP 0xA1
#define REQUEST_CAMERA_SET_FRAME_SIZE_NUM 0xA2
#define REQUEST_CAMERA_SET_EXPOSURE 0xA3

#define REQUEST_IMU_START 0xB0
#define REQUEST_IMU_STOP 0xB1

uint8_t camera_ctrl(USBD_SetupReqTypedef *req, uint8_t *s_data)
{
	uint8_t s_len = 1;
	s_data[0] = 'F';
	switch (req->bRequest)
	{
	case REQUEST_CAMERA_START:
		if (vio_status.cam_status == SENSOR_STATUS_WAIT)
		{
			s_data[0] = 'S';
			s_data[1] = vio_status.cam_id;
			s_data[2] = vio_status.cam_frame_size_num;
			s_data[3] = vio_status.gs_bpp;
			s_data[4] = vio_status.pixformat;
			s_len = 5;
			vio_status.cam_status = SENSOR_STATUS_START;
		}
		else
		{
			s_data[0] = 'F';
		}

		break;
	case REQUEST_CAMERA_STOP:
		if (vio_status.cam_status != SENSOR_STATUS_WAIT)
		{
			s_data[0] = 'S';
			vio_status.cam_status = SENSOR_STATUS_WAIT;
		}
		else
		{
			s_data[0] = 'F';
		}

		break;
	case REQUEST_IMU_START:
		if (vio_status.imu_status == SENSOR_STATUS_WAIT)
		{
			vio_status.imu_status = SENSOR_STATUS_START;
			s_data[0] = 'S';
		}
		else
		{
			s_data[0] = 'F';
		}
		break;
	case REQUEST_IMU_STOP:
		if (vio_status.imu_status != SENSOR_STATUS_WAIT)
		{
			vio_status.imu_status = SENSOR_STATUS_WAIT;
			s_data[0] = 'S';
		}
		else
		{
			s_data[0] = 'F';
		}
		break;
	case REQUEST_CAMERA_SET_EXPOSURE:
		if (vio_status.cam_id == OV7725_ID)
		{
			s_data[0] = 'F';
		}
		else if (vio_status.cam_id == MT9V034_ID)
		{
			int exposure = (int)((req->wValue<<16)|(req->wIndex));
			mt9v034_exposure(exposure);
			s_data[0] = 'S';
		}
		break;
	case REQUEST_CAMERA_SET_FRAME_SIZE_NUM:
		//			if(vio_status.cam_status == SENSOR_STATUS_WAIT)
		//			{
		vio_status.cam_status = SENSOR_STATUS_WAIT;
		//osDelay(1000);
		if (vio_status.cam_id == OV7725_ID)
		{
			ov7725_config((framesize_t)req->wValue);
		}
		else if (vio_status.cam_id == MT9V034_ID)
		{
			mt9v034_config((framesize_t)req->wValue);
		}

		s_data[0] = 'S';
		s_data[1] = vio_status.cam_id;
		s_data[2] = vio_status.cam_frame_size_num;
		s_data[3] = vio_status.gs_bpp;
		s_data[4] = vio_status.pixformat;
		s_len = 5;
		//			}else{
		//				s_data[0] = 'F';
		//			}

		break;
	default:
		break;
	}
	return s_len;
}

void StartOpenvioTask(void const *argument)
{

	openvio_status_init(&vio_status);

	lcd_init();

	MPU6050_Init();

	sdcard_init();

	camera_init();

	while (1)
	{
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
		osDelay(1000);
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
		osDelay(1000);

		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		osDelay(1000);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		osDelay(1000);

		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
		osDelay(1000);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
		osDelay(1000);
	}
}
