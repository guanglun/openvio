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

//OUT
#define REQUEST_SET_CAMERA_STATUS 0xA0

//IN
#define REQUEST_GET_CAMERA_STATUS 0x10

#define REQUEST_CAMERA_SET_FRAME_SIZE_NUM 0xA2
#define REQUEST_CAMERA_SET_EXPOSURE 0xA3

#define REQUEST_IMU_START 0xB0
#define REQUEST_IMU_STOP 0xB1

int camera_recv(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
	int ret = 0;
	switch (cmd)
	{
	case REQUEST_SET_CAMERA_STATUS:
		if (vio_status.cam_status == SENSOR_STATUS_WAIT && pbuf[0] == 1)
		{
			vio_status.cam_status = SENSOR_STATUS_START;
		}else if (vio_status.cam_status != SENSOR_STATUS_WAIT && pbuf[0] == 0)
		{
			vio_status.cam_status = SENSOR_STATUS_WAIT;
		}

		break;
	case REQUEST_IMU_START:
		if (vio_status.imu_status == SENSOR_STATUS_WAIT)
		{
			vio_status.imu_status = SENSOR_STATUS_START;
		}
		break;
	case REQUEST_IMU_STOP:
		if (vio_status.imu_status != SENSOR_STATUS_WAIT)
		{
			vio_status.imu_status = SENSOR_STATUS_WAIT;
		}
		break;
	case REQUEST_CAMERA_SET_EXPOSURE:
		if (vio_status.cam_id == MT9V034_ID)
		{
			int exposure = (int)((pbuf[0] << 16) | pbuf[1]);
			mt9v034_exposure(exposure);
		}
		break;
	case REQUEST_CAMERA_SET_FRAME_SIZE_NUM:

		vio_status.cam_status = SENSOR_STATUS_WAIT;

		uint16_t size_num = (uint16_t)((pbuf[0] << 16) | pbuf[1]);

		if (vio_status.cam_id == OV7725_ID)
		{
			ov7725_config((framesize_t)size_num);
		}
		else if (vio_status.cam_id == MT9V034_ID)
		{
			mt9v034_config((framesize_t)size_num);
		}

		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}

int camera_ctrl(uint8_t cmd, uint8_t *pbuf)
{
	int ret = 0;
	switch (cmd)
	{
	case REQUEST_GET_CAMERA_STATUS:
		pbuf[0] = vio_status.cam_status;
		pbuf[1] = vio_status.cam_id;
		pbuf[2] = vio_status.cam_frame_size_num;
		pbuf[3] = vio_status.gs_bpp;
		pbuf[4] = vio_status.pixformat;
		ret = 5;
		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}

void StartOpenvioTask(void const *argument)
{
	uint16_t count_1s = 0;

	usb_receive_struct_init();

	flash_eeprom_load();

	openvio_status_init(&vio_status);

	lcd_init();

	MPU6050_Init();

	sdcard_init();

	camera_init();

	while (1)
	{
		// HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
		// osDelay(1000);
		// HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
		// osDelay(1000);

		// HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		// osDelay(1000);
		// HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		// osDelay(1000);

		count_1s++;
		if (count_1s >= 100)
		{
			count_1s = 0;
			HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
		}

		usb_parse_loop();
		osDelay(10);
	}
}
