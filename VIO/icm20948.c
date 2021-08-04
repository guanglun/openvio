#include "icm20948.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "openvio_def.h"
#include "spi.h"

#include "usbd_def.h"
#include "usbd_cdc_if.h"
#include "openvio.h"
#include "ICM_20948.h"

extern struct OPENVIO_STATUS vio_status;
extern QueueHandle_t xQueue;
extern SPI_HandleTypeDef hspi2;

extern USBD_HandleTypeDef hUsbDeviceHS;
osThreadId IMUTaskHandle;
SemaphoreHandle_t xIMUSemaphore;
TimerHandle_t xTimerIMU; // 定义句柄

uint8_t imu_lock = 0;

#define ICM20948_ENABLE() HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_RESET)
#define ICM20948_DISABLE() HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_SET)

#define ICM20948_TIMEOUT_VALUE 0xFFFF
TickType_t IMUTimeNow;

// 定时器回调函数格式
static void vTimerCallback(TimerHandle_t xTimer)
{
	//HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
	//icm20948_transmit();
	//HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
}

static int icm20948_read_one_reg(uint8_t reg, uint8_t *buf)
{
	uint8_t cmd = reg | 0x80;

	ICM20948_ENABLE();

	if (HAL_SPI_Transmit(&hspi2, &cmd, 1, ICM20948_TIMEOUT_VALUE) != HAL_OK)
	{
		return 1;
	}

	if (HAL_SPI_Receive(&hspi2, buf, 1, ICM20948_TIMEOUT_VALUE) != HAL_OK)
	{
		return 1;
	}

	ICM20948_DISABLE();

	return 0;
}

static int icm20948_read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t cmd = reg | 0x80;

	ICM20948_ENABLE();

	if (HAL_SPI_Transmit(&hspi2, &cmd, 1, ICM20948_TIMEOUT_VALUE) != HAL_OK)
	{
		printf("HAL_SPI_Transmit Error \r\n");
		return 1;
	}

	if (HAL_SPI_Receive(&hspi2, buf, len, ICM20948_TIMEOUT_VALUE) != HAL_OK)
	{
		printf("HAL_SPI_Transmit Error \r\n");
		return 1;
	}

	ICM20948_DISABLE();

	return 0;
}

void StartIMUTask(void const *argument)
{
	for (;;)
	{
		if (xSemaphoreTake(xIMUSemaphore, portMAX_DELAY) == pdTRUE)
		{
			icm20948_transmit();
		}
	}
}

static int icm20948_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t cmd[2];
	cmd[0] = reg & 0x7F;
	cmd[1] = value;

	ICM20948_ENABLE();

	if (HAL_SPI_Transmit(&hspi2, cmd, 2, ICM20948_TIMEOUT_VALUE) != HAL_OK)
	{
		return 1;
	}

	ICM20948_DISABLE();
	//osDelay(1);

	return 0;
}

void icm_mag_write(uint8_t reg, uint8_t value)
{
	icm20948_write_reg(0x7F, 0x30);

	//osDelay(1);
	icm20948_write_reg(0x03, 0x0C); //mode: write

	//osDelay(1);
	icm20948_write_reg(0x04, reg); //set reg addr

	//osDelay(1);
	icm20948_write_reg(0x06, value); //send value

	//osDelay(1);
}

void us_delay(void)
{
	uint32_t t1,t1_old;
	uint16_t t2,t2_old;
	
	get_time(&t1_old, &t2_old);
	
	do{
		get_time(&t1, &t2);
	}while(t2 < (t2_old + 80));
		
}
static uint8_t icm_mag_read(uint8_t reg)
{
	uint8_t Data;
	
	icm20948_write_reg(0x7F, 0x30);
	osDelay(1);
	icm20948_write_reg(0x03, 0x0C | 0x80);
	osDelay(1);
	icm20948_write_reg(0x04, reg); // set reg addr
	osDelay(1);
	icm20948_write_reg(0x06, 0xff); //read
	osDelay(1);
	icm20948_write_reg(0x7F, 0x00);

	osDelay(1);
	icm20948_read_reg(0x3B, &Data, 1);
	osDelay(1);
	return Data;
}

static uint8_t icm_mag_read_buffer(uint8_t reg,uint8_t *buffer)
{
	uint8_t Data;
	
	icm20948_write_reg(0x7F, 0x30);
	osDelay(1);
	icm20948_write_reg(0x03, 0x0C | 0x80);
	osDelay(1);
	icm20948_write_reg(0x04, reg); // set reg addr
	osDelay(1);
	icm20948_write_reg(0x06, 0xff); //read
	osDelay(1);
	icm20948_write_reg(0x7F, 0x00);

	osDelay(1);
	icm20948_read_reg(0x3B, buffer, 6);
	osDelay(1);
	return Data;
}

void icm20948_read_mag(int16_t magn[3])
{
	uint8_t mag_buffer[10];

	//icm_mag_read_buffer(0x11,mag_buffer+1);
	
	icm20948_write_reg(0x7F, 0x30);
	osDelay(1);
	icm20948_write_reg(0x03, 0x0C | 0x80);
	osDelay(1);
	icm20948_write_reg(0x04, 0x11); // set reg addr
	osDelay(1);
	icm20948_write_reg(0x06, 0xff); //read
	osDelay(1);
	icm20948_write_reg(0x7F, 0x00);
	
	icm20948_read_reg(0x3B, mag_buffer, 6);
	
//	mag_buffer[0] = icm_mag_read(0x01);

//	mag_buffer[1] = icm_mag_read(0x11);
//	mag_buffer[2] = icm_mag_read(0x12);
	magn[0] = mag_buffer[1] | mag_buffer[2] << 8;
//	mag_buffer[3] = icm_mag_read(0x13);
//	mag_buffer[4] = icm_mag_read(0x14);
	magn[1] = mag_buffer[3] | mag_buffer[4] << 8;
//	mag_buffer[5] = icm_mag_read(0x15);
//	mag_buffer[6] = icm_mag_read(0x16);
	magn[2] = mag_buffer[5] | mag_buffer[6] << 8;

	icm_mag_write(0x31, 0x01);
}

void ICM_ReadAccelGyroData(int16_t *accel_data, int16_t *gyro_data)
{
	uint8_t raw_data[12];
	icm20948_read_reg(0x2D, raw_data, 12);

	accel_data[0] = (short)((raw_data[0] << 8) | raw_data[1]);
	accel_data[1] = (short)((raw_data[2] << 8) | raw_data[3]);
	accel_data[2] = (short)((raw_data[4] << 8) | raw_data[5]);

	gyro_data[0] = (raw_data[6] << 8) | raw_data[7];
	gyro_data[1] = (raw_data[8] << 8) | raw_data[9];
	gyro_data[2] = (raw_data[10] << 8) | raw_data[11];

	// accel_data[0] = accel_data[0] / 8;
	// accel_data[1] = accel_data[1] / 8;
	// accel_data[2] = accel_data[2] / 8;

	// gyro_data[0] = gyro_data[0] / 250;
	// gyro_data[1] = gyro_data[1] / 250;
	// gyro_data[2] = gyro_data[2] / 250;
}
void ICM_SelectBank(uint8_t bank)
{
	icm20948_write_reg(USER_BANK_SEL, bank);
}
void ICM_Disable_I2C(void)
{
	icm20948_write_reg(0x03, 0x78);
}
void ICM_SetClock(uint8_t clk)
{
	icm20948_write_reg(PWR_MGMT_1, clk);
}
void ICM_AccelGyroOff(void)
{
	icm20948_write_reg(PWR_MGMT_2, (0x38 | 0x07));
}
void ICM_AccelGyroOn(void)
{
	icm20948_write_reg(0x07, (0x00 | 0x00));
}
uint8_t icm_who_am_i(void)
{
	uint8_t spiData = 0x01;
	icm20948_read_one_reg(0x00, &spiData);
	return spiData;
}
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf)
{
	icm20948_write_reg(GYRO_CONFIG_1, ((rate << 1) | lpf));
}

int icm20948_init(void)
{
	uint8_t id = icm_who_am_i();
	printf("[ICM-20948] [ID: %02X]\r\n", id);
	if (id == 0xEA)
	{
		printf("[ICM-20948] [Init Success]\r\n");
		ICM_SelectBank(USER_BANK_0);
		osDelay(10);
		ICM_Disable_I2C();
		osDelay(10);
		ICM_SetClock((uint8_t)CLK_BEST_AVAIL);
		osDelay(10);
		ICM_AccelGyroOff();
		osDelay(20);
		ICM_AccelGyroOn();
		osDelay(10);

		ICM_SelectBank(USER_BANK_2);
		osDelay(20);
		ICM_SetGyroRateLPF(GYRO_RATE_2000, GYRO_LPF_17HZ);
		osDelay(10);

		// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
		icm20948_write_reg(0x00, 0x0A);
		osDelay(10);

		// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
		icm20948_write_reg(0x14, (0x04 | 0x11));

		// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
		icm20948_write_reg(0x10, 0x00);
		osDelay(10);

		// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
		icm20948_write_reg(0x11, 0x0A);
		osDelay(10);

		ICM_SelectBank(USER_BANK_2);
		osDelay(20);

		// Configure AUX_I2C Magnetometer (onboard ICM-20948)
		icm20948_write_reg(0x7F, 0x00);
		icm20948_write_reg(0x0F, 0x30);
		icm20948_write_reg(0x03, 0x20);
		icm20948_write_reg(0x7F, 0x30);
		icm20948_write_reg(0x01, 0x4D);
		icm20948_write_reg(0x02, 0x01);
		icm20948_write_reg(0x05, 0x81);
		icm_mag_write(0x32, 0x01);
		osDelay(1000);
		icm_mag_write(0x31, 0x02);
	}
	else
	{
		printf("[ICM-20948] [Init Fail]\r\n");
	}

	icm_init_new();
	
	ICM_SelectBank(USER_BANK_0);

	// 申请定时器， 配置
	xTimerIMU = xTimerCreate
		/*调试用， 系统不用*/
		("IMU Timer",
		 /*定时溢出周期， 单位是任务节拍数*/
		 10,
		 /*是否自动重载， 此处设置周期性执行*/
		 pdTRUE,
		 /*记录定时器溢出次数， 初始化零, 用户自己设置*/
		 (void *)0,
		 /*回调函数*/
		 vTimerCallback);

	if (xTimerIMU != NULL)
	{
		// 启动定时器， 0 表示不阻塞
		xTimerStart(xTimerIMU, 0);
		printf("[xTimerIMU][SUCCESS]\r\n");
	}else{
		printf("[xTimerIMU][FAIL]\r\n");
	}
	return 0;
}

void icm20948_read(uint8_t *buf)
{
	//ICM_SelectBank(USER_BANK_0);
	icm20948_read_reg(0x2D, buf, 14);

}

#define OX 	-0.122902
#define OY 	-0.048230
#define OZ 	0.188431
#define RX 	1.012000
#define RY 	1.012190
#define RZ 	1.016193


#define MAG_OX 	-26.020547
#define MAG_OY 	35.461694
#define MAG_OZ 	-95.002544
#define MAG_RX 	0.744489
#define MAG_RY 	0.824312
#define MAG_RZ 	0.814524

uint8_t isIMUReady = 0;
TickType_t xIMUTimeNow = 0, xIMUTimeLast = 0;
uint8_t icm20948_data[20+9];

#define CAL_COUNT 100
int32_t cnnt = 0,cal_gyro[3]={0,0,0};

#define M_PI		3.14159265358979323846

//#define CAL_ACC
//#define PRINTF_ACC
void icm20948_transmit(void)
{
	static uint32_t t1;
	static uint16_t t2;
	static int16_t accel_data[3], gyro_data[3],mag_data_t[3],mag_data[3];
	static float acc1[3],acc2[3],gyro[3];
	static float acc_cal = 9.8f*8.0f/65535*2;
	
	#ifndef CAL_ACC
	if (vio_status.imu_status == SENSOR_STATUS_START)
	#endif
	{
		//icm20948_read(icm20948_data + 6);

		icm_get_agmt_buff(icm20948_data + 6);

		
		
		get_time(&t1, &t2);

		
		
		mag_data_t[0] = ((icm20948_data[16 + 6] << 8) | (icm20948_data[15 + 6] & 0xFF)); //Mag data is read little endian
		mag_data_t[1] = ((icm20948_data[18 + 6] << 8) | (icm20948_data[17 + 6] & 0xFF));
		mag_data_t[2] = ((icm20948_data[20 + 6] << 8) | (icm20948_data[19 + 6] & 0xFF));

			mag_data[0] = (mag_data_t[0]-MAG_OX)/MAG_RX;
        	mag_data[1] = (mag_data_t[1]-MAG_OY)/MAG_RY;
        	mag_data[2] = (mag_data_t[2]-MAG_OZ)/MAG_RZ;

//			mag_data[0] = mag_data_t[0];
//        	mag_data[1] = mag_data_t[1];
//        	mag_data[2] = mag_data_t[2];
		
		//printf("%d\t%d\t%d\r\n",mag_data[0], mag_data[1], mag_data[2]);
		
		//return;

		icm20948_data[0] = (uint8_t)(t1 >> 24);
		icm20948_data[1] = (uint8_t)(t1 >> 16);
		icm20948_data[2] = (uint8_t)(t1 >> 8);
		icm20948_data[3] = (uint8_t)(t1 >> 0);
		icm20948_data[4] = (uint8_t)(t2 >> 8);
		icm20948_data[5] = (uint8_t)(t2 >> 0);

		accel_data[0] = (short)((icm20948_data[0 + 6] << 8) | icm20948_data[1 + 6]);
		accel_data[1] = (short)((icm20948_data[2 + 6] << 8) | icm20948_data[3 + 6]);
		accel_data[2] = (short)((icm20948_data[4 + 6] << 8) | icm20948_data[5 + 6]);

		gyro_data[0] = (icm20948_data[6 + 6] << 8) | icm20948_data[7 + 6];
		gyro_data[1] = (icm20948_data[8 + 6] << 8) | icm20948_data[9 + 6];
		gyro_data[2] = (icm20948_data[10 + 6] << 8) | icm20948_data[11 + 6];

		// printf("%d\t%d\t%d\t%d\t%d\t%d\r\n",
		// 	   accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2]);
		
		acc1[0] = accel_data[0]*acc_cal;
		acc1[1] = accel_data[1]*acc_cal;
		acc1[2] = accel_data[2]*acc_cal;
		
		#ifdef CAL_ACC
		printf("%f\t%f\t%f;\r\n", acc1[0], acc1[1], acc1[2]);		
		return;
		#endif
		
		acc2[0] = (acc1[0]-OX)/RX;
        acc2[1] = (acc1[1]-OY)/RY;
        acc2[2] = (acc1[2]-OZ)/RZ;
		
		accel_data[0] = acc2[0] / acc_cal;
		accel_data[1] = acc2[1] / acc_cal;
		accel_data[2] = acc2[2] / acc_cal;
		
//		printf("%f\t%f\t%f;\r\n", \
//			acc2[0], acc2[1], acc2[2]);


		if(cnnt<CAL_COUNT)
		{
			cnnt++;
			
			cal_gyro[0] += gyro_data[0];
			cal_gyro[1] += gyro_data[1];
			cal_gyro[2] += gyro_data[2];
			
			if(cnnt >= CAL_COUNT)
			{
				cal_gyro[0] /= CAL_COUNT;
				cal_gyro[1] /= CAL_COUNT;
				cal_gyro[2] /= CAL_COUNT;
			}
			
			return;
		}
		
		gyro_data[0] -= cal_gyro[0];
		gyro_data[1] -= cal_gyro[1];
		gyro_data[2] -= cal_gyro[2];
		
		/*ACC*/
		icm20948_data[6] = (uint8_t)(accel_data[0] >> 8);
		icm20948_data[7] = (uint8_t)(accel_data[0] >> 0);
		icm20948_data[8] = (uint8_t)(accel_data[1] >> 8);
		icm20948_data[9] = (uint8_t)(accel_data[1] >> 0);
		icm20948_data[10] = (uint8_t)(accel_data[2] >> 8);
		icm20948_data[11] = (uint8_t)(accel_data[2] >> 0);

		/*GYRO*/
		icm20948_data[12] = (uint8_t)(gyro_data[0] >> 8);
		icm20948_data[13] = (uint8_t)(gyro_data[0] >> 0);
		icm20948_data[14] = (uint8_t)(gyro_data[1] >> 8);
		icm20948_data[15] = (uint8_t)(gyro_data[1] >> 0);
		icm20948_data[16] = (uint8_t)(gyro_data[2] >> 8);
		icm20948_data[17] = (uint8_t)(gyro_data[2] >> 0);

		/*MAG*/
		icm20948_data[18] = (uint8_t)(mag_data[0] >> 8);
		icm20948_data[19] = (uint8_t)(mag_data[0] >> 0);
		icm20948_data[20] = (uint8_t)(mag_data[1] >> 8);
		icm20948_data[21] = (uint8_t)(mag_data[1] >> 0);
		icm20948_data[22] = (uint8_t)(mag_data[2] >> 8);
		icm20948_data[23] = (uint8_t)(mag_data[2] >> 0);	
		
		gyro[0] = gyro_data[0] * (500.0 / 65536.0) * (M_PI / 180.0);
		gyro[1] = gyro_data[1] * (500.0 / 65536.0) * (M_PI / 180.0);
		gyro[2] = gyro_data[2] * (500.0 / 65536.0) * (M_PI / 180.0);
	
		#ifdef PRINTF_ACC
//		printf("%d\t%d\t%d\t%d\t%d\t%d\r\n",
//		 	   accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2]);	
//		printf("%f\t%f\t%f\t%f\t%f\t%f\r\n", \
//			acc2[0], acc2[1], acc2[2],gyro[0],gyro[1],gyro[2]);
		#endif

		//while (MPU_Transmit_HS(icm20948_data, 24) != 0);
		
		struct USB_FRAME_STRUCT usb_frame_s;
        usb_frame_s.addr = (uint8_t *)icm20948_data;
        usb_frame_s.len = 24;
        usb_frame_s.sensor = SENSOR_USB_IMU;
        
        xQueueSendFromISR(xQueue, (void *)&usb_frame_s, (TickType_t)0);
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	//	static BaseType_t xHigherPriorityTaskWoken;
	//	if (vio_status.imu_status == SENSOR_STATUS_START)
	//	{
	//		vio_status.is_imu_send = 1;
	//		//		xSemaphoreGiveFromISR( xIMUSemaphore, &xHigherPriorityTaskWoken );
	//		//        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	//	}
}
