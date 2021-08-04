#include "mpu6000.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "openvio_def.h"
#include "spi.h"

#include "usbd_def.h"
#include "usbd_cdc_if.h"
#include "openvio.h"

extern struct OPENVIO_STATUS vio_status;

extern SPI_HandleTypeDef hspi2;

extern USBD_HandleTypeDef hUsbDeviceHS;
osThreadId IMUTaskHandle;
SemaphoreHandle_t xIMUSemaphore;

#define MPU6000_ENABLE() HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_RESET)
#define MPU6000_DISABLE() HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_SET)

#define MPU6000_TIMEOUT_VALUE 0xFF

static int mpu6000_read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t cmd = reg;

	MPU6000_ENABLE();

	if (HAL_SPI_Transmit(&hspi2, &cmd, 1, MPU6000_TIMEOUT_VALUE) != HAL_OK)
	{
		return 1;
	}

	if (HAL_SPI_Receive(&hspi2, buf, len, MPU6000_TIMEOUT_VALUE) != HAL_OK)
	{
		return 1;
	}

	MPU6000_DISABLE();

	return 0;
}

void StartIMUTask(void const *argument)
{
	for(;;)
	{
		if(xSemaphoreTake(xIMUSemaphore, portMAX_DELAY ) == pdTRUE)
        {
			mpu6000_transmit();
        }
	}
}

static int mpu6000_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t cmd[2];
	cmd[0] = reg;
	cmd[1] = value;

	MPU6000_ENABLE();

	if (HAL_SPI_Transmit(&hspi2, cmd, 2, MPU6000_TIMEOUT_VALUE) != HAL_OK)
	{
		return 1;
	}

	MPU6000_DISABLE();
	osDelay(10);

	return 0;
}



int mpu6000_init(void)
{

	uint8_t recv[6], id;

	mpu6000_read_reg((0 | 0x80), &id, 1);
	printf("[MPU6000] ID: %02X\r\n", id);

	if (id == 0x68)
	{
		mpu6000_write_reg(MPU6000_RA_PWR_MGMT_1, 0x80);		
		osDelay(100);
		mpu6000_write_reg(MPU6000_RA_SIGNAL_PATH_RESET, 0x07);
		osDelay(100);

		mpu6000_write_reg(MPU6000_RA_PWR_MGMT_1, 0x00);
		mpu6000_write_reg(MPU6000_RA_PWR_MGMT_2, 0x00);
		
		//mpu6000_write_reg(MPU6000_RA_SMPLRT_DIV, 0x00);
		//mpu6000_write_reg(MPU6000_RA_USER_CTRL, 0x10);
		//mpu6000_write_reg(MPU6000_RA_CONFIG, 0x02);
		
		mpu6000_write_reg(MPU6000_RA_GYRO_CONFIG, (MPU6050_GYRO_FS_2000 << 3));
		mpu6000_write_reg(MPU6000_RA_ACCEL_CONFIG, (MPU6050_ACCEL_FS_4 << 3));
		
		/*enable int*/
		mpu6000_write_reg(MPU6000_RA_CONFIG,0x11);
		mpu6000_write_reg(MPU6000_RA_INT_PIN_CFG,0X9C);
		mpu6000_write_reg(MPU6000_RA_INT_ENABLE, 0x01); 
	}
	else
	{
		printf("mpu6000 init fail\r\n");
		return 1;
	}
	
	/* 创建信号量 */
//    xIMUSemaphore = xSemaphoreCreateBinary();
//    osThreadDef(IMUTask, StartIMUTask, osPriorityNormal, 0, 512);
//    IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);  
	
	return 0;
}

void mpu6000_read(uint8_t *buf)
{
	mpu6000_read_reg((MPU6000_RA_ACCEL_XOUT_H | 0x80), buf, 14);
}

void mpu6000_transmit(void)
{
	static uint8_t mpu6000_data[14];
	static short acc[3],gyro[3],temp;
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
	
	if(vio_status.is_imu_send)
	{
		vio_status.is_imu_send = 0;
		mpu6000_read(mpu6000_data);
		//MPU_Transmit_HS(mpu6000_data, 14);
		
		openvio_usb_send(SENSOR_USB_IMU,mpu6000_data, 14);

	}

	//		for(int i=0;i<14;i++)
	//		{
	//			printf("%02X ",mpu6000_data[i]);
	//		}
	//		printf("\r\n");
	
	
//	acc[0] = (short)((mpu6000_data[0]<<8)|mpu6000_data[1]);
//	acc[1] = (short)((mpu6000_data[2]<<8)|mpu6000_data[3]);
//	acc[2] = (short)((mpu6000_data[4]<<8)|mpu6000_data[5]);
//	
//	temp = (short)((mpu6000_data[6]<<8)|mpu6000_data[7]);
//	
//	gyro[0] = (short)((mpu6000_data[8]<<8)|mpu6000_data[9]);
//	gyro[1] = (short)((mpu6000_data[10]<<8)|mpu6000_data[11]);
//	gyro[2] = (short)((mpu6000_data[12]<<8)|mpu6000_data[13]);
//	
//	printf("%d %d %d %d %d %d %d\r\n", 	acc[0],acc[1],acc[2],
//										temp,
//										gyro[0],gyro[1],gyro[2]);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static BaseType_t xHigherPriorityTaskWoken;
	if(vio_status.imu_status == SENSOR_STATUS_START)
	{
		vio_status.is_imu_send = 1;
//		xSemaphoreGiveFromISR( xIMUSemaphore, &xHigherPriorityTaskWoken );
//        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
	
}