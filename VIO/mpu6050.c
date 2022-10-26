#include "mpu6050.h"
#include "myiic.h"
#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "openvio_def.h"
#include "lcd.h"

osThreadId MPU6050TaskHandle;
SemaphoreHandle_t xMPU6050Semaphore;
TimerHandle_t xTimerMPU6050; // 定义句柄

extern struct OPENVIO_STATUS vio_status;
extern QueueHandle_t xQueue;

uint8_t mpu6050_data[20 + 9];

#define CAL_COUNT 100
int32_t cnnt = 0, cal_gyro[3] = {0, 0, 0};

void MPU6050ReadData(short *mpudata);

/**
  * @brief   写数据到MPU6050寄存器
  * @param   reg_add:寄存器地址
	* @param	 reg_data:要写入的数据
  * @retval  
  */
void MPU6050_WriteReg(uint8_t reg_add, uint8_t reg_dat)
{
  Sensor_write(reg_add, reg_dat);
}

/**
  * @brief   从MPU6050寄存器读取数据
  * @param   reg_add:寄存器地址
	* @param	 Read：存储数据的缓冲区
	* @param	 num：要读取的数据量
  * @retval  
  */
void MPU6050_ReadData(uint8_t reg_add, unsigned char *Read, uint8_t num)
{
  Sensor_Read(reg_add, Read, num);
}

#define OX 0.433201f
#define OY -0.092553f
#define OZ -0.761175f
#define RX 1.002928f
#define RY 1.001974f
#define RZ 1.009245f

void mpu6050_transmit(void)
{
  static uint32_t t1;
  static uint16_t t2;

  static short mpu6050_read_data[6];

  static float acc1[3], acc2[3];
  static float acc_cal = 9.8f * 8.0f / 65535 * 2;

  if (vio_status.imu_status == SENSOR_STATUS_START)
  {

    get_time(&t1, &t2);

    mpu6050_data[0] = (uint8_t)(t1 >> 24);
    mpu6050_data[1] = (uint8_t)(t1 >> 16);
    mpu6050_data[2] = (uint8_t)(t1 >> 8);
    mpu6050_data[3] = (uint8_t)(t1 >> 0);
    mpu6050_data[4] = (uint8_t)(t2 >> 8);
    mpu6050_data[5] = (uint8_t)(t2 >> 0);

    MPU6050ReadData(mpu6050_read_data);

    if (cnnt < CAL_COUNT)
    {
      cnnt++;

      cal_gyro[0] += mpu6050_read_data[3];
      cal_gyro[1] += mpu6050_read_data[4];
      cal_gyro[2] += mpu6050_read_data[5];

      if (cnnt >= CAL_COUNT)
      {
        cal_gyro[0] /= CAL_COUNT;
        cal_gyro[1] /= CAL_COUNT;
        cal_gyro[2] /= CAL_COUNT;
      }

      return;
    }

    mpu6050_read_data[3] -= cal_gyro[0];
    mpu6050_read_data[4] -= cal_gyro[1];
    mpu6050_read_data[5] -= cal_gyro[2];

    //    printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", \
//          mpu6050_read_data[0], mpu6050_read_data[1], mpu6050_read_data[2], \
//          mpu6050_read_data[3], mpu6050_read_data[4], mpu6050_read_data[5]);

    acc1[0] = mpu6050_read_data[0] * acc_cal;
    acc1[1] = mpu6050_read_data[1] * acc_cal;
    acc1[2] = mpu6050_read_data[2] * acc_cal;

    acc2[0] = (acc1[0] - OX) / RX;
    acc2[1] = (acc1[1] - OY) / RY;
    acc2[2] = (acc1[2] - OZ) / RZ;

//    printf("%f\t%f\t%f;\r\n",
//           acc2[0], acc2[1], acc2[2]);

    mpu6050_read_data[0] = acc2[0] / acc_cal;
    mpu6050_read_data[1] = acc2[1] / acc_cal;
    mpu6050_read_data[2] = acc2[2] / acc_cal;


    /*ACC*/
    mpu6050_data[6] = (uint8_t)(mpu6050_read_data[0] >> 8);
    mpu6050_data[7] = (uint8_t)(mpu6050_read_data[0] >> 0);
    mpu6050_data[8] = (uint8_t)(mpu6050_read_data[1] >> 8);
    mpu6050_data[9] = (uint8_t)(mpu6050_read_data[1] >> 0);
    mpu6050_data[10] = (uint8_t)(mpu6050_read_data[2] >> 8);
    mpu6050_data[11] = (uint8_t)(mpu6050_read_data[2] >> 0);

    /*GYRO*/
    mpu6050_data[12] = (uint8_t)(mpu6050_read_data[3] >> 8);
    mpu6050_data[13] = (uint8_t)(mpu6050_read_data[3] >> 0);
    mpu6050_data[14] = (uint8_t)(mpu6050_read_data[4] >> 8);
    mpu6050_data[15] = (uint8_t)(mpu6050_read_data[4] >> 0);
    mpu6050_data[16] = (uint8_t)(mpu6050_read_data[5] >> 8);
    mpu6050_data[17] = (uint8_t)(mpu6050_read_data[5] >> 0);

    /*MAG*/
    // mpu6050_data[18] = (uint8_t)(mag_data[0] >> 8);
    // mpu6050_data[19] = (uint8_t)(mag_data[0] >> 0);
    // mpu6050_data[20] = (uint8_t)(mag_data[1] >> 8);
    // mpu6050_data[21] = (uint8_t)(mag_data[1] >> 0);
    // mpu6050_data[22] = (uint8_t)(mag_data[2] >> 8);
    // mpu6050_data[23] = (uint8_t)(mag_data[2] >> 0);
	
    struct USB_FRAME_STRUCT usb_frame_s;
    usb_frame_s.addr = (uint8_t *)mpu6050_data;
    usb_frame_s.len = 24;
    usb_frame_s.sensor = SENSOR_USB_IMU;

    xQueueSendFromISR(xQueue, (void *)&usb_frame_s, (TickType_t)0);
  }
}

//static void vTimerMPU6050Callback(TimerHandle_t xTimer)
//{
//  mpu6050_transmit();
//}

/**
  * @brief   初始化MPU6050芯片
  * @param   
  * @retval  
  */
void MPU6050_Init(void)
{

  IMU_I2C_Init();

  MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x80); //复位MPU6050
  osDelay(100);
  MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00); //解除休眠状态
  MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV, 0x07); //陀螺仪采样率
  MPU6050_WriteReg(MPU6050_RA_CONFIG, 0x06);
  MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG, 0x10); //配置加速度传感器工作在16G模式
  MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);  //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
  osDelay(50);

  if (MPU6050ReadID() == 1)
  {
    printf("[MPU6050] [Init Success]\r\n");
	  //LCD_ShowString(0,16 * 1,"[MPU6050] [Init Success]",RED,WHITE,16,0);	
	  
    // while (1)
    // {
    //   MPU6050ReadAcc(Acel);
    //   MPU6050ReadGyro(Gyro);
    //   MPU6050_ReturnTemp(&Temp);

    //   printf("%d\t%d\t%d\t%d\t%d\t%d\t%f\r\n", Acel[0], Acel[1], Acel[2], Gyro[0], Gyro[1], Gyro[2], Temp);
    //   osDelay(200);
    // }
  }
  else
  {
    printf("[MPU6050] [Init Fail]\r\n");
	  //LCD_ShowString(0,16 * 1,"[MPU6050] [Init Fail]",RED,WHITE,16,0);	
  }

  //  // 申请定时器， 配置
  //  xTimerMPU6050 = xTimerCreate
  //      /*调试用， 系统不用*/
  //      ("IMU Timer",
  //       /*定时溢出周期， 单位是任务节拍数*/
  //       10,
  //       /*是否自动重载， 此处设置周期性执行*/
  //       pdTRUE,
  //       /*记录定时器溢出次数， 初始化零, 用户自己设置*/
  //       (void *)0,
  //       /*回调函数*/
  //       vTimerMPU6050Callback);

  //  if (xTimerMPU6050 != NULL)
  //  {
  //    // 启动定时器， 0 表示不阻塞
  //    xTimerStart(xTimerMPU6050, 0);
  //    printf("[xTimerIMU][SUCCESS]\r\n");
  //  }
  //  else
  //  {
  //    printf("[xTimerIMU][FAIL]\r\n");
  //  }
}

/**
  * @brief   读取MPU6050的ID
  * @param   
  * @retval  正常返回1，异常返回0
  */
uint8_t MPU6050ReadID(void)
{
  unsigned char Re = 0;
  MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &Re, 1); //读器件地址
  if (Re != 0x68)
  {
    printf("[MPU6050][ID ERRORR]\r\n");
    return 0;
  }
  else
  {
    //printf("MPU6050 ID = %d\r\n", Re);
    return 1;
  }
}

void MPU6050ReadData(short *mpudata)
{
  uint8_t buf[14];

  MPU6050_ReadData(MPU6050_ACC_OUT, buf, 14);

  mpudata[0] = (buf[0] << 8) | buf[1];
  mpudata[1] = (buf[2] << 8) | buf[3];
  mpudata[2] = (buf[4] << 8) | buf[5];

  mpudata[3] = (buf[8] << 8) | buf[9];
  mpudata[4] = (buf[10] << 8) | buf[11];
  mpudata[5] = (buf[12] << 8) | buf[13];
}

/**
  * @brief   读取MPU6050的加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadAcc(short *accData)
{
  uint8_t buf[6];
  MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
  accData[0] = (buf[0] << 8) | buf[1];
  accData[1] = (buf[2] << 8) | buf[3];
  accData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的角加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadGyro(short *gyroData)
{
  uint8_t buf[6];
  MPU6050_ReadData(MPU6050_GYRO_OUT, buf, 6);
  gyroData[0] = (buf[0] << 8) | buf[1];
  gyroData[1] = (buf[2] << 8) | buf[3];
  gyroData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的原始温度数据
  * @param   
  * @retval  
  */
void MPU6050ReadTemp(short *tempData)
{
  uint8_t buf[2];
  MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H, buf, 2); //读取温度值
  *tempData = (buf[0] << 8) | buf[1];
}

/**
  * @brief   读取MPU6050的温度数据，转化成摄氏度
  * @param   
  * @retval  
  */
void MPU6050_ReturnTemp(float *Temperature)
{
  short temp3;
  uint8_t buf[2];

  MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H, buf, 2); //读取温度值
  temp3 = (buf[0] << 8) | buf[1];
  *Temperature = ((double)temp3 / 340.0) + 36.53;
}
