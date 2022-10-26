#include "camera.h"

#include "openvio.h"
#include "mt9v034.h"
#include "ov7725.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "cambus.h"
#include "config.h"

extern struct EEPROM_CONFIG_STRUCT eeprom;

extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;
extern TIM_HandleTypeDef htim2;

DMA_BUFFER uint8_t dcmi_image_buffer[CAM_PACKAGE_MAX_SIZE * 2] = {0};

volatile uint8_t is_send_cam_head = 0;
TimerHandle_t xTimerCAM;
SemaphoreHandle_t xSemaphore;

HAL_StatusTypeDef USER_DCMI_Start_DMA(DCMI_HandleTypeDef *hdcmi, uint32_t DCMI_Mode, uint32_t pData, uint32_t Length);

// 定时器回调函数格式
static void vCAMTimerCallback(TimerHandle_t xTimer)
{
    //if (vio_status.cam_status == SENSOR_STATUS_START)
    {
        //dcmi_dma_start();
    }
}

void camera_timer_init(int fps)
{
    // 申请定时器， 配置
    xTimerCAM = xTimerCreate
        /*调试用， 系统不用*/
        ("CAM Timer",
         /*定时溢出周期， 单位是任务节拍数*/
         1000 / fps,
         //100,
         //50,
         //20,
         /*是否自动重载， 此处设置周期性执行*/
         pdTRUE,
         /*记录定时器溢出次数， 初始化零, 用户自己设置*/
         (void *)0,
         /*回调函数*/
         vCAMTimerCallback);

    if (xTimerCAM != NULL)
    {
        // 启动定时器， 0 表示不阻塞
        xTimerStart(xTimerCAM, 0);
        printf("[xTimerCAM][SUCCESS]\r\n");
    }
    else
    {
        printf("[xTimerCAM][FAIL]\r\n");
    }
}

void camera_start(void)
{
    //__HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME | DCMI_IT_LINE);
    __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
    HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)dcmi_image_buffer, vio_status.cam_frame_size / 4 * vio_status.gs_bpp); //DCMI启动DMA通道
}

void camera_stop(void)
{
    __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_FRAME);
    HAL_DCMI_Stop(&hdcmi);
}


void camera_init(void)
{
    uint8_t chip_id, cam_slv_addr;

    xSemaphore = xSemaphoreCreateBinary();

    HAL_DCMI_Stop(&hdcmi);
    
    HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    cam_slv_addr = cambus_scan();

    printf("[cam slv addr][%02X]\r\n", cam_slv_addr);

    switch (cam_slv_addr)
    {
    case OV7725_SLV_ADDR:                                                    // Same for OV7690.
        HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLL1QCLK, RCC_MCODIV_10); //8 or 9
        cambus_readb(cam_slv_addr, OV_CHIP_ID, &chip_id);
        break;
    case MT9V034_SLV_ADDR:
        HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLL1QCLK, RCC_MCODIV_6);
        cambus_readb(cam_slv_addr, ON_CHIP_ID, &chip_id);
        break;
    default:
        break;
    }

    printf("[chip id][%02X]\r\n", chip_id);

    switch (chip_id)
    {
    case OV7725_ID:
        vio_status.cam_id = OV7725_ID;
        vio_status.gs_bpp = 2;
        vio_status.cam_frame_size_num = FRAMESIZE_VGA; //FRAMESIZE_QVGA;//FRAMESIZE_MLCD; //
        printf("[CAM CHIP][OV7725]\r\n");
        ov7725_init();
        
        HAL_Delay(10);
        camera_timer_init(20);
        break;
    case MT9V034_ID:
        vio_status.pixformat = PIXFORMAT_GRAYSCALE;
        vio_status.cam_id = MT9V034_ID;
        vio_status.gs_bpp = 1;
        vio_status.cam_frame_size_num = FRAMESIZE_WVGA2;
        printf("[CAM CHIP][MT9V034]\r\n");
        mt9v034_init();

        //printf("eeprom.exposure %d\r\n",eeprom.exposure);

        vio_status.exposure = eeprom.exposure;
        mt9v034_exposure(eeprom.exposure);

        vio_status.is_sync_mode = eeprom.is_sync_mode;
        set_triggered_mode(vio_status.is_sync_mode);

        vio_status.camera_fps = eeprom.camera_fps;
        USER_MX_TIM6_Init(eeprom.camera_fps);

        vio_status.infrared_pwm = eeprom.infrared_pwm;      
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, eeprom.infrared_pwm);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, eeprom.infrared_pwm);
        //HAL_Delay(10);
        //camera_timer_init(30);
        //camera_start();
        break;
    default:

        break;
    }
}

extern QueueHandle_t xQueue;
void USER_DCMI_MemDMAXferCplt(uint32_t data, uint32_t size)
{

    if (vio_status.cam_status == SENSOR_STATUS_START && is_send_cam_head == 1)
    {
        //HAL_GPIO_WritePin(GPIOD, TEST2_Pin, GPIO_PIN_SET);
        if (vio_status.cam_frame_size * vio_status.gs_bpp > CAM_PACKAGE_MAX_SIZE)
        {

            struct USB_FRAME_STRUCT usb_frame_s;
            usb_frame_s.addr = (uint8_t *)data;
            usb_frame_s.len = size;
            usb_frame_s.sensor = SENSOR_USB_CAM;

            xQueueSendFromISR(xQueue, (void *)&usb_frame_s, (TickType_t)0);
        }
        //HAL_GPIO_WritePin(GPIOD, TEST2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        LCD_Show_Cam((uint8_t *)data, size);
        is_send_cam_head = 0;
    }
}
static uint32_t t1;
static uint16_t t2;
uint8_t cam_head[6 + 6] = "CAMERA";


static line_cnt = 0;
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    HAL_GPIO_TogglePin(TEST1_GPIO_Port, TEST1_Pin);
    HAL_GPIO_WritePin(DCMI_FSYNC_GPIO_Port, DCMI_FSYNC_Pin, GPIO_PIN_RESET);
    
    if(vio_status.cam_status != SENSOR_STATUS_WAIT)
    {
        get_time(&t1, &t2);

        cam_head[6 + 0] = (uint8_t)(t1 >> 24);
        cam_head[6 + 1] = (uint8_t)(t1 >> 16);
        cam_head[6 + 2] = (uint8_t)(t1 >> 8);
        cam_head[6 + 3] = (uint8_t)(t1 >> 0);
        cam_head[6 + 4] = (uint8_t)(t2 >> 8);
        cam_head[6 + 5] = (uint8_t)(t2 >> 0);

        openvio_usb_send(SENSOR_USB_CAM, cam_head, 12);
        openvio_usb_send(SENSOR_USB_CAM, dcmi_image_buffer, vio_status.cam_frame_size * vio_status.gs_bpp);

        line_cnt = 0;
    }
}


void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    // HAL_GPIO_WritePin(GPIOD, TEST1_Pin, GPIO_PIN_SET);
    // if(line_cnt < 480)
    // {
    //     for(int i=0;i<752;i++)
    //     {
    //         dcmi_image_buffer[752*line_cnt+i] = dcmi_image_buffer[752*line_cnt+i] > 200 ?255:0;
    //     }
    //     line_cnt++;
    // }
    // HAL_GPIO_WritePin(GPIOD, TEST1_Pin, GPIO_PIN_RESET);
}

const int resolution[][2] = {
    {0, 0},
    // C/SIF Resolutions
    {88, 72},   /* QQCIF     */
    {176, 144}, /* QCIF      */
    {352, 288}, /* CIF       */
    {88, 60},   /* QQSIF     */
    {176, 120}, /* QSIF      */
    {352, 240}, /* SIF       */
    // VGA Resolutions
    {40, 30},   /* QQQQVGA   */
    {80, 60},   /* QQQVGA    */
    {160, 120}, /* QQVGA     */
    {320, 240}, /* QVGA      */
    {640, 480}, /* VGA       */
    {60, 40},   /* HQQQVGA   */
    {120, 80},  /* HQQVGA    */
    {240, 160}, /* HQVGA     */
    // FFT Resolutions
    {64, 32},   /* 64x32     */
    {64, 64},   /* 64x64     */
    {128, 64},  /* 128x64    */
    {128, 128}, /* 128x64    */
    // Other
    {128, 160},   /* LCD       */
    {128, 160},   /* QQVGA2    */
    {720, 480},   /* WVGA      */
    {752, 480},   /* WVGA2     */
    {800, 600},   /* SVGA      */
    {1024, 768},  /* XGA       */
    {1280, 1024}, /* SXGA      */
    {1600, 1200}, /* UXGA      */

    {240, 240}, /* LCD      */
};

// static void USER_DCMI_DMAXferCplt(DMA_HandleTypeDef *hdma)
// {

//     uint32_t tmp;

//     DCMI_HandleTypeDef *hdcmi = (DCMI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
//     //str[cnt++] = '|';
//     if (hdcmi->XferCount != 0U)
//     {
//         /* Update memory 0 address location */
//         tmp = ((((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->CR) & DMA_SxCR_CT);
//         if (((hdcmi->XferCount % 2U) == 0U) && (tmp != 0U))
//         {
//             //str[cnt++] = '1';
//             tmp = ((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M0AR;
//             USER_DCMI_MemDMAXferCplt(tmp, hdcmi->XferSize * 4);
//             hdcmi->XferCount--;
//         }
//         /* Update memory 1 address location */
//         else if ((((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->CR & DMA_SxCR_CT) == 0U)
//         {
//             //str[cnt++] = '2';
//             tmp = ((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M1AR;
//             USER_DCMI_MemDMAXferCplt(tmp, hdcmi->XferSize * 4);
//             hdcmi->XferCount--;
//         }
//         else
//         {
//             //str[cnt++] = '3';
//             /* Nothing to do */
//         }
//     }
//     /* Update memory 0 address location */
//     else if ((((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->CR & DMA_SxCR_CT) != 0U)
//     {
//         //str[cnt++] = '4';
//         tmp = ((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M0AR;
//         USER_DCMI_MemDMAXferCplt((uint32_t)tmp, hdcmi->XferSize * 4);
//         //((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M0AR = hdcmi->pBuffPtr;
//     }
//     /* Update memory 1 address location */
//     else if ((((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->CR & DMA_SxCR_CT) == 0U)
//     {
//         //str[cnt++] = '5';
//         tmp = ((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M1AR;

//         USER_DCMI_MemDMAXferCplt((uint32_t)tmp, hdcmi->XferSize * 4);
//         //tmp = hdcmi->pBuffPtr;
//         //((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M1AR = (tmp + (4U*hdcmi->XferSize));
//         hdcmi->XferCount = hdcmi->XferTransferNumber;

//         //HAL_DCMI_Stop(hdcmi);
//     }
//     else
//     {
//         /* Nothing to do */
//     }

//     /* Check if the frame is transferred */
//     if (hdcmi->XferCount == hdcmi->XferTransferNumber)
//     {
//         //   if(((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M1AR > ((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M0AR)
//         //   {
//         //       str[cnt++] = '6';
//         //       USER_DCMI_MemDMAXferCplt(((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M1AR,hdcmi->XferSize*4);
//         //   }
//         //   else
//         //   {
//         //       str[cnt++] = '7';
//         //       USER_DCMI_MemDMAXferCplt(((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M0AR,hdcmi->XferSize*4);
//         //   }

//         //str[cnt++] = '8';
//         /* Enable the Frame interrupt */
//         __HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_FRAME);

//         /* When snapshot mode, set dcmi state to ready */
//         if ((hdcmi->Instance->CR & DCMI_CR_CM) == DCMI_MODE_SNAPSHOT)
//         {
//             //str[cnt++] = '9';
//             hdcmi->State = HAL_DCMI_STATE_READY;
//         }
//     }
// }

// HAL_StatusTypeDef USER_DCMI_Start_DMA(DCMI_HandleTypeDef *hdcmi, uint32_t DCMI_Mode, uint32_t pData, uint32_t Length)
// {
//     /* Initialize the second memory address */
//     uint32_t SecondMemAddress;

//     /* Check function parameters */
//     assert_param(IS_DCMI_CAPTURE_MODE(DCMI_Mode));

//     /* Process Locked */
//     __HAL_LOCK(hdcmi);

//     /* Lock the DCMI peripheral state */
//     hdcmi->State = HAL_DCMI_STATE_BUSY;

//     /* Enable DCMI by setting DCMIEN bit */
//     __HAL_DCMI_ENABLE(hdcmi);

//     /* Configure the DCMI Mode */
//     hdcmi->Instance->CR &= ~(DCMI_CR_CM);
//     hdcmi->Instance->CR |= (uint32_t)(DCMI_Mode);

//     /* Set the DMA memory0 conversion complete callback */
//     hdcmi->DMA_Handle->XferCpltCallback = USER_DCMI_DMAXferCplt;

//     /* Set the DMA error callback */
//     hdcmi->DMA_Handle->XferErrorCallback = NULL;

//     /* Set the dma abort callback */
//     hdcmi->DMA_Handle->XferAbortCallback = NULL;

//     /* Reset transfer counters value */
//     hdcmi->XferCount = 0;
//     hdcmi->XferTransferNumber = 0;
//     if (Length <= CAM_PACKAGE_MAX_SIZE / 4)
//     {
//         /* Enable the DMA Stream */
//         if (HAL_DMA_Start_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, (uint32_t)pData, Length) != HAL_OK)
//         {
//             /* Set Error Code */
//             hdcmi->ErrorCode = HAL_DCMI_ERROR_DMA;
//             /* Change DCMI state */
//             hdcmi->State = HAL_DCMI_STATE_READY;
//             /* Release Lock */
//             __HAL_UNLOCK(hdcmi);
//             /* Return function status */
//             return HAL_ERROR;
//         }
//     }
//     else /* DCMI_DOUBLE_BUFFER Mode */
//     {
//         /* Set the DMA memory1 conversion complete callback */
//         hdcmi->DMA_Handle->XferM1CpltCallback = USER_DCMI_DMAXferCplt;

//         /* Initialize transfer parameters */
//         hdcmi->XferCount = 1;
//         hdcmi->XferSize = Length;
//         hdcmi->pBuffPtr = pData;

//         /* Get the number of buffer */
//         while (hdcmi->XferSize > CAM_PACKAGE_MAX_SIZE / 4)
//         {
//             hdcmi->XferSize = (hdcmi->XferSize / 2U);
//             hdcmi->XferCount = hdcmi->XferCount * 2U;
//         }
//         /* Update DCMI counter  and transfer number*/
//         hdcmi->XferCount = (hdcmi->XferCount - 2U);
//         hdcmi->XferTransferNumber = hdcmi->XferCount;
//         /* Update second memory address */
//         SecondMemAddress = (uint32_t)(pData + (4U * hdcmi->XferSize));

//         /* Start DMA multi buffer transfer */
//         if (HAL_DMAEx_MultiBufferStart_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, (uint32_t)pData, SecondMemAddress, hdcmi->XferSize) != HAL_OK)
//         {
//             /* Set Error Code */
//             hdcmi->ErrorCode = HAL_DCMI_ERROR_DMA;
//             /* Change DCMI state */
//             hdcmi->State = HAL_DCMI_STATE_READY;
//             /* Release Lock */
//             __HAL_UNLOCK(hdcmi);
//             /* Return function status */
//             return HAL_ERROR;
//         }
//     }

//     /* Enable Capture */
//     hdcmi->Instance->CR |= DCMI_CR_CAPTURE;

//     /* Release Lock */
//     __HAL_UNLOCK(hdcmi);

//     /* Return function status */
//     return HAL_OK;
// }
