#include "openvio.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"

#include "mt9v034.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

USBD_StatusTypeDef MUSBD_Get_USB_Status(HAL_StatusTypeDef hal_status);

extern USBD_HandleTypeDef hUsbDeviceHS;

// typedef enum {
//     FRAMESIZE_INVALID = 0,
//     // C/SIF Resolutions
//     FRAMESIZE_QQCIF,    // 88x72
//     FRAMESIZE_QCIF,     // 176x144
//     FRAMESIZE_CIF,      // 352x288
//     FRAMESIZE_QQSIF,    // 88x60
//     FRAMESIZE_QSIF,     // 176x120
//     FRAMESIZE_SIF,      // 352x240
//     // VGA Resolutions
//     FRAMESIZE_QQQQVGA,  // 40x30
//     FRAMESIZE_QQQVGA,   // 80x60
//     FRAMESIZE_QQVGA,    // 160x120
//     FRAMESIZE_QVGA,     // 320x240
//     FRAMESIZE_VGA,      // 640x480
//     FRAMESIZE_HQQQVGA,  // 60x40
//     FRAMESIZE_HQQVGA,   // 120x80
//     FRAMESIZE_HQVGA,    // 240x160
//     // FFT Resolutions
//     FRAMESIZE_64X32,    // 64x32
//     FRAMESIZE_64X64,    // 64x64
//     FRAMESIZE_128X64,   // 128x64
//     FRAMESIZE_128X128,  // 128x128
//     // Other
//     FRAMESIZE_LCD,      // 128x160
//     FRAMESIZE_QQVGA2,   // 128x160
//     FRAMESIZE_WVGA,     // 720x480
//     FRAMESIZE_WVGA2,    // 752x480
//     FRAMESIZE_SVGA,     // 800x600
//     FRAMESIZE_XGA,      // 1024x768
//     FRAMESIZE_SXGA,     // 1280x1024
//     FRAMESIZE_UXGA,     // 1600x1200
// } framesize_t;

void openvio_status_init(struct OPENVIO_STATUS *status)
{
    status->cam_status = SENSOR_STATUS_WAIT;
    status->imu_status = SENSOR_STATUS_WAIT;
    status->usb_lock_status = 0;
    status->is_imu_send = 0;
    status->usb_status = USB_DISCONNECT;

	
    status->pixformat = PIXFORMAT_RGB565;
    //status->pixformat = PIXFORMAT_GRAYSCALE;

    status->cam_name = 0;
    status->cam_frame_size_num = FRAMESIZE_WVGA2;//FRAMESIZE_VGA; //FRAMESIZE_VGA;//FRAMESIZE_QVGA;//FRAMESIZE_WVGA2;//FRAMESIZE_HQVGA;//
}

void USER_PCD_IRQHandler(PCD_HandleTypeDef *hpcd)
{
    //  uint32_t send_size = USB_DMA_PACKAGE_SIZE;
    //	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
    //
    //	if(hcdc->TxState == USBD_OK)
    //	{
    //        if(vio_status.usb_s.status == USB_WORKING)
    //        {
    //            if(vio_status.usb_s.len == vio_status.usb_s.target_len)
    //            {
    //                vio_status.usb_s.status = USB_WAIT;
    //                return;
    //            }else if(vio_status.usb_s.len + USB_DMA_PACKAGE_SIZE > vio_status.usb_s.target_len)
    //            {
    //                send_size = vio_status.usb_s.target_len - vio_status.usb_s.len;
    //            }

    //            CDC_Transmit_HS(vio_status.usb_s.addr + vio_status.usb_s.len,send_size);
    //            vio_status.usb_s.len += send_size;
    //        }
    //
    //
    //	}
}

extern QueueHandle_t xQueue;

uint8_t openvio_usb_send(enum SENSOR_USB usb, uint8_t *Buf, uint32_t Len)
{
    struct USB_FRAME_STRUCT usb_frame_s;
    usb_frame_s.addr = (uint8_t *)Buf;
    usb_frame_s.len = Len;
    usb_frame_s.sensor = usb;

    xQueueSendFromISR(xQueue, (void *)&usb_frame_s, (TickType_t)0);
    return 1;
}

static uint8_t USBD_CAM_SetTxBuffer(USBD_HandleTypeDef *pdev,
                                    uint8_t *pbuff,
                                    uint32_t length)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

    hcdc->TxBuffer = pbuff;
    hcdc->TxLength = length;

    return USBD_OK;
}

static USBD_StatusTypeDef USBD_CAM_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;

    hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);

    usb_status = MUSBD_Get_USB_Status(hal_status);

    return usb_status;
}

static uint8_t USBD_CAM_TransmitPacket(USBD_HandleTypeDef *pdev)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

    if (pdev->pClassData != NULL)
    {
        if (hcdc->TxState == 0U)
        {
            /* Tx Transfer in progress */
            hcdc->TxState = 1U;

            /* Update the packet total length */
            pdev->ep_in[CDC_IN_EP & 0xFU].total_length = hcdc->TxLength;

            /* Transmit next packet */
            USBD_CAM_LL_Transmit(pdev, CDC_IN_EP, hcdc->TxBuffer,
                                 (uint32_t)hcdc->TxLength);

            return USBD_OK;
        }
        else
        {
            return USBD_BUSY;
        }
    }
    else
    {
        return USBD_FAIL;
    }
}

uint8_t CAM_Transmit_HS(uint8_t *Buf, uint32_t Len)
{
    uint8_t result = USBD_OK;
    /* USER CODE BEGIN 12 */
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    if (hcdc->TxState != 0)
    {
        return USBD_BUSY;
    }
    USBD_CAM_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
    result = USBD_CAM_TransmitPacket(&hUsbDeviceHS);
    /* USER CODE END 12 */
    return result;
}

uint8_t get_usb_tx_state(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    return hcdc->TxState;
}

USBD_StatusTypeDef MUSBD_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
  USBD_StatusTypeDef usb_status = USBD_OK;

  switch (hal_status)
  {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status;
}
