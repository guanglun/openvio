#include <string.h>
#include "ov7725.h"
#include "main.h"
#include "cambus.h"
#include "openvio.h"
#include "cmsis_os.h"

#include "dcmi.h"

extern DCMI_HandleTypeDef hdcmi;

static const uint8_t default_regs[][2] = {

// From App Note.

    {COM12,         0x03},
    {HSTART,        0x22},
    {HSIZE,         0xa4},
    {VSTART,        0x07},
    {VSIZE,         0xf0},
    {HREF,          0x00},
    {HOUTSIZE,      0xa0},
    {VOUTSIZE,      0xf0},
    {EXHCH,         0x00},
    {CLKRC,         0xC0}, // {CLKRC, 0x01},

    {TGT_B,         0x7f},
    {FIXGAIN,       0x09},
    {AWB_CTRL0,     0xe0},
    {DSP_CTRL1,     0xff},
    {DSP_CTRL2,     0x20 | DSP_CTRL2_VDCW_EN | DSP_CTRL2_HDCW_EN | DSP_CTRL2_VZOOM_EN | DSP_CTRL2_HZOOM_EN}, // {DSP_CTRL2, 0x20},
    {DSP_CTRL3,     0x00},
    {DSP_CTRL4,     0x48},

    {COM8,          0xf0},
    {COM4,          OMV_OV7725_PLL_CONFIG}, // {COM4, 0x41},
    {COM6,          0xc5},
    {COM9,          0x11},
    {BDBASE,        0x7f},
    {BDSTEP,        0x03},
    {AEW,           0x40},
    {AEB,           0x30},
    {VPT,           0xa1},
    {EXHCL,         0x00},
    {AWB_CTRL3,     0xaa},
    {COM8,          0xff},

    {EDGE1,         0x05},
    {DNSOFF,        0x01},
    {EDGE2,         0x03},
    {EDGE3,         0x00},
    {MTX1,          0xb0},
    {MTX2,          0x9d},
    {MTX3,          0x13},
    {MTX4,          0x16},
    {MTX5,          0x7b},
    {MTX6,          0x91},
    {MTX_CTRL,      0x1e},
    {BRIGHTNESS,    0x08},
    {CONTRAST,      0x20},
    {UVADJ0,        0x81},
    {SDE,           SDE_CONT_BRIGHT_EN | SDE_SATURATION_EN},

    {GAM1,          0x0c},
    {GAM2,          0x16},
    {GAM3,          0x2a},
    {GAM4,          0x4e},
    {GAM5,          0x61},
    {GAM6,          0x6f},
    {GAM7,          0x7b},
    {GAM8,          0x86},
    {GAM9,          0x8e},
    {GAM10,         0x97},
    {GAM11,         0xa4},
    {GAM12,         0xaf},
    {GAM13,         0xc5},
    {GAM14,         0xd7},
    {GAM15,         0xe8},
    {SLOP,          0x20},

    {DM_LNL,        0x00},
    {BDBASE,        OMV_OV7725_BANDING}, // {BDBASE, 0x7f}
    {BDSTEP,        0x03},

    {LC_RADI,       0x10},
    {LC_COEF,       0x10},
    {LC_COEFB,      0x14},
    {LC_COEFR,      0x17},
    {LC_CTR,        0x01}, // {LC_CTR, 0x05},

    {COM5,          0xf5}, // {COM5, 0x65},

// OpenMV Custom.

    {COM7,          COM7_FMT_RGB565},

// End.

    {0x00,          0x00},
	
	
	
	
};

static int ov7725_reset(void)
{
   // Reset all registers
   int ret = cambus_writeb(OV7725_SLV_ADDR, COM7, COM7_RESET);

   // Delay 2 ms
   osDelay(2);

   // Write default regsiters
   for (int i = 0; default_regs[i][0]; i++) {
       ret |= cambus_writeb(OV7725_SLV_ADDR, default_regs[i][0], default_regs[i][1]);
   }

   // Delay 300 ms
   osDelay(300);

   return ret;
}

static int ov7725_set_framesize(framesize_t framesize)
{
    int ret=0;
    uint16_t w = resolution[framesize][0];
    uint16_t h = resolution[framesize][1];

    // Write MSBs
    ret |= cambus_writeb(OV7725_SLV_ADDR, HOUTSIZE, w>>2);
    ret |= cambus_writeb(OV7725_SLV_ADDR, VOUTSIZE, h>>1);

    // Write LSBs
    ret |= cambus_writeb(OV7725_SLV_ADDR, EXHCH, ((w&0x3) | ((h&0x1) << 2)));

    if ((w <= 320) && (h <= 240)) {
        // Set QVGA Resolution
        uint8_t reg;
        int ret = cambus_readb(OV7725_SLV_ADDR, COM7, &reg);
        reg = COM7_SET_RES(reg, COM7_RES_QVGA);
        ret |= cambus_writeb(OV7725_SLV_ADDR, COM7, reg);

        // Set QVGA Window Size
        ret |= cambus_writeb(OV7725_SLV_ADDR, HSTART, 0x3F);
        ret |= cambus_writeb(OV7725_SLV_ADDR, HSIZE,  0x50);
        ret |= cambus_writeb(OV7725_SLV_ADDR, VSTART, 0x03);
        ret |= cambus_writeb(OV7725_SLV_ADDR, VSIZE,  0x78);

        // Enable auto-scaling/zooming factors
        ret |= cambus_writeb(OV7725_SLV_ADDR, DSPAUTO, 0xFF);
    } else {
        // Set VGA Resolution
        uint8_t reg;
        int ret = cambus_readb(OV7725_SLV_ADDR, COM7, &reg);
        reg = COM7_SET_RES(reg, COM7_RES_VGA);
        ret |= cambus_writeb(OV7725_SLV_ADDR, COM7, reg);

        // Set VGA Window Size
        ret |= cambus_writeb(OV7725_SLV_ADDR, HSTART, 0x23);
        ret |= cambus_writeb(OV7725_SLV_ADDR, HSIZE,  0xA0);
        ret |= cambus_writeb(OV7725_SLV_ADDR, VSTART, 0x07);
        ret |= cambus_writeb(OV7725_SLV_ADDR, VSIZE,  0xF0);

        // Disable auto-scaling/zooming factors
        ret |= cambus_writeb(OV7725_SLV_ADDR, DSPAUTO, 0xF3);

        // Clear auto-scaling/zooming factors
        ret |= cambus_writeb(OV7725_SLV_ADDR, SCAL0, 0x00);
        ret |= cambus_writeb(OV7725_SLV_ADDR, SCAL1, 0x40);
        ret |= cambus_writeb(OV7725_SLV_ADDR, SCAL2, 0x40);
    }

    return ret;
}

static int set_pixformat(pixformat_t pixformat)
{
    uint8_t reg;
    int ret = cambus_readb(OV7725_SLV_ADDR, COM7, &reg);

    switch (pixformat) {
        case PIXFORMAT_RGB565:
            reg = COM7_SET_FMT(reg, COM7_FMT_RGB);
            ret |= cambus_writeb(OV7725_SLV_ADDR, DSP_CTRL4, DSP_CTRL4_YUV_RGB);
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
            reg = COM7_SET_FMT(reg, COM7_FMT_YUV);
            ret |= cambus_writeb(OV7725_SLV_ADDR, DSP_CTRL4, DSP_CTRL4_YUV_RGB);
            break;
        case PIXFORMAT_BAYER:
            reg = COM7_SET_FMT(reg, COM7_FMT_P_BAYER);
            ret |= cambus_writeb(OV7725_SLV_ADDR, DSP_CTRL4, DSP_CTRL4_RAW8);
            break;
        default:
            return -1;
    }

    // Write back register
    return cambus_writeb(OV7725_SLV_ADDR, COM7, reg) | ret;
}

void ov7725_config(framesize_t frame_size_num)
{
	vio_status.cam_frame_size_num = frame_size_num;
	vio_status.cam_frame_size = resolution[frame_size_num][0]*resolution[frame_size_num][1];
	ov7725_set_framesize(frame_size_num);
}

void ov7725_dcmi_init(void)
{

  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }

}

void ov7725_init(void)
{
    ov7725_dcmi_init();
    ov7725_reset();
    ov7725_config(vio_status.cam_frame_size_num);
    set_pixformat(vio_status.pixformat);
    
}
