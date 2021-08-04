#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "main.h"

#define OV2640_SLV_ADDR     (0x60)
#define OV5640_SLV_ADDR     (0x78)
#define OV7725_SLV_ADDR     (0x42)
#define MT9V034_SLV_ADDR    (0xB8)
#define LEPTON_SLV_ADDR     (0x54)
#define HM01B0_SLV_ADDR     (0x48)

// Chip ID Registers
#define OV5640_CHIP_ID      (0x300A)
#define OV_CHIP_ID          (0x0A)
#define ON_CHIP_ID          (0x00)
#define HIMAX_CHIP_ID       (0x0001)

// Chip ID Values
#define OV2640_ID           (0x26)
#define OV5640_ID           (0x56)
#define OV7690_ID           (0x76)
#define OV7725_ID           (0x77)
#define OV9650_ID           (0x96)
#define MT9V034_ID          (0x13)
#define LEPTON_ID           (0x54)
#define HM01B0_ID           (0xB0)

typedef enum {
    FRAMESIZE_INVALID = 0,
    // C/SIF Resolutions
    FRAMESIZE_QQCIF,    // 88x72
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_CIF,      // 352x288
    FRAMESIZE_QQSIF,    // 88x60
    FRAMESIZE_QSIF,     // 176x120
    FRAMESIZE_SIF,      // 352x240
    // VGA Resolutions
    FRAMESIZE_QQQQVGA,  // 40x30
    FRAMESIZE_QQQVGA,   // 80x60
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_HQQQVGA,  // 60x40
    FRAMESIZE_HQQVGA,   // 120x80
    FRAMESIZE_HQVGA,    // 240x160
    // FFT Resolutions
    FRAMESIZE_64X32,    // 64x32
    FRAMESIZE_64X64,    // 64x64
    FRAMESIZE_128X64,   // 128x64
    FRAMESIZE_128X128,  // 128x128
    // Other
    FRAMESIZE_LCD,      // 128x160
    FRAMESIZE_QQVGA2,   // 128x160
    FRAMESIZE_WVGA,     // 720x480
    FRAMESIZE_WVGA2,    // 752x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200

    FRAMESIZE_MLCD,     // 240x240
} framesize_t;

extern const int resolution[][2];

void camera_init(void);
void dcmi_dma_start(void);

#endif
