#ifndef __OV7725_H__
#define __OV7725_H__

#include "ov7725_regs.h"
#include <stdint.h>
#include "main.h"
#include "camera.h"

// Sensor PLL register value.
#define OMV_OV7725_PLL_CONFIG   (0x41)  // x4

// Sensor Banding Filter Value
#define OMV_OV7725_BANDING      (0x7F)

void ov7725_init(void);
void ov7725_config(framesize_t frame_size_num);

#endif
