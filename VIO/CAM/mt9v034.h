#ifndef MT9V34_H_
#define MT9V34_H_

#include <stdint.h>
#include "main.h"
#include "camera.h"

/* Constants */
#define TIMEOUT_MAX      				10000

#define BINNING_ROW_A					4
#define BINNING_COLUMN_A				4
#define BINNING_ROW_B					2
#define BINNING_COLUMN_B				2
#define MINIMUM_HORIZONTAL_BLANKING		91 // see datasheet
#define MAX_IMAGE_HEIGHT				482
#define MAX_IMAGE_WIDTH					752
#define MINIMUM_COLUMN_START			1
#define MINIMUM_ROW_START				4

/* Camera I2C registers */
#define mt9v034_DEVICE_WRITE_ADDRESS    0xB8
#define mt9v034_DEVICE_READ_ADDRESS     0xB9

/* Context A */
#define MTV_COLUMN_START_REG_A  		0x01
#define MTV_ROW_START_REG_A     		0x02
#define MTV_WINDOW_HEIGHT_REG_A 		0x03
#define MTV_WINDOW_WIDTH_REG_A  		0x04
#define MTV_HOR_BLANKING_REG_A  		0x05
#define MTV_VER_BLANKING_REG_A  		0x06
#define MTV_COARSE_SW_1_REG_A			0x08
#define MTV_COARSE_SW_2_REG_A			0x09
#define MTV_COARSE_SW_CTRL_REG_A		0x0A
#define MTV_COARSE_SW_TOTAL_REG_A 		0x0B
#define MTV_FINE_SW_1_REG_A				0xD3
#define MTV_FINE_SW_2_REG_A				0xD4
#define MTV_FINE_SW_TOTAL_REG_A			0xD5
#define MTV_READ_MODE_REG_A        		0x0D
#define MTV_V1_CTRL_REG_A				0x31
#define MTV_V2_CTRL_REG_A				0x32
#define MTV_V3_CTRL_REG_A				0x33
#define MTV_V4_CTRL_REG_A				0x34
#define MTV_ANALOG_GAIN_CTRL_REG_A		0x35

/* Context B */
#define MTV_COLUMN_START_REG_B  		0xC9
#define MTV_ROW_START_REG_B     		0xCA
#define MTV_WINDOW_HEIGHT_REG_B 		0xCB
#define MTV_WINDOW_WIDTH_REG_B  		0xCC
#define MTV_HOR_BLANKING_REG_B  		0xCD
#define MTV_VER_BLANKING_REG_B  		0xCE
#define MTV_COARSE_SW_1_REG_B			0xCF
#define MTV_COARSE_SW_2_REG_B			0xD0
#define MTV_COARSE_SW_CTRL_REG_B		0xD1
#define MTV_COARSE_SW_TOTAL_REG_B 		0xD2
#define MTV_FINE_SW_1_REG_B				0xD6
#define MTV_FINE_SW_2_REG_B				0xD7
#define MTV_FINE_SW_TOTAL_REG_B			0xD8
#define MTV_READ_MODE_REG_B        		0x0E
#define MTV_V1_CTRL_REG_B				0x39
#define MTV_V2_CTRL_REG_B				0x3A
#define MTV_V3_CTRL_REG_B				0x3B
#define MTV_V4_CTRL_REG_B				0x3C
#define MTV_ANALOG_GAIN_CTRL_REG_B		0x36

/* General Registers */
#define MTV_CHIP_VERSION_REG    		0x00
#define MTV_CHIP_CONTROL_REG    		0x07
#define MTV_SOFT_RESET_REG      		0x0C

#define MTV_HDR_ENABLE_REG				0x0F
#define MTV_ADC_RES_CTRL_REG			0x1C
#define MTV_ROW_NOISE_CORR_CTRL_REG		0x70
#define MTV_TEST_PATTERN_REG       		0x7F
#define MTV_TILED_DIGITAL_GAIN_REG		0x80
#define MTV_AGC_AEC_DESIRED_BIN_REG		0xA5
#define MTV_MAX_GAIN_REG        		0xAB
#define MTV_MIN_EXPOSURE_REG       		0xAC  // datasheet min coarse shutter width
#define MTV_MAX_EXPOSURE_REG       		0xAD  // datasheet max coarse shutter width
#define MTV_AEC_AGC_ENABLE_REG			0xAF
#define MTV_AGC_AEC_PIXEL_COUNT_REG		0xB0
#define MTV_AEC_UPDATE_REG				0xA6
#define MTV_AEC_LOWPASS_REG				0xA8
#define MTV_AGC_UPDATE_REG				0xA9
#define MTV_AGC_LOWPASS_REG				0xAA
#define MTV_DIGITAL_TEST_REG			0x7F

#define MT9V034_MAX_HEIGHT                      (480)
#define MT9V034_MAX_WIDTH                       (752)
#define MT9V034_CHIP_VERSION                    (0x00)
#define MT9V034_COL_START                       (0x01)
#define MT9V034_COL_START_MIN                   (1)
#define MT9V034_COL_START_MAX                   (752)
#define MT9V034_ROW_START                       (0x02)
#define MT9V034_ROW_START_MIN                   (4)
#define MT9V034_ROW_START_MAX                   (482)
#define MT9V034_WINDOW_HEIGHT                   (0x03)
#define MT9V034_WINDOW_HEIGHT_MIN               (1)
#define MT9V034_WINDOW_HEIGHT_MAX               (480)
#define MT9V034_WINDOW_WIDTH                    (0x04)
#define MT9V034_WINDOW_WIDTH_MIN                (1)
#define MT9V034_WINDOW_WIDTH_MAX                (752)
#define MT9V034_HORIZONTAL_BLANKING             (0x05)
#define MT9V034_HORIZONTAL_BLANKING_MIN         (43)
#define MT9V034_HORIZONTAL_BLANKING_MAX         (1023)
#define MT9V034_HORIZONTAL_BLANKING_DEF         (94)
#define MT9V034_VERTICAL_BLANKING               (0x06)
#define MT9V034_VERTICAL_BLANKING_MIN           (4)
#define MT9V034_VERTICAL_BLANKING_MAX           (3000)
#define MT9V034_VERTICAL_BLANKING_DEF           (45)
#define MT9V034_CHIP_CONTROL                    (0x07)
#define MT9V034_CHIP_CONTROL_MASTER_MODE        (1 << 3)
#define MT9V034_CHIP_CONTROL_SNAP_MODE          (3 << 3)
#define MT9V034_CHIP_CONTROL_MODE_MASK          (3 << 3)
#define MT9V034_CHIP_CONTROL_DOUT_ENABLE        (1 << 7)
#define MT9V034_CHIP_CONTROL_SEQUENTIAL         (1 << 8)
#define MT9V034_CHIP_CONTROL_RESERVED           (1 << 9)
#define MT9V034_SHUTTER_WIDTH1                  (0x08)
#define MT9V034_SHUTTER_WIDTH2                  (0x09)
#define MT9V034_SHUTTER_WIDTH_CONTROL           (0x0A)
#define MT9V034_TOTAL_SHUTTER_WIDTH             (0x0B)
#define MT9V034_TOTAL_SHUTTER_WIDTH_MIN         (1)
#define MT9V034_TOTAL_SHUTTER_WIDTH_MAX         (32767)
#define MT9V034_RESET                           (0x0C)
#define MT9V034_READ_MODE                       (0x0D)
#define MT9V034_READ_MODE_ROW_BIN_2             (1 << 0)
#define MT9V034_READ_MODE_ROW_BIN_4             (1 << 1)
#define MT9V034_READ_MODE_COL_BIN_2             (1 << 2)
#define MT9V034_READ_MODE_COL_BIN_4             (1 << 3)
#define MT9V034_READ_MODE_ROW_FLIP              (1 << 4)
#define MT9V034_READ_MODE_COL_FLIP              (1 << 5)
#define MT9V034_READ_MODE_DARK_COLS             (1 << 6)
#define MT9V034_READ_MODE_DARK_ROWS             (1 << 7)
#define MT9V034_PIXEL_OPERATION_MODE            (0x0F)
#define MT9V034_PIXEL_OPERATION_MODE_HDR        (1 << 0)
#define MT9V034_PIXEL_OPERATION_MODE_COLOR      (1 << 1)
#define MT9V034_ANALOG_GAIN                     (0x35)
#define MT9V034_ANALOG_GAIN_MIN                 (16)
#define MT9V034_ANALOG_GAIN_MAX                 (64)
#define MT9V034_MAX_ANALOG_GAIN                 (0x36)
#define MT9V034_MAX_ANALOG_GAIN_MAX             (127)
#define MT9V034_FRAME_DARK_AVERAGE              (0x42)
#define MT9V034_DARK_AVG_THRESH                 (0x46)
#define MT9V034_DARK_AVG_LOW_THRESH_MASK        (255 << 0)
#define MT9V034_DARK_AVG_LOW_THRESH_SHIFT       (0)
#define MT9V034_DARK_AVG_HIGH_THRESH_MASK       (255 << 8)
#define MT9V034_DARK_AVG_HIGH_THRESH_SHIFT      (8)
#define MT9V034_ROW_NOISE_CORR_CONTROL          (0x70)
#define MT9V034_ROW_NOISE_CORR_ENABLE           (1 << 5)
#define MT9V034_ROW_NOISE_CORR_USE_BLK_AVG      (1 << 7)
#define MT9V034_PIXEL_CLOCK                     (0x72)
#define MT9V034_PIXEL_CLOCK_INV_LINE            (1 << 0)
#define MT9V034_PIXEL_CLOCK_INV_FRAME           (1 << 1)
#define MT9V034_PIXEL_CLOCK_XOR_LINE            (1 << 2)
#define MT9V034_PIXEL_CLOCK_CONT_LINE           (1 << 3)
#define MT9V034_PIXEL_CLOCK_INV_PXL_CLK         (1 << 4)
#define MT9V034_TEST_PATTERN                    (0x7F)
#define MT9V034_TEST_PATTERN_DATA_MASK          (1023 << 0)
#define MT9V034_TEST_PATTERN_DATA_SHIFT         (0)
#define MT9V034_TEST_PATTERN_USE_DATA           (1 << 10)
#define MT9V034_TEST_PATTERN_GRAY_MASK          (3 << 11)
#define MT9V034_TEST_PATTERN_GRAY_NONE          (0 << 11)
#define MT9V034_TEST_PATTERN_GRAY_VERTICAL      (1 << 11)
#define MT9V034_TEST_PATTERN_GRAY_HORIZONTAL    (2 << 11)
#define MT9V034_TEST_PATTERN_GRAY_DIAGONAL      (3 << 11)
#define MT9V034_TEST_PATTERN_ENABLE             (1 << 13)
#define MT9V034_TEST_PATTERN_FLIP               (1 << 14)
#define MT9V034_AEC_AGC_ENABLE                  (0xAF)
#define MT9V034_AEC_ENABLE                      (1 << 0)
#define MT9V034_AGC_ENABLE                      (1 << 1)
#define MT9V034_THERMAL_INFO                    (0xC1)
#define MT9V034_ID_REG                          (0x6B)
#define MT9V034_MAX_GAIN                        (0xAB)
#define MT9V034_MAX_EXPOSE                      (0xAD)
#define MT9V034_PIXEL_COUNT                     (0xB0)
#define MT9V034_FINE_SHUTTER_WIDTH_TOTAL        (0xD5)

/*
 * Resolution:
 * ROW_SIZE * BINNING_ROW <= MAX_IMAGE_WIDTH
 * COLUMN_SIZE * BINNING_COLUMN <= MAX_IMAGE_HEIGHT
 */

#define FULL_IMAGE_SIZE (FULL_IMAGE_ROW_SIZE*FULL_IMAGE_COLUMN_SIZE)

#define FULL_IMAGE_ROW_SIZE 		      (752)
#define FULL_IMAGE_COLUMN_SIZE 	    	(480)

/* Functions */

/**
  * @brief  Configures the mt9v034 camera with two context (binning 4 and binning 2).
  */
void mt9v034_context_configuration(void);

/**
  * @brief  Changes sensor context based on settings
  */
void mt9v034_set_context(void);

int reset_mt9v034_by_hardware(void);
/*
uint8_t mt9v034_ReadReg(uint16_t Addr);
uint8_t mt9v034_WriteReg(uint16_t Addr, uint8_t Data);
*/
int mt9v034_init(void);
void mt9v034_config(framesize_t frame_size_num);
void mt9v034_exposure(int exposure);

#endif /* MT9V34_H_ */
