#include "sd_card.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "openvio.h"
#include "lcd.h"

#include "ff.h"

FRESULT fr;
DMA_BUFFER FATFS fs;
DMA_BUFFER FIL fd;

void sdcard_init(void)
{
  char filename[] = "test.txt";
  uint8_t write_dat[] = "hello";

  uint16_t write_num = 0;

  fr = f_mount(&fs, "0:/", 1);
  if (fr == FR_OK)
  {
    //LCD_ShowString(0, 16 * 2, "[SD] [Mount Success]", RED, WHITE, 16, 0);
    printf("SD card mount ok!\r\n");
  }
  else
  {
    //LCD_ShowString(0, 16 * 2, "[SD] [Mount Fail]", RED, WHITE, 16, 0);
    printf("[SD] [MOUNT ERROR %d]", fr);
    return;
  }

  fr = f_open(&fd, filename, FA_CREATE_ALWAYS | FA_WRITE);
  if (fr == FR_OK)
  {
    printf("open file \"%s\" ok! \r\n", filename);
  }
  else
  {
    printf("open file \"%s\" error : %d\r\n", filename, fr);
    return;
  }

  fr = f_write(&fd, write_dat, sizeof(write_dat), (void *)&write_num);
  if (fr == FR_OK)
  {
    printf("write %d dat to file \"%s\" ok,dat is \"%s\".\r\n", write_num, filename, write_dat);
  }
  else
  {
    printf("write dat to file \"%s\" error,error code is:%d\r\n", filename, fr);
    return;
  }

  fr = f_close(&fd);
  if (fr == FR_OK)
  {
    printf("close file \"%s\" ok!\r\n", filename);
  }
  else
  {
    printf("close file \"%s\" error, error code is:%d.\r\n", filename, fr);
    return;
  }
}
