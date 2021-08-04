#include "lcd.h"
#include "lcd_init.h"
#include "lcdfont.h"
#include "openvio.h"
#include "camera.h"
/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
								color       要填充的颜色
      返回值：  无
******************************************************************************/
DMA_BUFFER uint8_t lcd_buffer[240 * 240 * 2];
//DMA_BUFFER uint8_t lcd_buffer[2];
uint8_t *lcd_buffer8, *lcd_buffer16;
volatile uint32_t lcd_show_size = 0, send_count = 0;

extern const int resolution[][2];
extern uint8_t is_spi_wait, fps_count;
void LCD_Show_Cam(uint8_t *img, uint32_t size)
{
	static uint32_t pix_count = 0, recv_size_count = 0;
	uint32_t temp = 0;
	uint8_t step = 0;
	uint16_t img_x = resolution[vio_status.cam_frame_size_num][0];
	uint16_t img_y = resolution[vio_status.cam_frame_size_num][1];
	uint16_t rgb;

	if (is_spi_wait == 0)
	{

		lcd_buffer16 = lcd_buffer;

		step = img_x / LCD_W;
		if ((img_x % LCD_W) > 0)
		{
			step++;
		}

		do
		{
			temp = (pix_count / (img_x / step) * step * img_x + pix_count % (img_x / step) * step);
			if (temp < recv_size_count + size / vio_status.gs_bpp)
			{
				if (vio_status.cam_id == OV7725_ID)
				{
					lcd_buffer[pix_count * 2] = img[(temp - recv_size_count) * 2];
					lcd_buffer[pix_count * 2 + 1] = img[(temp - recv_size_count) * 2 + 1];
				}
				else if (vio_status.cam_id == MT9V034_ID)
				{
					rgb = ((img[temp - recv_size_count] >> 3) |
						   ((img[temp - recv_size_count] & ~3) << 3) |
						   ((img[temp - recv_size_count] & ~7) << 8));

					lcd_buffer[pix_count * 2 + 1] = (uint8_t)(rgb);
					lcd_buffer[pix_count * 2] = (uint8_t)(rgb >> 8);
				}
				pix_count++;
			}
		} while (temp < recv_size_count + size / vio_status.gs_bpp);

		recv_size_count += size / vio_status.gs_bpp;

		if (pix_count == img_x / step * img_y / step)
		{
			LCD_Address_Set(
				(LCD_W - img_x / step) / 2,
				(LCD_H - img_y / step) / 2,
				(LCD_W - img_x / step) / 2 + img_x / step - 1,
				(LCD_H - img_y / step) / 2 + img_y / step - 1); //设置显示范围

			lcd_buffer8 = lcd_buffer;
			lcd_show_size = img_x / step * img_y / step * 2;

			if (lcd_show_size > 65535)
			{
				send_count = 65535;
				LCD_Writ_Buffer(lcd_buffer8, 65535);
			}
			else
			{
				send_count = lcd_show_size;
				LCD_Writ_Buffer(lcd_buffer8, lcd_show_size);
			}

			pix_count = 0;
			recv_size_count = 0;
			is_spi_wait = 1;
		}
	}
}

void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
	static uint8_t cnnt = 0;
	uint16_t i;

	uint16_t *lcd_buffer16 = (uint16_t *)lcd_buffer;
	LCD_Address_Set(xsta, ysta, xend - 1, yend - 1);

	send_count = 65535;
	for (i = 0; i < 240 * 240; i++)
	{
		*(lcd_buffer16 + i) = ((color << 8) | (color >> 8));
	}
	lcd_buffer8 = lcd_buffer;
	lcd_show_size = 240 * 240 * 2;
	LCD_Writ_Buffer(lcd_buffer, 65535);

	cnnt++;
}
#define LCD_CS_Set() HAL_GPIO_WritePin(GPIOD, TFT_SPI_CS_Pin, GPIO_PIN_SET);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (send_count < lcd_show_size)
	{
		if ((send_count + 65535) < lcd_show_size)
		{
			LCD_Writ_Buffer(lcd_buffer8 + send_count, 65535);
			send_count += 65535;
		}
		else
		{

			LCD_Writ_Buffer(lcd_buffer8 + send_count, lcd_show_size - send_count);
			send_count = lcd_show_size;
		}
	}
	else
	{
		LCD_CS_Set();
		is_spi_wait = 0;
		fps_count++;
	}
}
/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
	LCD_Address_Set(x, y, x, y); //设置光标位置
	LCD_WR_DATA(color);
}

/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t t;
	int xerr = 0, yerr = 0, delta_x, delta_y, distance;
	int incx, incy, uRow, uCol;
	delta_x = x2 - x1; //计算坐标增量
	delta_y = y2 - y1;
	uRow = x1; //画线起点坐标
	uCol = y1;
	if (delta_x > 0)
		incx = 1; //设置单步方向
	else if (delta_x == 0)
		incx = 0; //垂直线
	else
	{
		incx = -1;
		delta_x = -delta_x;
	}
	if (delta_y > 0)
		incy = 1;
	else if (delta_y == 0)
		incy = 0; //水平线
	else
	{
		incy = -1;
		delta_y = -delta_x;
	}
	if (delta_x > delta_y)
		distance = delta_x; //选取基本增量坐标轴
	else
		distance = delta_y;
	for (t = 0; t < distance + 1; t++)
	{
		LCD_DrawPoint(uRow, uCol, color); //画点
		xerr += delta_x;
		yerr += delta_y;
		if (xerr > distance)
		{
			xerr -= distance;
			uRow += incx;
		}
		if (yerr > distance)
		{
			yerr -= distance;
			uCol += incy;
		}
	}
}

/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	LCD_DrawLine(x1, y1, x2, y1, color);
	LCD_DrawLine(x1, y1, x1, y2, color);
	LCD_DrawLine(x1, y2, x2, y2, color);
	LCD_DrawLine(x2, y1, x2, y2, color);
}

/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
      返回值：  无
******************************************************************************/
void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
	int a, b;
	a = 0;
	b = r;
	while (a <= b)
	{
		LCD_DrawPoint(x0 - b, y0 - a, color); //3
		LCD_DrawPoint(x0 + b, y0 - a, color); //0
		LCD_DrawPoint(x0 - a, y0 + b, color); //1
		LCD_DrawPoint(x0 - a, y0 - b, color); //2
		LCD_DrawPoint(x0 + b, y0 + a, color); //4
		LCD_DrawPoint(x0 + a, y0 - b, color); //5
		LCD_DrawPoint(x0 + a, y0 + b, color); //6
		LCD_DrawPoint(x0 - b, y0 + a, color); //7
		a++;
		if ((a * a + b * b) > (r * r)) //判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
      函数说明：显示汉字串
      入口数据：x,y显示坐标
                *s 要显示的汉字串
                fc 字的颜色
                bc 字的背景色
                sizey 字号 可选 16 24 32
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
	while (*s != 0)
	{
		if (sizey == 16)
			LCD_ShowChinese16x16(x, y, s, fc, bc, sizey, mode);
		else if (sizey == 24)
			LCD_ShowChinese24x24(x, y, s, fc, bc, sizey, mode);
		else if (sizey == 32)
			LCD_ShowChinese32x32(x, y, s, fc, bc, sizey, mode);
		else
			return;
		s += 2;
		x += sizey;
	}
}

/******************************************************************************
      函数说明：显示单个16x16汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese16x16(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
	//	uint8_t i,j;
	//	uint16_t k;
	//	uint16_t HZnum;//汉字数目
	//	uint16_t TypefaceNum;//一个字符所占字节大小
	//	uint16_t x0=x;
	//	TypefaceNum=sizey/8*sizey;//此算法只适用于字宽等于字高，且字高是8的倍数的字，
	//	                          //也建议用户使用这样大小的字,否则显示容易出问题！
	//	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//统计汉字数目
	//	for(k=0;k<HZnum;k++)
	//	{
	//		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
	//		{
	//			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
	//			for(i=0;i<TypefaceNum;i++)
	//			{
	//				for(j=0;j<8;j++)
	//				{
	//					if(!mode)//非叠加方式
	//					{
	//						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
	//						else LCD_WR_DATA(bc);
	//					}
	//					else//叠加方式
	//					{
	//						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
	//						x++;
	//						if((x-x0)==sizey)
	//						{
	//							x=x0;
	//							y++;
	//							break;
	//						}
	//					}
	//				}
	//			}
	//		}
	//		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	//	}
}

/******************************************************************************
      函数说明：显示单个24x24汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese24x24(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
	//	uint8_t i,j;
	//	uint16_t k;
	//	uint16_t HZnum;//汉字数目
	//	uint16_t TypefaceNum;//一个字符所占字节大小
	//	uint16_t x0=x;
	//	TypefaceNum=sizey/8*sizey;//此算法只适用于字宽等于字高，且字高是8的倍数的字，
	//	                          //也建议用户使用这样大小的字,否则显示容易出问题！
	//	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//统计汉字数目
	//	for(k=0;k<HZnum;k++)
	//	{
	//		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
	//		{
	//			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
	//			for(i=0;i<TypefaceNum;i++)
	//			{
	//				for(j=0;j<8;j++)
	//				{
	//					if(!mode)//非叠加方式
	//					{
	//						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
	//						else LCD_WR_DATA(bc);
	//					}
	//					else//叠加方式
	//					{
	//						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
	//						x++;
	//						if((x-x0)==sizey)
	//						{
	//							x=x0;
	//							y++;
	//							break;
	//						}
	//					}
	//				}
	//			}
	//		}
	//		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	//	}
}

/******************************************************************************
      函数说明：显示单个32x32汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese32x32(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
	//	uint8_t i,j;
	//	uint16_t k;
	//	uint16_t HZnum;//汉字数目
	//	uint16_t TypefaceNum;//一个字符所占字节大小
	//	uint16_t x0=x;
	//	TypefaceNum=sizey/8*sizey;//此算法只适用于字宽等于字高，且字高是8的倍数的字，
	//	                          //也建议用户使用这样大小的字,否则显示容易出问题！
	//	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//统计汉字数目
	//	for(k=0;k<HZnum;k++)
	//	{
	//		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
	//		{
	//			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
	//			for(i=0;i<TypefaceNum;i++)
	//			{
	//				for(j=0;j<8;j++)
	//				{
	//					if(!mode)//非叠加方式
	//					{
	//						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
	//						else LCD_WR_DATA(bc);
	//					}
	//					else//叠加方式
	//					{
	//						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
	//						x++;
	//						if((x-x0)==sizey)
	//						{
	//							x=x0;
	//							y++;
	//							break;
	//						}
	//					}
	//				}
	//			}
	//		}
	//		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	//	}
}

/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
	uint8_t temp, sizex, t;
	uint16_t i, TypefaceNum; //一个字符所占字节大小
	uint16_t x0 = x;
	sizex = sizey / 2;
	TypefaceNum = sizex / 8 * sizey;
	num = num - ' ';									 //得到偏移后的值
	LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1); //设置光标位置
	for (i = 0; i < TypefaceNum; i++)
	{
		if (sizey == 16)
			temp = ascii_1608[num][i]; //调用8x16字体
		else if (sizey == 32)
			temp = ascii_3216[num][i]; //调用16x32字体
		else
			return;
		for (t = 0; t < 8; t++)
		{
			if (!mode) //非叠加模式
			{
				if (temp & (0x01 << t))
					LCD_WR_DATA(fc);
				else
					LCD_WR_DATA(bc);
			}
			else //叠加模式
			{
				if (temp & (0x01 << t))
					LCD_DrawPoint(x, y, fc); //画一个点
				x++;
				if ((x - x0) == sizex)
				{
					x = x0;
					y++;
					break;
				}
			}
		}
	}
}

/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
	while (*p != '\0')
	{
		LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
		x += sizey / 2;
		p++;
	}
}

/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
uint32_t mypow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	while (n--)
		result *= m;
	return result;
}

/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{
	uint8_t t, temp;
	uint8_t enshow = 0;
	uint8_t sizex = sizey / 2;
	for (t = 0; t < len; t++)
	{
		temp = (num / mypow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
				continue;
			}
			else
				enshow = 1;
		}
		LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
	}
}

/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{
	uint8_t t, temp, sizex;
	uint16_t num1;
	sizex = sizey / 2;
	num1 = num * 100;
	for (t = 0; t < len; t++)
	{
		temp = (num1 / mypow(10, len - t - 1)) % 10;
		if (t == (len - 2))
		{
			LCD_ShowChar(x + (len - 2) * sizex, y, '.', fc, bc, sizey, 0);
			t++;
			len += 1;
		}
		LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
	}
}

/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组    
      返回值：  无
******************************************************************************/
void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[])
{
	uint16_t i, j, k = 0;
	LCD_Address_Set(x, y, x + length - 1, y + width - 1);
	for (i = 0; i < length; i++)
	{
		for (j = 0; j < width; j++)
		{
			LCD_WR_DATA8(pic[k * 2]);
			LCD_WR_DATA8(pic[k * 2 + 1]);
			k++;
		}
	}
}
