/**
  ******************************************************************************
  * @file           :lcd_3224.c
  * @brief          :
	*	3.2 inch tft_lcd driver
  *	DATA_PIN:	GPIOE				CS/RS/WR/RD PIN:PC6/PD13/PD14/PD15
  ******************************************************************************
  */
#include <stdlib.h>
#include "lcd_3224.h"
#include "stm32f4xx_hal.h"

//3.2寸液晶控制引脚设置
#define LCD_CS_L()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
#define LCD_CS_H()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
#define LCD_RS_L() 	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
#define LCD_RS_H() 	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
#define LCD_WR_L()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
#define LCD_WR_H()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
#define LCD_RD_L()	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
#define LCD_RD_H()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

//16bit 颜色值
#define WHITE        0xFFFF
#define BLACK        0x0000	  
#define BLUE         0x001F  
#define RED          0xF800
#define MAGENTA      0xF81F
#define GREEN        0x07E0
#define CYAN         0x7FFF
#define YELLOW       0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色
#define LGRAY 			 0XC618 //浅灰色

#define POINT_COLOR RED
#define BACK_COLOR	0xFFFF

uint16_t DeviceCode; //LCD的ID号变量

/**
*	@功能：写入LCD控制命令
*	@作者
*	@输入
*	@输出
*/
void LCD_WR_REG(uint16_t LCD_Reg){
	
	LCD_RD_H(); //读失能
	LCD_RS_L(); //写入的是命令
	LCD_WR_L(); //拉低写入的引脚，准备写入
	LCD_CS_L(); //拉低片选段CS
	GPIOE->ODR = LCD_Reg;
	//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_All,LCD_Reg);//写入命令
	LCD_CS_H(); //拉高片选段CS
	LCD_WR_H(); //拉高写入的引脚 ，写入命令
}

/**
*	@功能：写入LCD数据
*	@作者
*	@输入
*	@输出
*/
void LCD_WR_DATA(uint16_t LCD_Data){
	
	LCD_RD_H(); //读失能
	LCD_RS_H(); //写入的是数据
	LCD_WR_L(); //拉低写入的引脚，准备写入
	LCD_CS_L(); //拉低片选段CS
	GPIOE->ODR = LCD_Data;
	LCD_CS_H(); //拉高片选段CS
	LCD_WR_H(); //拉高写入的引脚，写入数据	
} 

/**
*	@功能：读取LCD数据
*	@作者
*	@输入：要读取的寄存器
*	@输出：读取到的数据
*/
uint16_t LCD_ReadReg(uint16_t LCD_Reg){
	
	uint16_t temp;
	
	GPIO_InitTypeDef GPIO_InitStruct;
	LCD_WR_REG(LCD_Reg);
	/* 设置为输入模式 */
	GPIO_InitStruct.Pin = GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);	 
	
	LCD_CS_L(); //拉低片选段CS					
	LCD_RS_H(); //读取的是数据
	LCD_RD_L(); //拉低读取的引脚，准备读取数据
	LCD_RD_H(); //拉高读取的引脚，读取数据	
	temp=(uint16_t)GPIOE->IDR;
	LCD_CS_H(); //拉高片选段CS

	/* 设置为输出模式 */
	GPIO_InitStruct.Pin = GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);	

	return temp;				    	 	
}

/**
*	@功能：写入寄存器数据
*	@作者
*	@输入：要写入的寄存器，要写入的值
*	@输出：
*/
void LCD_WriteReg(uint16_t LCD_Reg ,uint16_t LCD_RegValue){
	
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA(LCD_RegValue);										    
}

/**
*	@功能：写入GRAM数据
*	@作者
*	@输入：
*	@输出：
*/
void LCD_WriteRAM_Prepare(void){
	
	LCD_WR_REG(0x22);
}

/**
*	@功能：写入GRAM 16bit颜色值
*	@作者
*	@输入：
*	@输出：
*/
void LCD_WriteRAM(uint16_t  RGB_Code){
	
	LCD_WR_DATA(RGB_Code);    
}

/**
*	@功能：设置光标位置
*	@作者
*	@输入：X坐标值，Y坐标值
*	@输出：
*/
void LCD_SetCursor(uint16_t  Xpos, uint16_t  Ypos){
	
 	if(DeviceCode==0x8999||DeviceCode==0x9919){
		LCD_WriteReg(0x004E, Xpos);
		LCD_WriteReg(0X004F, Ypos);
	}else{
		LCD_WriteReg(0x0020, Xpos);
		LCD_WriteReg(0X0021, Ypos);
	}
}

/**
*	@功能：清屏操作，即刷屏
*	@作者
*	@输入：
*	@输出：
*/
void LCD_Clear(uint16_t Color){
	
	uint32_t index=0;      
	LCD_SetCursor(0x00,0x0000);      //设置光标位置 
	LCD_WriteRAM_Prepare();          //开始写入GRAM
				 
	for(index=0;index<76800;index++){
		LCD_WR_DATA(Color);		   
	}	
}

/**
*	@功能：指定位置画点
*	@作者
*	@输入：X坐标值，Y坐标值
*	@输出：
*/
void LCD_DrawPoint(uint16_t xsta, uint16_t ysta,uint16_t pixel){
	
	LCD_SetCursor(xsta,ysta);  //设置光标位置 
	LCD_WR_REG(0x22);           //开始写入GRAM
	LCD_WR_DATA(pixel); 
}


/**
*	@功能：指定位置绘制窗口
*	@作者
*	@输入：X始末坐标值，Y始末坐标值
*	@输出：
*/
//void LCD_WindowMax (unsigned int x,unsigned int y,unsigned int x_end,unsigned int y_end){

//	if(DeviceCode==0x8999){
//		LCD_WriteReg(0x44,x|((x_end-1)<<8));
//		LCD_WriteReg(0x45,y);
//		LCD_WriteReg(0x46,y_end-1);
//	}else{
//		LCD_WriteReg(0x50, x);                      // Horizontal GRAM Start Address     
//		LCD_WriteReg(0x51, x_end-1);               	// Horizontal GRAM End   Address (-1) 
//		LCD_WriteReg(0x52, y);                      // Vertical   GRAM Start Address     
//		LCD_WriteReg(0x53, y_end-1);                // Vertical   GRAM End   Address (-1) 
//	}
//}

/**
*	@功能：指定区域填充颜色
*	@作者
*	@输入：X始末坐标值，Y始末坐标值，颜色值
*	@输出：
*/
//void LCD_Fill(uint8_t xsta, uint16_t ysta, uint8_t xend, uint16_t yend, uint16_t colour){
//	
//  uint32_t  n;

//	//设置窗口	
//	LCD_WindowMax (xsta, ysta, xend, yend); 
//	LCD_SetCursor(xsta,ysta);        //设置光标位置 
//	LCD_WriteRAM_Prepare();         //开始写入GRAM	 	   	   
//	n=(uint32_t)(yend-ysta+1)*(xend-xsta+1);    
//	while(n--){LCD_WR_DATA(colour);} //显示所填充的颜
//	 
//	/*恢复窗口*/
//	LCD_WindowMax (0, 0, 240, 320); 
//}  

/**
*	@功能：绘制线条
*	@作者
*	@输入：X始末坐标值，Y始末坐标值
*	@输出：
*/
//void LCD_DrawLine(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend){
//	
//  uint16_t  x, y, t;
//	if((xsta==xend)&&(ysta==yend))LCD_DrawPoint(xsta, ysta);
//	else if(abs(yend-ysta)>abs(xend-xsta)){//斜率大于1
//		if(ysta>yend){
//			t=ysta;
//			ysta=yend;
//			yend=t; 
//			t=xsta;
//			xsta=xend;
//			xend=t; 
//		}
//		for(y=ysta;y<yend;y++){            //以y轴为基准
//			x=(uint32_t)(y-ysta)*(xend-xsta)/(yend-ysta)+xsta;
//			LCD_DrawPoint(x, y);  
//		}
//	}else{     //斜率小于等于1
//		if(xsta>xend){
//			t=ysta;
//			ysta=yend;
//			yend=t;
//			t=xsta;
//			xsta=xend;
//			xend=t;
//		}   
//		for(x=xsta;x<=xend;x++){  //以x轴为基准
//			y =(uint32_t)(x-xsta)*(yend-ysta)/(xend-xsta)+ysta;
//			LCD_DrawPoint(x,y); 
//		}
//	} 
//} 

/**
*	@功能：指定区域绘制制定大小的圆
*	@作者
*	@输入：X始末坐标值，Y始末坐标值，圆半径
*	@输出：
*/
//void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r){
//	
//	int a,b;
//	int di;
//	a=0;b=r;	  
//	di=3-(r<<1);             //判断下个点位置的标志
//	while(a<=b){
//		LCD_DrawPoint(x0-b,y0-a);             //3           
//		LCD_DrawPoint(x0+b,y0-a);             //0           
//		LCD_DrawPoint(x0-a,y0+b);             //1       
//		LCD_DrawPoint(x0-b,y0-a);             //7           
//		LCD_DrawPoint(x0-a,y0-b);             //2             
//		LCD_DrawPoint(x0+b,y0+a);             //4               
//		LCD_DrawPoint(x0+a,y0-b);             //5
//		LCD_DrawPoint(x0+a,y0+b);             //6 
//		LCD_DrawPoint(x0-b,y0+a);             
//		a++;

//		//使用Bresenham算法画圆     
//		if(di<0)di +=4*a+6;	  
//		else{
//			di+=10+4*(a-b);   
//			b--;
//		} 
//		LCD_DrawPoint(x0+a,y0+b);
//	}
//} 

/**
*	@功能：指定区域绘制矩形
*	@作者
*	@输入：X始末坐标值，Y始末坐标值
*	@输出：
*/
//void LCD_DrawRectangle(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend){
//	
//	LCD_DrawLine(xsta,ysta,xend,ysta);
//	LCD_DrawLine(xsta,ysta,xsta,yend);
//	LCD_DrawLine(xsta,yend,xend,yend);
//	LCD_DrawLine(xend,ysta,xend,yend);
//} 

/**
*	@功能：指定区域写入文字
*	@作者
*	@输入：X坐标，Y坐标，字体大小，前景色，背景色
*	@输出：
*/
//void LCD_ShowChar(uint8_t x, uint16_t  y, uint8_t num, uint8_t size, uint16_t  PenColor, uint16_t  BackColor){       
//	#define MAX_CHAR_POSX 232
//	#define MAX_CHAR_POSY 304 
//  uint8_t temp;
//  uint8_t pos,t;
//	if(x>MAX_CHAR_POSX||y>MAX_CHAR_POSY)return;		    
//	LCD_WindowMax(x,y,x+size/2,y+size);	   /*设置窗口	*/										
//	LCD_SetCursor(x, y);                  /*设置光标位置 */
//  
//	LCD_WriteRAM_Prepare();               /*开始写入GRAM  */   
//	num=num-' ';                         /*得到偏移后的值 */
//	for(pos=0;pos<size;pos++){
//		if(size==12){
//			temp=asc2_1206[num][pos];/*调用1206字体*/
//		}else{ 
//			temp=asc2_1608[num][pos];		 /*调用1608字体	*/
//		}
//		for(t=0;t<size/2;t++){                 
//	    if(temp&0x01){			   /*从低位开始*/
//				LCD_WR_DATA(PenColor);  /*画字体颜色 一个点*/
//			}else{ 
//				LCD_WR_DATA(BackColor);	   /*画背景颜色 一个点*/     
//	      temp>>=1;
//			}
//	  }
//	}			
//	LCD_WindowMax(0x0000,0x0000,240,320);	/*恢复窗体大小*/	 
//} 

  
/**
*	@功能：指定位置写入字符串
*	@作者
*	@输入：X始末坐标值，Y始末坐标值，颜色值
*	@输出：
*/
//void LCD_ShowCharString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t PenColor, uint16_t BackColor){ 
//  
//	uint8_t size = 16;     /*---字符大小默认16*8---*/
//	 
//  if(x>MAX_CHAR_POSX){x=0;y+=size;}			         /*超出X轴字体最小单位，换行*/
//  if(y>MAX_CHAR_POSY){y=x=0;LCD_Clear(WHITE);}	 /*超出Y轴字体最小单位，回到原点，并且清屏*/
//  LCD_ShowChar(x, y, *p, size, PenColor, BackColor);			   /*0表示非叠加方式*/
//}

/**
*	@功能：查找汉字
*	@作者
*	@输入：要查找的汉字
*	@输出：
*/
//uint16_t  findHzIndex(uint8_t *hz){                            /* 在自定义汉字库在查找所要显示 */
//                                                      //的汉字的位置
//	uint16_t  i=0;
//	FNT_GB16 *ptGb16 = (FNT_GB16 *)GBHZ_16;		  /*ptGb16指向GBHZ_16*/
//	while(ptGb16[i].Index[0] > 0x80){
//		
//	  if((*hz == ptGb16[i].Index[0]) && (*(hz+1) == ptGb16[i].Index[1])){ /*汉字用两位来表示地址码*/
//	        return i;
//	  }
//	  i++;
//	  if(i > (sizeof((FNT_GB16 *)GBHZ_16) / sizeof(FNT_GB16) - 1)){  /* 搜索下标约束 */
//		    break;
//	  }
//	}
//	return 0;
//}

/**
*	@功能：指定位置显示单个汉字
*	@作者
*	@输入：X坐标，Y坐标，汉字内容，前景色，背景色
*	@输出：
*/					
//void WriteOneHz(uint16_t  x0, uint16_t  y0, uint8_t *pucMsk, uint16_t  PenColor, uint16_t  BackColor){
//    
//	uint16_t  i,j;
//  uint16_t  mod[16];                                      /* 当前字模 16*16 */
//  uint16_t  *pusMsk;                                      /* 当前字库地址  */
//  uint16_t  y;

//	uint16_t  size = 16;       /*汉字默认大小16*16*/
//  pusMsk = (uint16_t  *)pucMsk;

//  for(i=0; i<16; i++){                                    /* 保存当前汉字点阵式字模       */   
//    mod[i] = *pusMsk;                                /* 取得当前字模，半字对齐访问   */
//    mod[i] = ((mod[i] & 0xff00) >> 8) | ((mod[i] & 0x00ff) << 8);/* 字模交换高低字节*/
//		pusMsk = pusMsk+1;
//  }
//	y = y0;
//	LCD_WindowMax(x0,y0,x0+size,y0+size);	 	/*设置窗口*/
//	LCD_SetCursor(x0,y0);                       /*设置光标位置 */ 
//	LCD_WriteRAM_Prepare();                     /*开始写入GRAM*/  
//  for(i=0; i<16; i++){                                    /* 16行   */                                                 
//		for(j=0; j<16; j++){                                /* 16列   */       
//			if((mod[i] << j) & 0x8000){       /* 显示第i行 共16个点 */             
//			    LCD_WriteRAM(PenColor);
//      }else{
//				LCD_WriteRAM(BackColor);      /* 用读方式跳过写空白点的像素*/
//			}
//   }
//   y++;
//  }
//	LCD_WindowMax(0x0000,0x0000,240,320);  	/*恢复窗体大小*/
//}

/**
*	@功能：指定位置显示字符串
*	@作者
*	@输入：X坐标值，Y坐标值，字符串，前景色，背景色
*	@输出：
*/
//void LCD_ShowHzString(uint16_t  x0, uint16_t  y0, uint8_t *pcStr, uint16_t  PenColor, uint16_t  BackColor){
//	
//	#define MAX_HZ_POSX 224
//	#define MAX_HZ_POSY 304 
//	uint16_t  usIndex;
//	uint8_t size = 16; 
//	FNT_GB16 *ptGb16 = 0;    
//  ptGb16 = (FNT_GB16 *)GBHZ_16; 

//	if(x0>MAX_HZ_POSX){
//		x0=0;
//		y0+=size;
//	}			         /*超出X轴字体最小单位，换行*/
//	if(y0>MAX_HZ_POSY){
//		y0=x0=0;
//		LCD_Clear(WHITE);
//	}	   /*超出Y轴字体最小单位，回到原点，并且清屏*/

//	usIndex = findHzIndex(pcStr);
//	WriteOneHz(x0, y0, (uint8_t *)&(ptGb16[usIndex].Msk[0]),  PenColor, BackColor); /* 显示字符 */
//}

/**
*	@功能：指定位置显示字符串
*	@作者
*	@输入：X坐标值，Y坐标值，字符串，前景色，背景色
*	@输出：
*/
//void LCD_ShowString(uint16_t  x0, uint16_t  y0, uint8_t *pcStr, uint16_t  PenColor, uint16_t  BackColor){
//	
//	while(*pcStr!='\0'){
//	 	if(*pcStr>0x80){ /*显示汉字*/
//			LCD_ShowHzString(x0, y0, pcStr, PenColor, BackColor);
//			pcStr += 2;
//			x0 += 16;	
//		}else{           /*显示字符*/		
//			LCD_ShowCharString(x0, y0, pcStr, PenColor, BackColor);	
//			pcStr +=1;
//			x0+= 8;
//		}
//	}	
//}

/**
*	@功能：RGB转换，RRRRRGGGGGGBBBBB to BBBBBGGGGGGRRRRR
*	@作者
*	@输入：颜色值
*	@输出：
*/
uint16_t  LCD_RGBtoBGR(uint16_t  Color){	
	
  uint16_t   r, g, b, bgr;
  b = (Color>>0)  & 0x1f;	/* 提取B    */
  g = (Color>>5)  & 0x3f;	/* 中间六位 */
  r = (Color>>11) & 0x1f;	/* 提取R    */
  
  bgr =  (b<<11) + (g<<5) + (r<<0);
  return( bgr );
}

/**
*	@功能：指定位置绘制图片
*	@作者
*	@输入：X,Y坐标值，图片内容
*	@输出：
*/
//void LCD_DrawPicture(uint16_t  StartX,uint16_t  StartY,uint16_t  Xend,uint16_t  Yend,uint8_t *pic){
//	
//	static	uint16_t  i=0,j=0;
//	uint16_t  *bitmap = (uint16_t  *)pic;
//	/*设置图片显示窗口大小*/
//	LCD_WindowMax(StartX, StartY, Xend, Yend);	
//	LCD_SetCursor(StartX,StartY);
//	LCD_WriteRAM_Prepare();
//	for(j=0; j<Yend-StartY; j++){
//		for(i=0; i<Xend-StartX; i++) LCD_WriteRAM(*bitmap++); 	
//	}
//	/*恢复窗口*/
//	LCD_WindowMax(0, 0, 240, 320);
//}


void LCD_Pin_Set(){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
*	@功能：LCD初始化，多驱动IC支持
*	@作者
*	@输入：
*	@输出：
*/
void LCD_Init(void)
{
	LCD_Pin_Set();
	HAL_Delay(10); /* delay 50 ms */ 
	HAL_Delay(10); /* delay 50 ms */
	DeviceCode = LCD_ReadReg(0x0000); /*读取屏的ID号*/
	HAL_Delay(10); /* delay 50 ms */
	if(DeviceCode==0x8999){	   /*对应的驱动IC为SSD1298*/
	
		/*-----   Start Initial Sequence ------*/
		LCD_WriteReg(0x00, 0x0001); /*开启内部振荡器*/
		LCD_WriteReg(0x01, 0x3B3F); /*驱动输出控制 */
		LCD_WriteReg(0x02, 0x0600); /* set 1 line inversion	*/
		/*-------- Power control setup --------*/
		LCD_WriteReg(0x0C, 0x0007); /* Adjust VCIX2 output voltage */
		LCD_WriteReg(0x0D, 0x0006); /* Set amplitude magnification of VLCD63 */
		LCD_WriteReg(0x0E, 0x3200); /* Set alternating amplitude of VCOM */
		LCD_WriteReg(0x1E, 0x00BB); /* Set VcomH voltage */
		LCD_WriteReg(0x03, 0x6A64); /* Step-up factor/cycle setting  */
		/*-------- RAM position control --------*/
		LCD_WriteReg(0x0F, 0x0000); /* Gate scan position start at G0 */
		LCD_WriteReg(0x44, 0xEF00); /* Horizontal RAM address position */
		LCD_WriteReg(0x45, 0x0000); /* Vertical RAM address start position*/
		LCD_WriteReg(0x46, 0x013F); /* Vertical RAM address end position */
		/* ------ Adjust the Gamma Curve -------*/
		LCD_WriteReg(0x30, 0x0000);
		LCD_WriteReg(0x31, 0x0706);
		LCD_WriteReg(0x32, 0x0206);
		LCD_WriteReg(0x33, 0x0300);
		LCD_WriteReg(0x34, 0x0002);
		LCD_WriteReg(0x35, 0x0000);
		LCD_WriteReg(0x36, 0x0707);
		LCD_WriteReg(0x37, 0x0200);
		LCD_WriteReg(0x3A, 0x0908);
		LCD_WriteReg(0x3B, 0x0F0D);
		/*--------- Special command -----------*/
		LCD_WriteReg(0x28, 0x0006); /* Enable test command	*/
		LCD_WriteReg(0x2F, 0x12EB); /* RAM speed tuning	 */
		LCD_WriteReg(0x26, 0x7000); /* Internal Bandgap strength */
		LCD_WriteReg(0x20, 0xB0E3); /* Internal Vcom strength */
		LCD_WriteReg(0x27, 0x0044); /* Internal Vcomh/VcomL timing */
		LCD_WriteReg(0x2E, 0x7E45); /* VCOM charge sharing time */ 
		/*--------- Turn On display ------------*/
		LCD_WriteReg(0x10, 0x0000); /* Sleep mode off */
		HAL_Delay(30);              /* Wait 30mS  */
		LCD_WriteReg(0x11, 0x6870); /* Entry mode setup. 262K type B, take care on the data bus with 16it only */
		LCD_WriteReg(0x07, 0x0033); /* Display ON	*/
	}else if(DeviceCode==0x9325||DeviceCode==0x9328){
		LCD_WriteReg(0x00e7,0x0010);      
    LCD_WriteReg(0x0000,0x0001);  			//start internal osc
    LCD_WriteReg(0x0001,0x0100);     
    LCD_WriteReg(0x0002,0x0700); 				//power on sequence 
		//核心 
	  LCD_WriteReg(0x0003,(1<<12)|(1<<7)|(1<<5)|(1<<4)|(0<<3) ); 	//65K 
		
    LCD_WriteReg(0x0004,0x0000);                                   
    LCD_WriteReg(0x0008,0x0207);	           
    LCD_WriteReg(0x0009,0x0000);         
    LCD_WriteReg(0x000a,0x0000); 				//display setting         
    LCD_WriteReg(0x000c,0x0001);				//display setting          
    LCD_WriteReg(0x000d,0x0000); 				//0f3c          
    LCD_WriteReg(0x000f,0x0000);
		//Power On sequence //
    LCD_WriteReg(0x0010,0x0000);   
    LCD_WriteReg(0x0011,0x0007);
    LCD_WriteReg(0x0012,0x0000);                                                                 
    LCD_WriteReg(0x0013,0x0000);                 

    LCD_WriteReg(0x0010,0x1590);   
    LCD_WriteReg(0x0011,0x0227);

    LCD_WriteReg(0x0012,0x009c);                  
      
    LCD_WriteReg(0x0013,0x1900);   
    LCD_WriteReg(0x0029,0x0023);
    LCD_WriteReg(0x002b,0x000e);

    LCD_WriteReg(0x0020,0x0000);                                                            
    LCD_WriteReg(0x0021,0x0000);           

		LCD_WriteReg(0x0030,0x0007); 
		LCD_WriteReg(0x0031,0x0707);   
    LCD_WriteReg(0x0032,0x0006);
    LCD_WriteReg(0x0035,0x0704);
    LCD_WriteReg(0x0036,0x1f04); 
    LCD_WriteReg(0x0037,0x0004);
    LCD_WriteReg(0x0038,0x0000);        
    LCD_WriteReg(0x0039,0x0706);     
    LCD_WriteReg(0x003c,0x0701);
    LCD_WriteReg(0x003d,0x000f);

    LCD_WriteReg(0x0050,0x0000);        
    LCD_WriteReg(0x0051,0x00ef);   
    LCD_WriteReg(0x0052,0x0000);     
    LCD_WriteReg(0x0053,0x013f);

    LCD_WriteReg(0x0060,0xa700);        
    LCD_WriteReg(0x0061,0x0001); 
    LCD_WriteReg(0x006a,0x0000);
    LCD_WriteReg(0x0080,0x0000);
    LCD_WriteReg(0x0081,0x0000);
    LCD_WriteReg(0x0082,0x0000);
    LCD_WriteReg(0x0083,0x0000);
    LCD_WriteReg(0x0084,0x0000);
    LCD_WriteReg(0x0085,0x0000);
      
		LCD_WriteReg(0x0090,0x0010);     
    LCD_WriteReg(0x0092,0x0000);  
    LCD_WriteReg(0x0093,0x0003);
    LCD_WriteReg(0x0095,0x0110);
    LCD_WriteReg(0x0097,0x0000);        
    LCD_WriteReg(0x0098,0x0000);  
         //display on sequence     
    LCD_WriteReg(0x0007,0x0133);
    
    LCD_WriteReg(0x0020,0x0000);                                                            
    LCD_WriteReg(0x0021,0x0000);
	}		
	HAL_Delay(50);	     /*延时50ms*/
	LCD_Clear(BLACK);
}
