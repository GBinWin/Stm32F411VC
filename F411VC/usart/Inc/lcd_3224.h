#ifdef	LCD_3224_H
#define LCD_3224_H

void LCD_Clear(uint16_t Color);
void LCD_Fill(uint8_t xsta, uint16_t ysta, uint8_t xend, uint16_t yend, uint16_t colour);
void LCD_DrawLine(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend);
void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r);
void LCD_DrawRectangle(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend);
void LCD_WindowMax (unsigned int x,unsigned int y,unsigned int x_end,unsigned int y_end);
/*��ʾ�ַ� ������Ӣ��*/
void LCD_ShowString(uint16_t x0, uint16_t y0, uint8_t *pcStr, uint16_t PenColor, uint16_t BackColor);
/*��ʾͼƬ gImageȡģ bmp��ʽ*/
void LCD_DrawPicture(uint16_t StartX,uint16_t StartY,uint16_t Xend,uint16_t Yend,uint8_t *pic);
void LCD_Configuration(void);
void LCD_Init(void);


#endif 
