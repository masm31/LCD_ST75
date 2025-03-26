/*
ST75161 LCD driver for My Costum Library
Shiraz Janaury 2025
Masoud MAhmoudi
*/



#include "main.h"
#include "stm32f1xx_hal.h"
#ifndef __STM32F1xx_HAL_GPIO_H
#define __STM32F1xx_HAL_GPIO_H
#endif

#include <stdbool.h>  // For boolean data type (bool, true, false)


#define D0_PORT    GPIOB
#define D0_PIN     GPIO_PIN_8

#define D1_PORT    GPIOB
#define D1_PIN     GPIO_PIN_9

#define D2_PORT    GPIOC
#define D2_PIN     GPIO_PIN_12

#define D3_PORT    GPIOC
#define D3_PIN     GPIO_PIN_11

#define D4_PORT    GPIOC
#define D4_PIN     GPIO_PIN_10

#define D5_PORT    GPIOA
#define D5_PIN     GPIO_PIN_15

#define D6_PORT    GPIOA
#define D6_PIN     GPIO_PIN_12

#define D7_PORT    GPIOA
#define D7_PIN     GPIO_PIN_11



// Pin definitions (configure these based on your design)
#define CSB_PIN GPIO_PIN_14
#define CSB_PORT GPIOC
#define SDA_PIN GPIO_PIN_9
#define SDA_PORT GPIOB
#define SCL_PIN GPIO_PIN_8
#define SCL_PORT GPIOB
#define A0_PIN GPIO_PIN_15
#define A0_PORT GPIOC
#define RES_PIN GPIO_PIN_0
#define RES_PORT GPIOC



#define LCD_PORT_IF0 GPIOC
#define LCD_Pin_IF0 GPIO_PIN_13
#define LCD_PORT_IF1 GPIOD
#define LCD_Pin_IF1 GPIO_PIN_2





#define IF1_PORT   GPIOD
#define IF1_PIN    GPIO_PIN_2

#define IF0_PORT   GPIOC
#define IF0_PIN    GPIO_PIN_13

#define LCD_PORT_ERD GPIOB
#define LCD_Pin_ERD GPIO_PIN_5
#define LCD_PORT_RWR GPIOB
#define LCD_Pin_RWR GPIO_PIN_4

#define LED_PORT GPIOB
#define LED_Pin GPIO_PIN_13




// Helper macros
//#define CSB_HIGH() HAL_GPIO_WritePin(CSB_PORT, CSB_PIN, GPIO_PIN_SET)
//#define CSB_LOW() HAL_GPIO_WritePin(CSB_PORT, CSB_PIN, GPIO_PIN_RESET)
//#define SDA_HIGH() HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET)
//#define SDA_LOW() HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET)
//#define SCL_HIGH() HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET)
//#define SCL_LOW() HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET)
//#define A0_HIGH() HAL_GPIO_WritePin(A0_PORT, A0_PIN, GPIO_PIN_SET)
//#define A0_LOW() HAL_GPIO_WritePin(A0_PORT, A0_PIN, GPIO_PIN_RESET)
//#define RES_HIGH() HAL_GPIO_WritePin(RES_PORT, RES_PIN, GPIO_PIN_SET)
//#define RES_LOW() HAL_GPIO_WritePin(RES_PORT, RES_PIN, GPIO_PIN_RESET)

//#define D2_HIGH() HAL_GPIO_WritePin(D2_PORT, D2_PIN, GPIO_PIN_SET)
//#define D2_LOW() HAL_GPIO_WritePin(D2_PORT, D2_PIN, GPIO_PIN_RESET)
//#define D3_HIGH() HAL_GPIO_WritePin(D3_PORT, D3_PIN, GPIO_PIN_SET)
//#define D3_LOW() HAL_GPIO_WritePin(D3_PORT, D3_PIN, GPIO_PIN_RESET)
//#define D4_HIGH() HAL_GPIO_WritePin(D4_PORT, D4_PIN, GPIO_PIN_SET)
//#define D4_LOW() HAL_GPIO_WritePin(D4_PORT, D4_PIN, GPIO_PIN_RESET)
//#define D5_HIGH() HAL_GPIO_WritePin(D5_PORT, D5_PIN, GPIO_PIN_SET)
//#define D5_LOW() HAL_GPIO_WritePin(D5_PORT, D5_PIN, GPIO_PIN_RESET)
//#define D6_HIGH() HAL_GPIO_WritePin(D6_PORT, D6_PIN, GPIO_PIN_SET)
//#define D6_LOW() HAL_GPIO_WritePin(D6_PORT, D6_PIN, GPIO_PIN_RESET)
//#define D7_HIGH() HAL_GPIO_WritePin(D7_PORT, D7_PIN, GPIO_PIN_SET)
//#define D7_LOW() HAL_GPIO_WritePin(D7_PORT, D7_PIN, GPIO_PIN_RESET)


#define CSB_HIGH()  (CSB_PORT->BSRR = CSB_PIN)    // Set CSB HIGH
#define CSB_LOW()   (CSB_PORT->BSRR = (CSB_PIN << 16))  // Set CSB LOW

#define SDA_HIGH()  (SDA_PORT->BSRR = SDA_PIN)  // Set SDA HIGH
#define SDA_LOW()   (SDA_PORT->BSRR = (SDA_PIN << 16))  // Set SDA LOW

#define SCL_HIGH()  (SCL_PORT->BSRR = SCL_PIN)  // Set SCL HIGH
#define SCL_LOW()   (SCL_PORT->BSRR = (SCL_PIN << 16))  // Set SCL LOW

#define A0_HIGH()   (A0_PORT->BSRR = A0_PIN)
#define A0_LOW()    (A0_PORT->BSRR = (A0_PIN << 16))

#define RES_HIGH()  (RES_PORT->BSRR = RES_PIN)
#define RES_LOW()   (RES_PORT->BSRR = (RES_PIN << 16))

#define D2_HIGH()   (D2_PORT->BSRR = D2_PIN)
#define D2_LOW()    (D2_PORT->BSRR = (D2_PIN << 16))
#define D3_HIGH()   (D3_PORT->BSRR = D3_PIN)
#define D3_LOW()    (D3_PORT->BSRR = (D3_PIN << 16))
#define D4_HIGH()   (D4_PORT->BSRR = D4_PIN)
#define D4_LOW()    (D4_PORT->BSRR = (D4_PIN << 16))
#define D5_HIGH()   (D5_PORT->BSRR = D5_PIN)
#define D5_LOW()    (D5_PORT->BSRR = (D5_PIN << 16))
#define D6_HIGH()   (D6_PORT->BSRR = D6_PIN)
#define D6_LOW()    (D6_PORT->BSRR = (D6_PIN << 16))
#define D7_HIGH()   (D7_PORT->BSRR = D7_PIN)
#define D7_LOW()    (D7_PORT->BSRR = (D7_PIN << 16))




#define LCD_WR_HIGH() HAL_GPIO_WritePin(LCD_PORT_RWR, LCD_Pin_RWR, GPIO_PIN_SET)
#define LCD_WR_LOW() HAL_GPIO_WritePin(LCD_PORT_RWR, LCD_Pin_RWR, GPIO_PIN_RESET)





void ST75161_init(void);
void ST75161_clear(void);
void ST75161_clear_area(uint8_t x_start, uint8_t y_start, uint8_t width, uint8_t height);

void lcd_init(uint8_t grayscale);
void lcd_fill_gray(uint8_t gray_level);

//void lcd_write_char(char c);

void MY_LED_POWER(GPIO_PinState Status);
void flip_display_horizontal(uint8_t enable) ;
void flip_display_horizontal(uint8_t enable);
void flip_display_vertical(uint8_t enable);
void set_grayscale_levels(void);
void LCD_SetInterfaceMode(uint8_t mode);
void lcd_draw_black_box();
void lcd_draw_black_box_gray();
void Fill_BLACK(void);
void Fill_8_row(int row,int column);
void lcd_set_pixel(uint8_t x, uint8_t y, uint8_t color) ;
void lcd_update_screen(void);
void lcd_draw_rectangle(uint8_t x_start, uint8_t y_start, uint8_t width, uint8_t height);
void lcd_draw_rectangle_border(uint8_t x_start, uint8_t y_start, uint8_t width, uint8_t height, uint8_t thickness) ;
void lcd_set_pixel(uint8_t x, uint8_t y, uint8_t color) ;
void lcd_set_pixel_gray(uint8_t x, uint8_t y, uint8_t gray_level);
void lcd_update_screen(void);
void lcd_draw_char(uint8_t x, uint8_t y, char c, const uint8_t *font, uint8_t font_width, uint8_t font_height) ;

void lcd_draw_char_bold(uint8_t x, uint8_t y, char c, const uint8_t *font, uint8_t font_width, uint8_t font_height);
void lcd_draw_string(uint8_t x, uint8_t y, char *str, const uint8_t *font, uint8_t font_width, uint8_t font_height) ;
void lcd_draw_string_bold(uint8_t x, uint8_t y, char *str, const uint8_t *font, uint8_t font_width, uint8_t font_height);

void lcd_draw_char_inverted(uint8_t x, uint8_t y, char c, const uint8_t *font, uint8_t font_width, uint8_t font_height);
void lcd_draw_string_inverted(uint8_t x, uint8_t y, char *str, const uint8_t *font, uint8_t font_width, uint8_t font_height) ;


void ST75161_show_image(uint8_t *image, uint8_t width, uint8_t height);
void lcd_draw_bitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height);
uint8_t reverse_byte(uint8_t b);
void lcd_draw_bitmap_Reverse(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height);
void lcd_draw_bitmap_gray(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height);
void lcd_draw_char_gray(uint8_t x, uint8_t y, char c, const uint16_t *font, uint8_t font_width, uint8_t font_height,uint8_t grayLevel) ;
void lcd_set_pixel_gray2(uint8_t x, uint8_t y, uint8_t gray_level);
void convertFontToGray(const uint8_t font5x7[][5], uint16_t font5x7_gray[][5], int numChars, uint8_t grayLevel) ;
void lcd_draw_string_gray(uint8_t x, uint8_t y, char *str, const uint16_t *font, uint8_t font_width, uint8_t font_height, uint8_t gray_level);
void lcd_draw_char_gray_background(uint8_t x, uint8_t y, char c, const uint16_t *font, uint8_t font_width, uint8_t font_height, uint8_t grayLevel, uint8_t bgGrayLevel, bool invert);
void convertFont(const uint8_t font5x7[][5], uint16_t font5x7_gray[][5], int numChars);
void lcd_draw_rect_gray(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t gray_level);





 


