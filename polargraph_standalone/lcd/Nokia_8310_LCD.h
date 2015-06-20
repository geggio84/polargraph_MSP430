/*
  * 3310 LCD User Manual
  * Nokia_init (); to make the initial settings for the LCD.
  * Nokia_ddram_temizle (), and the LCD of the DDRAM is deleted.
  * Nokia_komut_yaz (0x20) command is sent to þeklinde.
  * Nokia_veri_yaz (0x01); þeklinde data is sent.
  * Nokia_byte_yaz (0xFE) is sent a byte of information.
  * Nokia_lcd_git (2.3), and go to certain areas on the LCD. Limit: (0.0) to (83.5) tractor.
  * Nokia_contrast (0x0F), with the contrast adjustment is done.
  * Nokia_yazi_yaz ("Test"); þeklinde Text printed.
  * Nokia_karakter_yaz ('E'); þeklinde number is printed.
  * Lcdpixel (3,4); þeklinde pixe indicated.
  *
*/

#define rows 48
#define cols 84

#define char_h 8
#define char_w 6

#define num_rows rows/char_h
#define num_char cols/char_w

// PINS
#define nok_reset BIT0
#define nok_ce	  BIT1
#define nok_sdin  BIT3
#define nok_sclk  BIT2

extern void nokia_init(void);
extern void nokia_clear(void);
extern void nokia_write_command(char);
extern void nokia_write_data(char);
extern void nokia_write_byte(char );
extern void nokia_lcd_go(char, char);
extern void nokia_contrast(char);
extern void nokia_write_text(const char*);
extern void nokia_write_char(char);
extern void lcdpixel (char, char) ;
extern void wait(unsigned int);
