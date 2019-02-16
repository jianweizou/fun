
#define LED_WHITE	1
#define LED_RGB		2

#define LEDR	4
#define LEDG	3
#define LEDB	1

#define LED1	1
#define LED2	2
#define LED3	4
#define LED4	8

void Init_LED(void);
void LED_Process(unsigned char led_value);
void LED_WHITE_Setting(unsigned char led_value,unsigned char led_mode);
void LED_RGB_Setting(unsigned char led_value,unsigned char led_mode);