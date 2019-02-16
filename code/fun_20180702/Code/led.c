

#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "led.h"
#include "motor.h"

#define LED1_OUTPUT		P13_PushPull_Mode
#define LED2_OUTPUT		P00_PushPull_Mode
#define LED3_OUTPUT		P01_PushPull_Mode
#define LED4_OUTPUT		P03_PushPull_Mode
#define LED_R_OUTPUT	P11_PushPull_Mode
#define LED_G_OUTPUT	P15_PushPull_Mode
#define LED_B_OUTPUT	P10_PushPull_Mode

#define LED1_SETTING(x)		P13 = x
#define LED2_SETTING(x)		P00 = x
#define LED3_SETTING(x)		P01 = x
#define LED4_SETTING(x)		P03 = x
#define LED_R_SETTING(x)	P11 = x
#define LED_G_SETTING(x)	P15 = x
#define LED_B_SETTING(x)	P10 = x

extern unsigned char is_5ms_Flag;

char led_white_mode;
bit led_rgb_mode;
bit led_white_tog;
unsigned int led_display_time = 0;

void Init_LED(void)
{
	LED1_OUTPUT;
	LED2_OUTPUT;
	LED3_OUTPUT;
	LED4_OUTPUT;
	LED1_SETTING(0);
	LED2_SETTING(0);
	LED3_SETTING(0);
	LED4_SETTING(0);	
	
	LED_R_OUTPUT;
	LED_G_OUTPUT;
	LED_B_OUTPUT;
	
	LED_R_SETTING(0);
	LED_G_SETTING(0);
	LED_B_SETTING(0);
	
	led_white_mode = 0;
	led_rgb_mode = 0;
}

/*
led_type:	LED Or RGB(0:1)
led_value: led value
led_mode:	Normal or Flash(0:1)
*/
void LED_WHITE_Set_value(unsigned char led_value)
{
	LED1_SETTING(led_value&LED1);
	led_value = led_value >> 1;
	LED2_SETTING(led_value&LED1);
	led_value = led_value >> 1;
	LED3_SETTING(led_value&LED1);
	led_value = led_value >> 1;
	LED4_SETTING(led_value&LED1);	
}
void LED_WHITE_Setting(unsigned char led_value,unsigned char led_mode)
{
	led_white_mode = led_mode;
	LED_WHITE_Set_value(led_value);
	led_display_time = 0;
	led_white_tog = 0;
}

void LED_RGB_Set_value(unsigned char led_value)
{
//	LED_R_SETTING(led_value&LEDR);
	if (led_value&LEDR)
	{
		LED_G_SETTING(1);
		LED_B_SETTING(1);
	}
	else
	{
		led_value = led_value >> 1;
		LED_B_SETTING(led_value&LEDR);
		led_value = led_value >> 1;
		LED_G_SETTING(led_value&LEDR);	
	}
}
void LED_RGB_Setting(unsigned char led_value,unsigned char led_mode)
{
	led_rgb_mode = led_mode;
	LED_RGB_Set_value(led_value);
}

void LED_Process(unsigned char led_value)
{
	if (led_white_mode == 1)
	{
		if (led_display_time == 600)
		{
			led_display_time = 0;
			if (led_white_tog == 0)
			{
				led_white_tog = 1;
				led_display_time = 1;
				LED_WHITE_Set_value(0);
			}
			else
			{
				led_white_tog = 0;
				led_display_time = 0;
				LED_WHITE_Set_value(led_value);
			}
		}
	}
	else if (led_white_mode == 2)
	{
		if (led_display_time == 200)
		{
			led_display_time = 0;
			if (led_white_tog == 0)
			{
				led_white_tog = 1;
				led_display_time = 1;
				led_value = (led_value<<1) + 1;
				LED_WHITE_Set_value(led_value);
			}
			else
			{
				led_white_tog = 0;
				led_display_time = 0;
				LED_WHITE_Set_value(led_value);
			}
		}		
	}
	
	if (led_rgb_mode)
	{
	}
}
