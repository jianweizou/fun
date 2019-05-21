
#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "dataflash.h"
#include "key.h"
#include "motor.h"
#include "led.h"

#define TH0_INIT        (65536-6667)/256 //5.0ms@XTAL=12MHz, Period = (10.85/2) ms@XTAL=22.1184MHz
#define TL0_INIT        (65536-6667)%256
#define TH1_INIT        0xE0 //2.5ms@XTAL=12MHz, Period = (5.425/2) ms@XTAL=22.1184MHz
#define TL1_INIT        0x00

bit BIT_TMP;

unsigned char is_5ms_Flag;
unsigned int dpdtime;
#define ADC_CNT	8
unsigned char adccnt;
unsigned int adc[ADC_CNT];
unsigned char batlevel;
unsigned int batlevelledtimeout;
unsigned int adcvalue;
extern unsigned char Motor_Level;
extern unsigned int led_display_time;
/********************************/
#define Stage_A		1
#define Stage_B		2
#define Stage_C		4
#define Stage_D		8

unsigned char system_stage;
bit isneedinitstage;
bit isneedinitsys;
bit ischarging;
bit isstartsystem;
unsigned char startADC_cnt;
unsigned char batlevel_led_value;
unsigned char adc_pre_cnt;
/*******************************/


/************************************************************************************************************
*    TIMER 0 interrupt subroutine
************************************************************************************************************/
void Timer0_ISR (void) interrupt 1          //interrupt address is 0x000B
{
    TH0 = TH0_INIT;
    TL0 = TL0_INIT;
	is_5ms_Flag = 1;
	dpdtime++;
	startADC_cnt ++;
	led_display_time++;
	if (isStartMotor)
		Motor_done_cnt++;
}

void Timer0_Init(void)
{
	TMOD = 0XFF;
	TIMER0_MODE1_ENABLE;                        //Timer 0  mode configuration
    
	clr_T0M;
    
	TH0 = TH0_INIT;
	TL0 = TL0_INIT;
    
	set_ET0;                                    //enable Timer0 interrupt
	set_EA;                                     //enable interrupts
	
	set_TR0;                                    //Timer0 run
}

unsigned int getadcvalue(void)
{
	unsigned char i;
	unsigned int adcvalue;
	adcvalue = 0;
	for(i=0;i<ADC_CNT;i++)
	{
		adcvalue += adc[i];
	}
	adcvalue >>=3;
	return adcvalue;
}


unsigned char getbatlevel(unsigned char adc_delay)
{
	unsigned char templevel;
	unsigned char i;
	unsigned char cur_pwm_level;
	if (startADC_cnt > adc_delay)
	{
		startADC_cnt = 0;
		clr_ADCF;
		set_ADCS;									// ADC start trig signal
		while(ADCF == 0);
		adcvalue = ADCRH;
		adcvalue <<= 4;
		adcvalue |= ADCRL&0x0F;
		adc[adccnt] = adcvalue;
		adccnt++;
		if (adccnt >= ADC_CNT)
		{
			adccnt = 0;
			adcvalue = getadcvalue();
			i = get_motor_level();
			if (i == 1)
			{
				cur_pwm_level = cur_pwm();
				if (cur_pwm_level == 1)
					adcvalue = adcvalue + 0x08;
				else if (cur_pwm_level == 2)
					adcvalue = adcvalue + 0x10;
				else
					adcvalue = adcvalue + 0x20;
			}
			else if (i == 2)
			{
				adcvalue = adcvalue + 0x10;
			}
			else if (i == 4)
			{
				adcvalue = adcvalue + 0x08;
			}
			if (ischarging)
			{
				if (batlevel < 3)
					adcvalue = adcvalue - 0x50;
				else if (batlevel < 5)
					adcvalue = adcvalue - 0x40;
				else
					adcvalue = adcvalue - 0x30;	//0x90 -> 0.3v
			}
			if (adcvalue > 0xD20)		//100% 8.2
			{
				templevel = 6;
			}
			else if (adcvalue > 0xCB0)	//>75%	8v		
			{
				templevel = 5;
			}
			else if (adcvalue > 0xC40)	//>50%	7.7v
			{
				templevel = 4;
			}
			else if (adcvalue > 0xBE0)	//>25%	7.5
			{
				templevel = 3;
			}
			else if (adcvalue > 0xBA0)	//10%	7.2v
			{
				templevel = 2;
			}
			else
			{
				templevel = 1;
			}
			if (batlevel != templevel)
				adc_pre_cnt++;
			else
				adc_pre_cnt = 0;
			if (adc_pre_cnt > 4)
			{
				adc_pre_cnt = 0;
				batlevel = templevel;
			}
			return 1;
		}
	}	
	return 0;
}


void SysInit(void)
{
	Init_LED();
	Timer0_Init();	
	startADC_cnt = 0;
	adccnt = 0;
	KeyInit();	
	InitPWM();
	system_stage = Stage_A;
	isneedinitstage = 1;
	batlevelledtimeout = 0;	
	batlevel_led_value = 0;
	ischarging = 0;
	adc_pre_cnt = 0;
	Enable_ADC_AIN2;
	if (isstartsystem == 0)
	{
		while(dpdtime<60)
		{
			getbatlevel(0);
		}
		isstartsystem = 1;
	}
	else
	{
		while(dpdtime<20)
		{
			getbatlevel(1);
		}		
	}
	dpdtime = 0;
	led_display_time = 0;
	startADC_cnt = 0;
}

void P05_wakeup_init(void){
	P05_Input_Mode;
	
	PINEN = 0x20;
	PIPEN = 0x00;
	PICON = 0x40;
	
	set_EPI;
	
	set_EA;
}

void main(void)
{
	unsigned char keystatus,i;
	Set_All_GPIO_Quasi_Mode;
	isneedinitsys = 1;
	isstartsystem = 0;
	while(1)
	{
		if (isneedinitsys)
		{
			SysInit();
			isneedinitsys = 0;
		}
		if (is_5ms_Flag)
		{
			if (batlevelledtimeout)
				batlevelledtimeout--;
			keystatus = KeyScan();		//keystatus->0:null；1：按键；2：安全开关；4：充电；8：长按
			if (system_stage == Stage_A)	//没用充电，安全开关没闭合
			{
				if (isneedinitstage == 1)
				{
					isneedinitstage = 0;
					//init stage A
					TurnOffMotor();
					LED_Setting(0);
					LED_RGB_Setting(0);
					dpdtime = 0;
					ischarging = 0;
				}
				if (keystatus & 0x01)//key
				{
					//display power as LED
					batlevelledtimeout = 600;
					LED_Setting(system_stage);
				}
				if (keystatus & 0x02)
				{
					system_stage = Stage_B;
					isneedinitstage = 1;
				}
				if (keystatus & 0x04)
				{
					system_stage = Stage_C;
					isneedinitstage = 1;
				}			
			}
			else if (system_stage == Stage_B)	//闭合安全开关
			{
				if (isneedinitstage == 1)
				{
					isneedinitstage = 0;
					//init stage B
					dpdtime = 0;
					ischarging = 0;
					//check motor level
					i = get_motor_level();					
					LED_RGB_Setting(i);
					if (i == 0)
						LED_Setting(0);
					else
						LED_Setting(system_stage);
				}
				if (keystatus & 0x01)//key
				{
					//change pwm
					i  = Change_Motor_PWM();
					LED_RGB_Setting(i);
					if (i == 0)
						LED_Setting(0);
					else
						LED_Setting(system_stage);
				}
				if (keystatus & 0x02)//safety
				{
				}
				else
				{
					system_stage = Stage_A;
					isneedinitstage = 1;
					//turn off pwm
					TurnOffMotor();
					LED_RGB_Setting(0);
					LED_Setting(0);
				}
				
				if (keystatus & 0x04)//charging
				{
					system_stage = Stage_D;
					isneedinitstage = 1;
				}
				
				if (keystatus & 0x08)
				{
					//turn off pwm
					TurnOffMotor();
					LED_RGB_Setting(0);
					LED_Setting(0);
				}
			}
			else if (system_stage == Stage_C)	//充电中
			{
				if (isneedinitstage == 1)
				{
					isneedinitstage = 0;
					//init stage C	
					dpdtime = 0;
					ischarging = 1;					
					TurnOffMotor();
					LED_RGB_Setting(0);
					LED_Setting(system_stage);
				}
				if (keystatus & 0x01)//key
				{
					
				}
				if (keystatus & 0x02)//safety
				{
					//enter stage_B
					system_stage = Stage_D;
					isneedinitstage = 1;
				}
				
				if (keystatus & 0x04)//charging
				{
					dpdtime = 0;
				}
				else
				{
					system_stage = Stage_A;
					ischarging = 0;
					LED_Setting(0);
					isneedinitstage = 1;
				}
			}
			else if (system_stage == Stage_D)	//充电和闭合安全开关
			{
				if (isneedinitstage == 1)
				{
					isneedinitstage = 0;
					//init stage D
					dpdtime = 0;
					ischarging = 1;
					//check motor level
					i = get_motor_level();					
					LED_RGB_Setting(i);
					LED_Setting(system_stage);
				}
				if (keystatus & 0x01)//key
				{
					//change pwm
					i  = Change_Motor_PWM();
					LED_RGB_Setting(i);
				}
				if (keystatus & 0x02)//safety
				{
				}
				else
				{
					system_stage = Stage_C;
					isneedinitstage = 1;
					TurnOffMotor();
					LED_RGB_Setting(0);
				}
				
				if (keystatus & 0x04)//charging
				{
					dpdtime = 0;
				}
				else
				{
					system_stage = Stage_B;
					ischarging = 0;
					i = get_motor_level();
					LED_RGB_Setting(i);
					if (i == 0)
						LED_Setting(0);
					else
						LED_Setting(system_stage);				
					isneedinitstage = 1;
				}
				
				if (keystatus & 0x08)
				{
					//turn off pwm
					TurnOffMotor();
					LED_RGB_Setting(0);
					LED_Setting(system_stage);
				}
			}
			
			//ADC process
			getbatlevel(5);
			//pwm rate
			if (check_motor_done())
			{
				//turn off pwm
				TurnOffMotor();
				LED_RGB_Setting(0);
				LED_Setting(0);
				isneedinitstage = 1;
			}
			if(get_motor_level())
			{
				dpdtime = 0;
			}
			//LED_Process		
			LED_Process(system_stage);
			
			//dpd
			if (dpdtime >= 2000)
			{
				dpdtime = 0;
				//enter dpd
				//
				Set_All_GPIO_Quasi_Mode;
				clr_ADCEN;
				
				TurnOffMotor();
				LED_WHITE_Setting(0);
				LED_RGB_Setting(0);
				DeInit_LED();
				
				P05_wakeup_init();
				
				P17_Input_Mode;
				set_P1S_7;
				set_EX1;
								
				set_PD;

				PICON  = 0;
							
				clr_EX0;
				clr_EX1;
				clr_EPI;
				isneedinitsys = 1;
			}
		}
	}
}

void EXT_INT1(void) interrupt 2
{
	
}

void PinInterrupt_ISR (void) interrupt 7
{
	if(PIF == 0x10)	//pin 0
		MOTOR_FG_PinInterrupt_ISR();
	PIF = 0x00;
}