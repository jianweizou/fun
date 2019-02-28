
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
unsigned char batlevelledtimeout;
extern unsigned char Motor_Level;
extern unsigned int led_display_time;
/********************************/
#define Stage_A		1
#define Stage_B		2
#define Stage_C		4
#define Stage_D		8

unsigned char system_stage;
bit isneedinitstage;
bit isneedinitbatled;
bit isneedinitsys;
bit ischarging;
bit isstartsystem;
unsigned char startADC_cnt;
unsigned char batlevel_led_value;
unsigned char led_type;
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

unsigned char batlevel_to_led_value(void)
{
	unsigned char val;
	if (batlevel == 4)
	{
		val = 0x0F;
	}
	else if (batlevel == 3)
	{
		val = 0x07;
	}
	else if (batlevel == 2)
	{
		val = 0x03;
	}
	else if (batlevel == 1)
	{
		val = 0x01;
	}
	else
	{
		val = 0;
	}
	return val;
}

unsigned char getbatlevel(void)
{
	unsigned int adcvalue;
	unsigned char templevel;
	unsigned char i;
	if (startADC_cnt > 0)
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
				adcvalue = adcvalue - 0x80;	//0x90 -> 0.3v
			}
			if (adcvalue > 0xCA0)	//>75%	8v		
			{
				templevel = 4;
			}
			else if (adcvalue > 0xC40)	//>50%	7.7v
			{
				templevel = 3;
			}
			else if (adcvalue > 0xBD0)	//>25%	7.4
			{
				templevel = 2;
			}
			else if (adcvalue > 0xB60)	//<25%	7v
			{
				templevel = 1;
			}
			else if (adcvalue < 0xA00)
			{
				templevel = 0;
			}
//			if (adc_pre_cnt > 4)
//			{
//				adc_pre_cnt = 10;
//				if (ischarging)
//				{
//					if (templevel >= batlevel)
//						batlevel = templevel;
//				}
//				else
//				{
//					if (templevel <= batlevel)
//						batlevel = templevel;
//				}
//			}
//			else
				batlevel = templevel;
			adc_pre_cnt++;
			batlevel_led_value = batlevel_to_led_value();
			return 1;
		}
	}	
	return 0;
}

//----------------------------------------------------------------------------------
// UART0 baud rate initial setting 
//----------------------------------------------------------------------------------
void InitialUART0_Timer1(unsigned long u32Baudrate)    //T1M = 1, SMOD = 1
{
		P06_Quasi_Mode;		//Setting UART pin as Quasi mode for transmit
		P07_Quasi_Mode;		//Setting UART pin as Quasi mode for transmit
	
    SCON = 0x50;     	//UART0 Mode1,REN=1,TI=1
    TMOD |= 0x20;    	//Timer1 Mode1
    
    set_SMOD;        	//UART0 Double Rate Enable
    set_T1M;
    clr_BRCK;        	//Serial port 0 baud rate clock source = Timer1
 
#ifdef FOSC_160000
    TH1 = 256 - (1000000/u32Baudrate+1);               /*16 MHz */
#endif    	
#ifdef FOSC_166000
    TH1 = 256 - (1037500/u32Baudrate);         		     /*16.6 MHz */
#endif
    set_TR1;
		set_TI;						//For printf function must setting TI = 1
}
void Timer3_Delay5ms(unsigned long u32CNT)
{
    T3CON = 0x07;                           		//Timer3 Clock = Fsys/128
    set_TR3;                                		//Trigger Timer3
    while (u32CNT != 0)
    {
        RL3 = (65536-6667)>>8; //Find  define in "Function_define.h" "TIMER VALUE"
        RH3 = (65536-6667)&0xFF;
        while ((T3CON&SET_BIT4) != SET_BIT4);		//Check Timer3 Time-Out Flag
        clr_TF3;
        u32CNT --;
    }
    clr_TR3;                                		//Stop Timer3
}

unsigned int adcvalue;
void SysInit(void)
{
	Init_LED();
//	Timer0_Init();	
	startADC_cnt = 0;
	adccnt = 0;
	KeyInit();	
	P04_OpenDrain_Mode;
	Timer3_Delay5ms(1);
		Enable_ADC_AIN5;						// Enable AIN0 P1.7 as ADC input, Find in "Function_define.h" - "ADC INIT"
		while(1)
    {
			clr_ADCF;
			set_ADCS;									// ADC start trig signal
      while(ADCF == 0);
		adcvalue = ADCRH;
		adcvalue <<= 4;
		adcvalue |= ADCRL&0x0F;
		adcvalue = adcvalue >> 1;
			printf ("\n Value = 0x%x",adcvalue);
			Timer3_Delay5ms(1);
    }	
//	InitPWM();
	system_stage = Stage_A;
	isneedinitstage = 1;
	led_type = 0; 
	isneedinitbatled = 0;
	batlevelledtimeout = 0;	
	batlevel_led_value = 0;
	ischarging = 0;
	adc_pre_cnt = 0;
	P07_PushPull_Mode;
	P07 = 1;
	P07_Input_Mode;
	Enable_ADC_AIN5;
	if (isstartsystem == 0)
	{
		while(dpdtime<300)
		{
			getbatlevel();
//			startADC_cnt++;
		}
		isstartsystem = 1;
	}
	dpdtime = 0;
	led_display_time = 0;
	startADC_cnt = 0;
//	while(1)
//	{
//		if (getbatlevel() == 1)
//			break;
//	}
}

void main(void)
{
	unsigned char keystatus,i;
	Set_All_GPIO_Quasi_Mode;
	
		set_CLOEN;
		Set_All_GPIO_Quasi_Mode;					// Define in Function_define.h
		InitialUART0_Timer1(115200);
	
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
			keystatus = KeyScan();
			if (system_stage == Stage_A)
			{
				if (isneedinitstage == 1)
				{
					isneedinitstage = 0;
					//init stage A
					LED_WHITE_Setting(0,0);
					LED_RGB_Setting(0,0);
					led_type = 0;
					dpdtime = 0;
					ischarging = 0;
				}
				if (keystatus & 0x01)//key
				{
					//display power as LED
					led_type = 1;
					isneedinitbatled = 1;
					batlevelledtimeout = 200;
				}
				if (keystatus & 0x02)
				{
					system_stage = Stage_B;
					isneedinitstage = 1;
				}
				if (keystatus & 0x04)
				{
					if (system_stage == Stage_B)
						system_stage = Stage_D;
					else
						system_stage = Stage_C;
					isneedinitstage = 1;
				}			
			}
			else if (system_stage == Stage_B)
			{
				if (isneedinitstage == 1)
				{
					isneedinitstage = 0;
					//init stage B
					led_type = 2;	
					isneedinitbatled = 1;
					dpdtime = 0;
					ischarging = 0;
				}
				if (keystatus & 0x01)//key
				{
					//change pwm
					i  = Change_Motor_PWM();
					LED_RGB_Setting(i,0);
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
					LED_RGB_Setting(0,0);
				}
				
				if (keystatus & 0x04)//charging
				{
					if (system_stage == Stage_A)
					{
						system_stage = Stage_C;
					}
					else
						system_stage = Stage_D;
					TurnOffMotor();
					LED_RGB_Setting(0,0);
					isneedinitstage = 1;
				}
				
				if (keystatus & 0x08)
				{
					//turn off pwm
					TurnOffMotor();
					LED_RGB_Setting(0,0);
				}
				
			}
			else if (system_stage == Stage_C)
			{
				if (isneedinitstage == 1)
				{
					isneedinitstage = 0;
					//init stage C
					led_type = 3;	
					isneedinitbatled = 1;
					dpdtime = 0;
					ischarging = 1;
					//turn off pwm
					TurnOffMotor();
					LED_RGB_Setting(0,0);
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
					if (system_stage == Stage_D)
					{
						system_stage = Stage_B;
					}
					else
						system_stage = Stage_A;
					isneedinitstage = 1;
				}
			}
			else if (system_stage == Stage_D)
			{
				if (isneedinitstage == 1)
				{
					isneedinitstage = 0;
					isneedinitbatled = 1;
					//init stage D
					led_type = 3;
					dpdtime = 0;
					ischarging = 1;
				}
				if (keystatus & 0x01)//key
				{
					//change pwm
					i  = Change_Motor_PWM();
					LED_RGB_Setting(i,0);
				}
				if (keystatus & 0x02)//safety
				{
				}
				else
				{
					system_stage = Stage_C;
					isneedinitstage = 1;
					TurnOffMotor();
					LED_RGB_Setting(0,0);
				}
				
				if (keystatus & 0x04)//charging
				{
					dpdtime = 0;
				}
				else
				{
					if (system_stage == Stage_C)
					{
						system_stage = Stage_A;
					}
					else
						system_stage = Stage_B;
					isneedinitstage = 1;	
					TurnOffMotor();	
					LED_RGB_Setting(0,0);
				}
				
				if (keystatus & 0x08)
				{
					//turn off pwm
					TurnOffMotor();
					LED_RGB_Setting(0,0);
				}
			}
			
			//ADC process
			getbatlevel();
			if (led_type)
			{
				if(isneedinitbatled && batlevel_led_value)
				{
					i = led_type - 1;
					LED_WHITE_Setting(batlevel_led_value,i);
					isneedinitbatled = 0;
				}
				if(batlevelledtimeout==0)
				{
					if (led_type == 1)
					{
						led_type = 0;
						LED_WHITE_Setting(0,0);
					}
				}
			}
			//pwm rate
			if (check_motor_done())
			{
				//turn off pwm
				TurnOffMotor();
				LED_RGB_Setting(0,0);
				//goto stage A,B
				#warning "change stage to A or B"
			}
			if(get_motor_level())
			{
				dpdtime = 0;
			}
			//LED_Process		
			LED_Process(batlevel_led_value);
			
			//dpd
			if (dpdtime >= 2000)
			{
				dpdtime = 0;
				//enter dpd
				//
				Set_All_GPIO_Quasi_Mode;
				clr_ADCEN;
				
				TurnOffMotor();
				LED_WHITE_Setting(0,0);
				LED_RGB_Setting(0,0);
				DeInit_LED();
				
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