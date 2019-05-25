
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
unsigned int DPD_CNT=0;
unsigned char adcchangecnt=0;
unsigned char adc_dis_cnt=0;
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

float bat_val;



#define set_SWRST   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON|=SET_BIT7 ;EA=BIT_TMP;
void SW_Reset(void)
{
    TA = 0xAA;
    TA = 0x55;
    set_SWRST;
}

/*******************************/
typedef unsigned char         UINT8;
typedef unsigned int          UINT16;
typedef unsigned long         UINT32;

typedef unsigned char         uint8_t;
typedef unsigned int          uint16_t;
typedef unsigned long         uint32_t;
/*uart1*/
void InitialUART1_Timer3(UINT32 u32Baudrate) //use timer3 as Baudrate generator
{
		P02_Quasi_Mode;		//Setting UART pin as Quasi mode for transmit
		P16_Quasi_Mode;		//Setting UART pin as Quasi mode for transmit
	
	  SCON_1 = 0x50;   	//UART1 Mode1,REN_1=1,TI_1=1
    T3CON = 0x08;   	//T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1), UART1 in MODE 1
		clr_BRCK;
	
#ifdef FOSC_160000
		RH3    = HIBYTE(65536 - (1000000/u32Baudrate)-1);  		/*16 MHz */
		RL3    = LOBYTE(65536 - (1000000/u32Baudrate)-1);			/*16 MHz */
#endif
#ifdef FOSC_166000
		RH3    = HIBYTE(65536 - (1037500/u32Baudrate));  			/*16.6 MHz */
		RL3    = LOBYTE(65536 - (1037500/u32Baudrate));				/*16.6 MHz */
#endif
    set_TR3;         //Trigger Timer3
}
UINT8 Receive_Data_From_UART1(void)
{
    UINT8 c;
    
    while (!RI_1);
    c = SBUF_1;
    RI_1 = 0;
    return (c);
}

void Send_Data_To_UART1 (UINT8 c)
{
    TI_1 = 0;
    SBUF_1 = c;
    while(TI_1==0);
}
char putchar (char c)
{
		while (!TI_1);  /* wait until transmitter ready */
		TI_1 = 0;
		SBUF_1 = c;      /* output character */
		return (c);
}

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
	unsigned int adcvalue_temp;
	adcvalue_temp = 0;
	for(i=0;i<ADC_CNT;i++)
	{
		adcvalue_temp += adc[i];
	}
	adcvalue_temp >>=3;
	return adcvalue_temp;
}


unsigned char getbatlevel(unsigned char adc_delay)
{
	unsigned char templevel;
	unsigned char i;
	unsigned char cur_pwm_level;
	unsigned int adcvaluetemp;
	if (startADC_cnt > adc_delay)
	{
		startADC_cnt = 0;
		CKDIV = 0x04; 
		clr_ADCF;
		set_ADCS;									// ADC start trig signal
		while(ADCF == 0);
		CKDIV = 0x00; 
		adcvaluetemp = ADCRH;
		adcvaluetemp <<= 4;
		adcvaluetemp |= ADCRL&0x0F;
		adc[adccnt] = adcvaluetemp;
		adccnt++;
		if (adccnt >= ADC_CNT)
		{
			adccnt = 0;
			adcvaluetemp = getadcvalue();
			printf("\n adcvaluetemp=0x%x",adcvaluetemp);
			i = get_motor_level();
			if ((ischarging) &&(i != 0))
			{
				if (i == 1)
				{
					cur_pwm_level = cur_pwm();
					if (cur_pwm_level == 1)
						adcvaluetemp = adcvaluetemp + 0x20;
					else if (cur_pwm_level == 2)
						adcvaluetemp = adcvaluetemp + 0x38;
					else
						adcvaluetemp = adcvaluetemp + 0x60;
					adcvaluetemp = adcvaluetemp - 0x80;	//0x90 -> 0.3v
				}
				else if (i == 2)
				{
					adcvaluetemp = adcvaluetemp + 0x38;
					adcvaluetemp = adcvaluetemp - 0x90;	//0x90 -> 0.3v
				}
				else if (i == 4)
				{
					adcvaluetemp = adcvaluetemp + 0x20;
					adcvaluetemp = adcvaluetemp - 0xA0;	//0x90 -> 0.3v
				}
				else
				{
					adcvaluetemp = adcvaluetemp - 0xB0;	//0x90 -> 0.3v
				}
			}
			else if ((ischarging== 0) && (i != 0))
			{
				if (i == 1)
				{
					cur_pwm_level = cur_pwm();
					if (cur_pwm_level == 1)
						adcvaluetemp = adcvaluetemp + 0x28;
					else if (cur_pwm_level == 2)
						adcvaluetemp = adcvaluetemp + 0x48;
					else
						adcvaluetemp = adcvaluetemp + 0x70;
				}
				else if (i == 2)
				{
					adcvaluetemp = adcvaluetemp + 0x48;
				}
				else if (i == 4)
				{
					adcvaluetemp = adcvaluetemp + 0x28;
				}
			}
			else if ((ischarging == 1) && (i == 0))
			{
				adcvaluetemp = adcvaluetemp - 0xB0;
				if (adcvaluetemp > 0xD40)
					adcvaluetemp = 0xD40;
			}
			if (isstartsystem == 0)
			{
				if (adcvaluetemp >= 0xD40)
					adcvalue = adcvaluetemp - 0x40;
				else
				{
					adcvalue = adcvaluetemp + 0x100;
				}
				if (adcvalue > 0xD20)
					adcvalue = 0xD20;
			}
			else
			{
				if (adcvalue == 0)
					adcvalue = adcvaluetemp;
				if (adcvaluetemp > (adcvalue + 8))
				{
					adcchangecnt++;
					if (adcchangecnt > 5)
					{
						adcchangecnt = 0;
						adcvalue++;
					}
				}
				else
				{
					adcchangecnt = 0;
					adcvalue--;
				}
			}
			printf("\n adcvalue=0x%x",adcvalue);
			bat_val = ((float)adcvalue) *3.34 /4096.0 * 3.0;
			printf("\n bat=%0.4f",bat_val);
			if (adcvalue > 0xCD0)		//100% 8.2
			{
				templevel = 6;
			}
			else if (adcvalue > 0xCA0)	//>75%	7.9v		
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
			if (adc_pre_cnt > 5)
			{
				adc_pre_cnt = 0;
				batlevel = templevel;
			}
			return 1;
		}
	}	
	return 0;
}

void P05_wakeup_init(void){
	P05_Input_Mode;
	
	PINEN = 0x20;
	PIPEN = 0x00;
	PICON = 0x40;
	
	set_EPI;
	
	set_EA;
}
void Enter_DPD(void)
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

void SysInit(void)
{	
	InitialUART1_Timer3(115200);
	TI_1 = 1;
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
	adcvalue = 0;
	Enable_ADC_AIN2;
	
	if (isstartsystem != 1)
	{
		dpdtime = 0;
		while(dpdtime<200)
		{
			if (P05 == 0)
				ischarging = 1;
			getbatlevel(0);
		}
		isstartsystem = 1;
		DPD_CNT = 2000;
	}
	dpdtime = 0;
	led_display_time = 0;
	startADC_cnt = 0;
	adc_dis_cnt = 50;
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
					adccnt = 0;
					adc_dis_cnt = 400;
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
					led_display_time = 600;
					adccnt = 0;
					adc_dis_cnt = 400;
				}
				if (keystatus & 0x01)//key
				{
					//change pwm
					i  = Change_Motor_PWM();
					LED_RGB_Setting(i);
//					if (i == 0)
//						LED_Setting(0);
//					else
						LED_Setting(system_stage);
					adccnt = 0;
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
//					LED_Setting(0);
					adccnt = 0;
					adc_dis_cnt = 400;
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
					led_display_time = 600;
					adccnt = 0;
					adc_dis_cnt = 400;
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
					adccnt = 0;
					adc_dis_cnt = 400;
				}
				if (keystatus & 0x01)//key
				{
					//change pwm
					i  = Change_Motor_PWM();
					LED_RGB_Setting(i);
					adccnt = 0;
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
//					if (i == 0)
//						LED_Setting(0);
//					else
						LED_Setting(system_stage);				
					isneedinitstage = 1;
				}
				
				if (keystatus & 0x08)
				{
					//turn off pwm
					TurnOffMotor();
					LED_RGB_Setting(0);
//					LED_Setting(system_stage);
					adccnt = 0;
					adc_dis_cnt = 400;
				}
			}
			
			//ADC process
			if (adc_dis_cnt > 0)
			{
				adc_dis_cnt--;
			}
			else
			{
				getbatlevel(5);
			}
			//pwm rate
			if (check_motor_done())
			{
				//turn off pwm
				TurnOffMotor();
				LED_RGB_Setting(0);
//				LED_Setting(0);
				isneedinitstage = 1;
				adccnt = 0;
				adc_dis_cnt = 400;
			}
			if(get_motor_level())
			{
				dpdtime = 0;
			}
			//LED_Process		
			LED_Process(system_stage);
			
			//dpd
			if (dpdtime >= DPD_CNT)
			{
				DPD_CNT = 2000;
				Enter_DPD();
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