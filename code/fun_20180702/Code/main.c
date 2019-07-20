
#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "dataflash.h"
#include "key.h"
#include "motor.h"
#include "led.h"
#include "Delay.h"

#define TH0_INIT        (65536-6667)/256 //5.0ms@XTAL=12MHz, Period = (10.85/2) ms@XTAL=22.1184MHz
#define TL0_INIT        (65536-6667)%256
#define TH1_INIT        0xE0 //2.5ms@XTAL=12MHz, Period = (5.425/2) ms@XTAL=22.1184MHz
#define TL1_INIT        0x00

bit BIT_TMP;
#define ADC_CNT	8

extern unsigned char Motor_Level;
extern unsigned int led_display_time;
/********************************/
#define Stage_A		1
#define Stage_B		2
#define Stage_C		4
#define Stage_D		8

bit isneedinitstage;
bit isneedinitsys;
bit ischarging;
bit isstartsystem;
bit isWaitTurnOffCharging;
bit isfirstenterdpd;
bit isenableLED;

unsigned char is_5ms_Flag;
unsigned char system_stage;

unsigned char batlevel_led_value;
unsigned char adc_pre_cnt;
unsigned char adccnt;
unsigned char batlevel;
unsigned char adc_dis_cnt;
unsigned char adcchangecnt=0;

unsigned int startADC_cnt;
unsigned int first_run_times;
unsigned int adc_wait_cnt;
unsigned int dpdtime;
unsigned int batlevelledtimeout;
unsigned int adcvalue;
unsigned int adc[ADC_CNT];
int testtemp;

#define UART_TEST	1
#define WDT_ENABLE	1

#if UART_TEST

float bat_val;
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

void Timer1_Delay100us(unsigned long cnt)
{
    clr_T1M;																		//T1M=0, Timer1 Clock = Fsys/12
    TMOD |= 0x10;																//Timer1 is 16-bit mode
    set_TR1;																		//Start Timer1
    while (cnt != 0)
    {
        TL1 = LOBYTE(TIMER_DIV12_VALUE_100us);		//Find  define in "Function_define.h" "TIMER VALUE"
        TH1 = HIBYTE(TIMER_DIV12_VALUE_100us);
        while (TF1 != 1);												//Check Timer1 Time-Out Flag
        clr_TF1;
        cnt --;
    }
    clr_TR1;                               			//Stop Timer1
}
#endif
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
	adc_wait_cnt++;
	if (first_run_times < 2000) 
		first_run_times++;
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

unsigned int getadcvalue(unsigned int * buf)
{
	unsigned char i;
	unsigned int adcvalue_temp;
	adcvalue_temp = 0;
	for(i=0;i<ADC_CNT;i++)
	{
		adcvalue_temp += buf[i];
	}
	adcvalue_temp >>=3;
	return adcvalue_temp;
}

unsigned int charging_bat_feedback(unsigned int cur_adc,unsigned char pwm)
{
	unsigned int temp;
	unsigned int temp_pwm;
	
//	if (pwm == 0)
	{
		if (batlevel < 3)
		{
			temp = 180;
		}
		else if (batlevel <5)
		{
			temp = 120;
		}
		else 
		{
			temp = 60;
		}
	}
//	else
//	{
//		if (pwm == 1)
//		{
//			
//		}
//		else if (pwm == 2)
//		{
//		}
//		else if (pwm == 4)
//		{
//		}
//		
//	}
	
	return temp;
}

unsigned char getbatlevel(unsigned char adc_delay)
{
	unsigned char templevel;
	unsigned char cur_pwm_level;
	unsigned int adcvaluetemp;
	if (startADC_cnt > adc_delay)
	{
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
		startADC_cnt = 0;
		adc_wait_cnt=0;
		if (adccnt >= ADC_CNT)
		{
			adccnt = 0;
			if(adc_dis_cnt > 0)
			{
				return 0;
			}
			adcvaluetemp = getadcvalue(adc);
			
			if (ischarging)
			{
				if (Motor_Level == 0)
				{										
					if (adcvaluetemp < 3000)
						adcvaluetemp = adcvaluetemp-50;
					else if (adcvaluetemp < 3200)
						adcvaluetemp = adcvaluetemp-20;
				}
			}
			else if (Motor_Level)
			{
				if (Motor_Level == 1)
				{
					cur_pwm_level = cur_pwm();
					if (cur_pwm_level == 1)
						adcvaluetemp = adcvaluetemp + 60;
					else if (cur_pwm_level == 2)
						adcvaluetemp = adcvaluetemp + 80;
					else
						adcvaluetemp = adcvaluetemp + 120;
				}
				else if (Motor_Level == 2)
				{
					adcvaluetemp = adcvaluetemp + 80;
				}
				else if (Motor_Level == 4)
				{
					adcvaluetemp = adcvaluetemp + 60;
				}
			}
			
			#if UART_TEST
			printf("ap=%d\n",adcvaluetemp);//adcvaluetemp
			#endif
			if (isstartsystem == 1)
			{
				if (ischarging)
				{
					if (adcvaluetemp > (adcvalue + 8))
					{
						adcchangecnt++;
						if (adcchangecnt > 3)
						{
							adcchangecnt = 0;
							adcvalue+=(adcvaluetemp - adcvalue)>>1;
						}
					}
					else if (adcvaluetemp < (adcvalue-8))
					{
						adcchangecnt = 0;
						adcvalue-=5;
					}
					else
					{
						adcchangecnt = 0;
					}
				}
				else
				{
					if (adcvaluetemp > (adcvalue + 8))
					{
						adcchangecnt++;
						if (adcchangecnt > 5)
						{
							adcchangecnt = 0;
							adcvalue+=5;
						}
					}
					else if (adcvaluetemp < (adcvalue-8))
					{
						adcchangecnt = 0;
						adcvalue-=5;
					}
					else
					{
						adcchangecnt = 0;
					}
				}
				if (adcvalue == 0)
					adcvalue = adcvaluetemp;				
			}
			#if UART_TEST
			printf("ae=%d\n",adcvalue);//adcvalue
//			bat_val = ((float)adcvalue) *3.34 /4096.0 * 3.0;
//			printf("bat=%0.4f\n",bat_val);
			#endif
			if (adcvalue > 3300)		//100% 8.2
			{
				templevel = 6;
				if(ischarging == 1)
				{
					isWaitTurnOffCharging = 1;
				}
			}
			else if (adcvalue > 3180)	//>75%	7.9v	0xCA0	
			{
				templevel = 5;
			}
			else if (adcvalue > 3050)	//>50%	7.7v	0xC40
			{
				templevel = 4;
			}
			else if (adcvalue > 2980)	//>25%	7.5		0xBE0
			{
				templevel = 3;
			}
			else if (adcvalue > 2830)	//10%	7.2v	0xBA0
			{
				templevel = 2;
			}
			else
			{
				templevel = 1;
			}
			if (isWaitTurnOffCharging == 0)
			{
				if (batlevel != templevel)
					adc_pre_cnt++;
				else
					adc_pre_cnt = 0;
				#if UART_TEST				
				testtemp = batlevel;
				printf("pl=%d\n",testtemp);//power level
				testtemp = templevel;
				printf("tl=%d\n",testtemp);//templevel
				testtemp = adc_pre_cnt;
				printf("at=%d\n",testtemp);//adc_pre_cnt
				#endif
				if (adc_pre_cnt > 5)
				{
					adc_pre_cnt = 0;
					if (ischarging)
					{
						if (templevel > batlevel)
						{
							batlevel = templevel;
						}
					}
					else if(Motor_Level != 0)
					{
						if (templevel < batlevel)
						{
							batlevel = templevel;
						}
					}
					else
					{
						batlevel = templevel;
					}
				}
			}
			else
			{
				batlevel = 6;
			}
			return 1;
		}
	}	
	return 0;
}

#if WDT_ENABLE
void Enable_WDT_Reset_Config(void)
{
	#if 1
	set_IAPEN;
    IAPAL = 0x04;
    IAPAH = 0x00;
    IAPFD = 0x0F;
    IAPCN = 0xE1;
    set_CFUEN;
    set_IAPGO;                                  //trigger IAP
	while((CHPCON&SET_BIT6)==SET_BIT6);          //check IAPFF (CHPCON.6)
    clr_CFUEN;
    clr_IAPEN;
	#else
	TA=0xAA;TA=0x55;WDCON=0x07;						//Setting WDT prescale 
	set_WDCLR;														//Clear WDT timer
	while((WDCON|~SET_BIT6)==0xFF);				//confirm WDT clear is ok before into power down mode
	EA = 1;
	set_WDTR;
	#endif
}

void Disable_WDT_Reset_Config(void)
{
	unsigned char cf0,cf1,cf2,cf3,cf4;
#define     CFG_READ            0xC0
#define     CFG_ERASE           0xE2
#define     CFG_BYTE_PROGRAM    0xE1	
	  set_IAPEN;
    IAPAL = 0x00;
    IAPAH = 0x00;
    IAPCN = CFG_READ;
    set_IAPGO;                                  //Storage CONFIG0 data
		cf0 = IAPFD;
		IAPAL = 0x01;
		set_IAPGO;                                  //Storage CONFIG1 data
		cf1 = IAPFD;
		IAPAL = 0x02;
	  set_IAPGO;                                  //Storage CONFIG2 data
		cf2 = IAPFD;
		IAPAL = 0x03;
	  set_IAPGO;                                  //Storage CONFIG3 data
		cf3 = IAPFD;
		IAPAL = 0x04;
	  set_IAPGO;                                  //Storage CONFIG4 data
		cf4 = IAPFD;
		cf4 |= 0xF0;																//Moidfy Storage CONFIG4 data disable WDT reset
		
		set_CFUEN;	
		IAPCN = CFG_ERASE;													//Erase CONFIG all
		IAPAH = 0x00;
		IAPAL = 0x00;
		IAPFD = 0xFF;
		set_IAPGO;
		
		IAPCN = CFG_BYTE_PROGRAM;										//Write CONFIG
		IAPFD = cf0;
		set_IAPGO;
		IAPAL = 0x01;
		IAPFD = cf1;
		set_IAPGO;
		IAPAL = 0x02;
		IAPFD = cf2;
		set_IAPGO;
		IAPAL = 0x03;
		IAPFD = cf3;
		set_IAPGO;
		IAPAL = 0x04;
		IAPFD = cf4;
		set_IAPGO;

    clr_CFUEN;
    clr_IAPEN;
}
#endif
void Enter_DPD(void)
{
	P06 = 1;	//enable chargine
	dpdtime = 0;

	Set_All_GPIO_Quasi_Mode;
	clr_ADCEN;

	TurnOffMotor();
	LED_WHITE_Setting(0);
	LED_RGB_Setting(0);
	DeInit_LED();
	
	P05_Input_Mode;	
	while(P05 == 0);
	#if UART_TEST
	printf("P05\n");
	#endif
//	set_P1SR_5;
//	Enable_INT_Port0;
//	Enable_BIT5_FallEdge_Trig;
	PICON = 0x40;	//port0
	PINEN  = 0x20;
	PIPEN = 0x00;	//IO 4	
	
	set_EPI;
	
	set_EA;
	
	P17_Input_Mode;
	#if UART_TEST
	printf("P07\n");
	#endif
	set_P1S_7;
	set_EX1;

	#if WDT_ENABLE
	//clear wdt
	Disable_WDT_Reset_Config();
	#endif
	
	set_PD;

	PICON  = 0;
				
	clr_EX0;
	clr_EX1;
	clr_EPI;
	isneedinitsys = 1;	
}


void SysInit(void)
{	
	#if UART_TEST
	InitialUART1_Timer3(115200);
	TI_1 = 1;
	#endif
	PICON  = 0;
				
	clr_EX0;
	clr_EX1;
	clr_EPI;
	
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
	P06_PushPull_Mode;
	dpdtime = 0;	
	if (isstartsystem != 1)
	{		
		P06 = 0;	//disable charging
		ischarging = 0;
		while(dpdtime<300)
		{
			getbatlevel(0);
		}
		isstartsystem = 1;
	}
//	else
//	{
//		while(dpdtime<60)
//		{
//			if (P05 == 0)
//				ischarging = 1;
//			getbatlevel(0);
//		}
//	}
	P06 = 1;	//enable chargine
	dpdtime = 0;
	led_display_time = 0;
	startADC_cnt = 0;
	if (isfirstenterdpd == 1)
		adc_dis_cnt = 200;
	isWaitTurnOffCharging = 0;
	
	#if UART_TEST
	printf("sysinit over\n");
	testtemp = batlevel;
	printf("power level=%d\n",testtemp);
	if (WDCON & 0x08)
	{
		printf("wdt reset\n");
		clr_WDTRF;
	}
	#endif
	#if WDT_ENABLE
	Enable_WDT_Reset_Config();
	#endif
	
	isenableLED = 0;
}


void main(void)
{
	unsigned char keystatus;
	Set_All_GPIO_Quasi_Mode;
	isneedinitsys = 1;
	isstartsystem = 0;
	isfirstenterdpd = 0;
	first_run_times= 0;
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
					#if UART_TEST
					printf("init stage A\n");
					#endif
					isneedinitstage = 0;
					//init stage A
					TurnOffMotor();
					LED_Setting(0);
					isenableLED = 0;
					LED_RGB_Setting(0);
					dpdtime = 0;
					ischarging = 0;
					adccnt = 0;
					
					if (isfirstenterdpd == 1)
					adc_dis_cnt = 200;
				}
				if (keystatus & 0x01)//key
				{
					//display power as LED
					batlevelledtimeout = 600;
					LED_Setting(system_stage);
					isenableLED = 1;
					dpdtime = 0;
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
					#if UART_TEST
					printf("init stage B\n");
					#endif
					isneedinitstage = 0;
					//init stage B
					dpdtime = 0;
					ischarging = 0;
					//check motor level				
					LED_RGB_Setting(Motor_Level);
					if (Motor_Level == 0)
					{
						LED_Setting(0);
						isenableLED = 0;
					}
					else
					{
						LED_Setting(system_stage);
						isenableLED = 1;
					}
					led_display_time = 600;
					adccnt = 0;
					if (isfirstenterdpd == 1)
					adc_dis_cnt = 200;
				}
				if (keystatus & 0x01)//key
				{
					//change pwm
//					if (adcvalue > 2400)
					{
						Change_Motor_PWM();
						#if UART_TEST
						testtemp = Motor_Level;
						printf("motor level=%d\n",testtemp);
						#endif
						LED_RGB_Setting(Motor_Level);
						if (Motor_Level == 0)
						{
							LED_Setting(0);
							isenableLED = 0;
						}
						else
						{
							LED_Setting(system_stage);
							isenableLED = 1;
						}
						adccnt = 0;
						if (isfirstenterdpd == 1)
							adc_dis_cnt = 200;
						dpdtime = 0;	
					}					
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
					isenableLED = 0;
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
					isenableLED = 0;
					adccnt = 0;
					if (isfirstenterdpd == 1)
						adc_dis_cnt = 200;
					dpdtime = 0;
				}
			}
			else if (system_stage == Stage_C)	//充电中
			{
				if (isneedinitstage == 1)
				{
					#if UART_TEST
					printf("init stage C\n");
					#endif
					isneedinitstage = 0;
					//init stage C	
					dpdtime = 0;
					ischarging = 1;					
					TurnOffMotor();
					LED_RGB_Setting(0);
					LED_Setting(system_stage);
					isenableLED = 1;
					led_display_time = 600;
					adccnt = 0;
					if (isfirstenterdpd == 1)
					adc_dis_cnt = 200;
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
					isenableLED = 0;
					isneedinitstage = 1;
				}
			}
			else if (system_stage == Stage_D)	//充电和闭合安全开关
			{
				if (isneedinitstage == 1)
				{
					#if UART_TEST
					printf("init stage D\n");
					#endif
					isneedinitstage = 0;
					//init stage D
					dpdtime = 0;
					ischarging = 1;
					//check motor level				
					LED_RGB_Setting(Motor_Level);
					LED_Setting(system_stage);
					isenableLED = 1;
					adccnt = 0;
					
					if (isfirstenterdpd == 1)
					adc_dis_cnt = 200;
				}
				if (keystatus & 0x01)//key
				{
					//change pwm
//					if (adcvalue > 2400)
					{
						Change_Motor_PWM();
						#if UART_TEST
						testtemp = Motor_Level;
						printf("motor level=%d\n",testtemp);
						#endif
						LED_RGB_Setting(Motor_Level);
						adccnt = 0;
						if (isfirstenterdpd == 1)
							adc_dis_cnt = 200;
						dpdtime = 0;
					}
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
					LED_Setting(0);
					isenableLED = 0;
				}
				
				if (keystatus & 0x04)//charging
				{
					dpdtime = 0;
				}
				else
				{
					system_stage = Stage_B;
					ischarging = 0;
					LED_RGB_Setting(Motor_Level);
					LED_Setting(system_stage);	
					isenableLED = 1;
					isneedinitstage = 1;
				}
				
				if (keystatus & 0x08)
				{
					//turn off pwm
					TurnOffMotor();
					LED_RGB_Setting(0);
					LED_Setting(system_stage);	
					isenableLED = 1;
					adccnt = 0;
					if (isfirstenterdpd == 1)
						adc_dis_cnt = 200;
					dpdtime = 0;
				}
			}
			
			//ADC process
//			if ((ischarging == 1)||(get_motor_level()!=0))
//				getbatlevel(200);
//			else
//				getbatlevel(2);
			if (first_run_times < 1000)
			{
				P06 = 0;
			}
			//ADC Porcess
			if ((ischarging == 1) && (first_run_times > 1000))
			{
				first_run_times = 1001;
				if (adc_wait_cnt == 1000)
				{
					//disable charging
					adc_wait_cnt = 1001;
					P06 = 0;	//disable chargine
				}
				else if (adc_wait_cnt >= 1050)
				{
					getbatlevel(0);
					P06 = 1;	//enable chargine
					adc_wait_cnt = 0;
				}
			}
			else if (get_motor_level()!=0)
			{
				getbatlevel(80);
			}
			else
			{
				getbatlevel(4);
			}
			
			if (adc_dis_cnt > 0)
				adc_dis_cnt--;
			
			if (isWaitTurnOffCharging == 1)
			{
				if (keystatus & 0x04)
				{
					batlevel = 6;
				}
				else
				{
					isWaitTurnOffCharging = 0;
				}
			}
			//pwm rate
			if (check_motor_done())
			{
				#if UART_TEST
				printf("motor-locked,%d\n",__LINE__);
				#endif
				//turn off pwm
				TurnOffMotor();
				LED_RGB_Setting(0);
				isneedinitstage = 1;
				adccnt = 0;
				if (isfirstenterdpd == 1)
				adc_dis_cnt = 200;
			}
			if(Motor_Level)
			{
				dpdtime = 0;
			}
			//LED_Process		
			if (isenableLED)
			LED_Process(system_stage);
			
			//dpd
			if (dpdtime >= 2000)
			{
				first_run_times = 0;
				isfirstenterdpd = 1;
				#if UART_TEST
				printf("enter dpd\n");
				#endif
				isfirstenterdpd = 1;
				Enter_DPD();
			}
//			P13 = 1;
			#if UART_TEST
//			if (dpdtime > 1000)
//			{
//				printf("test wdt reset\n");
//				while(1);
//			}
			#endif
			#if WDT_ENABLE
			//clr wdt
			set_WDCLR;
			#endif
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