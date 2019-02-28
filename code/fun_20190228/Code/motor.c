#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "dataflash.h"
#include "motor.h"


unsigned char Motor_Level;
extern bit BIT_TMP;
bit isStartMotor;
bit isMaxPWM;
unsigned int cur_Motor_PWM;
unsigned char Motor_Wakeup_cnt;
unsigned char Motor_done_cnt;

void MOTOR_FG_PinInterrupt_ISR (void)
{
	Motor_done_cnt = 0;
}


void InitPWM(void)
{
	Motor_Level = 0;
//    set_PWMRUN;	
	P12_PushPull_Mode;
	P12 = 1;
	isStartMotor = 0;
	Motor_done_cnt = 0;
	
	P06_PushPull_Mode;
	P06 = 0;			//CW
	P04_PushPull_Mode;	//motor power
	P04 = 0;
}

void TurnOffMotor(void)
{
		clr_EPI;
		//turn off
		Motor_Level = 0;
		PWM0_P12_OUTPUT_DISABLE;
		set_SFRPAGE;
		PWM0H = 0x00;
		PWM0L = 0x00;	
		clr_SFRPAGE;
		set_LOAD;
		clr_CLRPWM;
		//dispower
		P12 = 0;	
		isStartMotor = 0;
		Motor_done_cnt = 0;
	
		P04 = 0;
		P06 = 0;
		P12_PushPull_Mode;
		P12 = 1;
}
/*
unsigned Change_Motor_PWM(void)
{
	if (Motor_Level == 0)
	{
		//enable FG ext interrupt
		isStartMotor = 1;
		PICON = 0x05;	//port1
		PINEN  = 0x00;
		PIPEN = 0x10;	//IO 4
		set_EPI;							// Enable pin interrupt
		
		Motor_Level = 1;
		//pwm High
		P12 = 0;
		P04 = 1;
//		set_SFRPAGE;
//		PWM0H = 0x00;						
//		PWM0L = 0xA6;	
//		clr_SFRPAGE;
//		set_LOAD;
//		set_PWMRUN;				
	}
	else if (Motor_Level == 1)
	{
		Motor_Level = 2;

		PWM0_P12_OUTPUT_ENABLE;
		
		PWM_IMDEPENDENT_MODE;
		PWM_CLOCK_FSYS;
		PWMPH = 0x03;
		PWMPL = 0xE7;						//0x3E7 = 16KHZ,	0x290=24.46khz
		set_SFRPAGE;						//PWM4 and PWM5 duty seting is in SFP page 1
		PWM0H = 0x01;						
		PWM0L = 0xF3;
		clr_SFRPAGE;				
		set_LOAD;
		
		//pwm mid
		set_SFRPAGE;
		PWM0H = 0x00;						
		PWM0L = 0xF0;	
		clr_SFRPAGE;
		set_LOAD;
		set_PWMRUN;			
	}
	else if (Motor_Level == 2)
	{
		Motor_Level = 4;
		//pwm low		
		set_SFRPAGE;
		PWM0H = 0x01;						
		PWM0L = 0x80;	
		clr_SFRPAGE;
		set_LOAD;
		set_PWMRUN;		
	}

	return Motor_Level;
}*/

unsigned Change_Motor_PWM(void)
{
	if (Motor_Level == 0)
	{
		P06 = 1;
		isStartMotor = 1;
		PICON = 0x05;	//port1
		PINEN  = 0x00;
		PIPEN = 0x10;	//IO 4
		set_EPI;							// Enable pin interrupt		
		
		Motor_Level = 1;
		P04 = 1;
		P12_Quasi_Mode;
		//enable PWM0
		PWM0_P12_OUTPUT_ENABLE;
		
		PWM_IMDEPENDENT_MODE;
		PWM_CLOCK_FSYS;
		PWMPH = 0x03;
		PWMPL = 0xE7;						//0x3E7 = 16KHZ,	0x290=24.46khz
		set_SFRPAGE;						//PWM4 and PWM5 duty seting is in SFP page 1
		PWM0H = 0x01;						
		PWM0L = 0xF3;
		clr_SFRPAGE;				
		set_LOAD;

		//pwm low		
		set_SFRPAGE;
		PWM0H = 0x01;						
		PWM0L = 0x80;	
		clr_SFRPAGE;
		set_LOAD;
		set_PWMRUN;	
		cur_Motor_PWM = 0x180;
		Motor_Wakeup_cnt = 0;
		isMaxPWM = 0;
	}
	else if (Motor_Level == 1)
	{
		Motor_Level = 2;
		//pwm mid
//		set_SFRPAGE;
//		PWM0H = 0x00;
//		PWM0L = 0xF0;	
//		clr_SFRPAGE;
//		set_LOAD;
//		set_PWMRUN;	
		//enable PWM0
		PWM0_P12_OUTPUT_ENABLE;
		
		PWM_IMDEPENDENT_MODE;
		PWM_CLOCK_FSYS;
		PWMPH = 0x03;
		PWMPL = 0xE7;						//0x3E7 = 16KHZ,	0x290=24.46khz
		set_SFRPAGE;						//PWM4 and PWM5 duty seting is in SFP page 1
		PWM0H = 0x01;						
		PWM0L = 0xF3;
		clr_SFRPAGE;				
		set_LOAD;

		//pwm low		
		set_SFRPAGE;
		PWM0H = 0x00;						
		PWM0L = 0xF0;	
		clr_SFRPAGE;
		set_LOAD;
		set_PWMRUN;		
	}
	else if (Motor_Level == 2)
	{
		Motor_Level = 4;

		//pwm low		
		set_SFRPAGE;
		PWM0H = 0x01;						
		PWM0L = 0x80;	
		clr_SFRPAGE;
		set_LOAD;
		set_PWMRUN;

	}
	else if (Motor_Level == 4)
	{
		TurnOffMotor();
	}
	return Motor_Level;
}

unsigned char get_motor_level(void)
{
	return Motor_Level;
}

unsigned char check_motor_done(void)
{
	if (isStartMotor)
	{
		if (Motor_done_cnt > 10)
			return 1;
	}
	if ((Motor_Level == 1)&&(isMaxPWM == 0))
	{
		Motor_Wakeup_cnt++;
		if (Motor_Wakeup_cnt >= 20)
		{
			Motor_Wakeup_cnt = 0;
			cur_Motor_PWM-=6;
			if (cur_Motor_PWM >= 10)
			{
				cur_Motor_PWM -= 6;
				set_SFRPAGE;
				PWM0H = cur_Motor_PWM>>8;
				PWM0L = cur_Motor_PWM&0xFF;	
				clr_SFRPAGE;
				set_LOAD;
				clr_CLRPWM;
			}
			else
			{
				isMaxPWM = 1;
				PWM0_P12_OUTPUT_DISABLE;
				set_SFRPAGE;
				PWM0H = 0x00;
				PWM0L = 0x00;	
				clr_SFRPAGE;
				set_LOAD;
				clr_CLRPWM;
				
				//High
				P12 = 0;
			}
		
		}
	}
	return 0;
}
