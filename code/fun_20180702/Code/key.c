#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "dataflash.h"
#include "key.h"

#define Fun_KEY_INPUT	P17_Input_Mode
#define Fun_KEY_P17		P17
#define Safety_KEY_INPUT	P30_Input_Mode
#define Safety_KEY		P30
#define Charging_KEY_INPUT	P05_Input_Mode
#define Charging_KEY	P05

unsigned char curKey,preKey;
unsigned int debounce;
unsigned char Safety_key_debounce;
unsigned char Charging_key_debounce;
extern char is_5ms_Flag;

extern bit BIT_TMP;


void KeyInit(void)
{
	curKey = 0;
	preKey = 0;
	debounce = 0;
	Safety_key_debounce = 0;
	Charging_key_debounce = 0;
	Fun_KEY_INPUT;
	Safety_KEY_INPUT;
	Charging_KEY_INPUT;
}

char KeyScan(void)
{
	unsigned char keystatus ;
	keystatus = 0;
	if (is_5ms_Flag)
	{
		is_5ms_Flag = 0;
		curKey = Fun_KEY_P17;
		if (curKey != preKey)
		{
			preKey = curKey;
			if (debounce>=600)
			{
				keystatus = 0x08;
			}
			else if (debounce > 10)
			{
				keystatus = 1;
			}	
			debounce = 0;
		}
		else if(curKey == 0)
		{
			debounce++;
			if (debounce >= 600)
				debounce = 600;
		}
		
		if (Safety_KEY)
		{
			Safety_key_debounce = 0;
		}
		else
		{
			Safety_key_debounce++;
			if (Safety_key_debounce > 20)
			{
				Safety_key_debounce = 20;
				keystatus |= 0x02;
			}
		}
		
		if (Charging_KEY)
		{
			Charging_key_debounce = 0;
		}
		else
		{
			Charging_key_debounce++;
			if (Charging_key_debounce > 20)
			{
				Charging_key_debounce = 20;
				keystatus |= 0x04;
			}
		}
	}
	return keystatus;
}

void EnterDPD(void)
{
	//…Ë÷√P17∞¥º¸ªΩ–—
    P17_Input_Mode;
	set_P0S_7;
	Enable_INT_Port1;
	Enable_BIT7_FallEdge_Trig;
    set_EPI;							// Enable pin interrupt
    set_EA;			
	//dpd
	set_PD;
	
	PICON  = PICON & 0xFE;
}