
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"

#include "gatt.h"

#include "hci.h"

#include "peripheral.h"

#include "gapbondmgr.h"

#include "timer.h"

static uint16 int_count = 0;


void start_timer1()
{
	int_count = 0;
	T1CTL = 0x01; //(3<<2)|(2<<0);  //0x01;  //freq/1  and repeat with 0x0000~0xFFFF
	T1CNTL = 0x00;
	T1CCTL0 = (1<<6)|(7<<3)|(1<<2)|(0<<0);
	T1CC0H = 0xFF;
	T1CC0L = 0xFF;
	IEN1  |= (1<<1);   //enable timer1 interrupt
	//EA = 1;            //enable interrupt controler
}

void stop_timer1()
{
	//EA = 0;
	IEN1 &= 0xFD;
	T1CTL = 0x00;  //freq/1
	T1CNTL = 0x00;
	int_count = 0;

}
#define T1STAT_CHOIF            (1<<0)  //定时器1的通道0状态位  


#pragma vector = T1_VECTOR 
__interrupt void Timer1_ISR(void) 
{ 
	unsigned char flags = T1STAT;  
		
	//通道0  
	if(flags & T1STAT_CHOIF)	
	{
		int_count ++;
	}
}


void get_timer1(uint8 * buf)
{
	buf[0] = HI_UINT16(int_count);
	buf[1] = LO_UINT16(int_count);
	buf[2] = T1CNTH;
	buf[3] = T1CNTL;
}

