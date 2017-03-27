/******************************************************************************

 @file  hal_led.c

 @brief This file contains the interface to the HAL LED Service.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2006-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:09
 *****************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_dig.h"
#include "osal.h"
#include "hal_board.h"
#include "hal_ch452.h"  

/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/


/***************************************************************************************************
 *                                           GLOBAL VARIABLES
 ***************************************************************************************************/


/***************************************************************************************************
 *                                            LOCAL FUNCTION
 ***************************************************************************************************/



/***************************************************************************************************
 *                                            FUNCTIONS - API
 ***************************************************************************************************/
 
static void  UDelay(uint16 microSecs)  
{  
  while(microSecs--)  
  {  
    /* 32 NOPs == 1 usecs */  
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");  
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");  
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");  
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");  
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");  
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");  
    asm("nop"); asm("nop");  
  }  
}  


static void CH452_SET_SDA_IN()
{
	CH452_SDA_SEL &= ~(CH452_SDA_BV);	/* Set pin function to GPIO */
	CH452_SDA_DDR &= ~(CH452_SDA_BV);	/* Set pin direction to Input */
}

static void CH452_SET_SDA_OUT()
{
	CH452_SDA_SEL &= ~(CH452_SDA_BV);	/* Set pin function to GPIO */
	CH452_SDA_DDR |= (CH452_SDA_BV);	/* Set pin direction to output */
}



#define CH452_SDA_SET HAL_SET_SDA_HIGH()
#define CH452_SDA_CLR HAL_SET_SDA_LOW()
#define CH452_SDA_D_IN CH452_SET_SDA_IN()
#define CH452_SDA_D_OUT CH452_SET_SDA_OUT()
#define CH452_SDA_IN	(CH452_SDA_PORT & CH452_SDA_BV)

#define CH452_SCL_SET HAL_SET_SCL_HIGH()
#define CH452_SCL_CLR HAL_SET_SCL_LOW()




#define DELAY_1US UDelay(1)
static void CH452_Start()
{
	CH452_SDA_SET;
	CH452_SDA_D_IN;   
	CH452_SCL_CLR;	
	do {
		DELAY_1US;  // ????????CH452????
		DELAY_1US;
	} while ( CH452_SDA_IN == 0 );  // CH452???????
	DELAY_1US;
	CH452_SDA_SET;   /*???????????*/
	CH452_SDA_D_OUT;   /* ??SDA????? */
	CH452_SCL_SET;
	DELAY_1US;
	CH452_SDA_CLR;   /*??????*/
	DELAY_1US;      
	CH452_SCL_CLR;   /*??I2C??,????????? */
	DELAY_1US;

}


static void CH452_Stop()
{
	CH452_SDA_CLR;
	CH452_SDA_D_OUT;   /* ??SDA????? */
	DELAY_1US;
	CH452_SCL_SET;
	DELAY_1US;
	CH452_SDA_SET;  /*??I2C??????*/
	DELAY_1US;
	CH452_SDA_D_IN;   /* ??SDA????? */
}

static void CH452_WriteByte(uint8 dat)
{
	uint8 i;
	uint8 data = dat;
	CH452_SDA_D_OUT;   /* ??SDA????? */
	for(i=0;i!=8;i++)  // ??8???
	{
		if(data&0x80) {CH452_SDA_SET;}
		else {CH452_SDA_CLR;}
		DELAY_1US;
		CH452_SCL_SET;
		data<<=1;
		DELAY_1US;
		DELAY_1US;
		CH452_SCL_CLR;
		DELAY_1US;
	}
	CH452_SDA_D_IN;   /* ??SDA????? */
	CH452_SDA_SET;
	DELAY_1US;
	CH452_SCL_SET;  // ????
	DELAY_1US;
	DELAY_1US;
	CH452_SCL_CLR;
	DELAY_1US;

}

static uint8 CH452_ReadByte()
{
	unsigned char dat,i;
	CH452_SDA_SET;
	CH452_SDA_D_IN;   /* ??SDA????? */
	dat=0;
	for(i=0;i!=8;i++)  // ??8???
	{
		CH452_SCL_SET;
		DELAY_1US;
		DELAY_1US;
		dat<<=1;
		if(CH452_SDA_IN) dat++;  // ??1?
		CH452_SCL_CLR;
		DELAY_1US;
//		DELAY_1US;
	}
	CH452_SDA_SET;
	DELAY_1US;
	CH452_SCL_SET;  // ??????
	DELAY_1US;
	DELAY_1US;
	CH452_SCL_CLR;
	DELAY_1US;
	return(dat);
}

void HalCH452Write( uint8 data)
{
	CH452_Start();			   /*????*/
	CH452_WriteByte((unsigned char)(data>>7)&CH452_I2C_MASK|CH452_I2C_ADDR1);  // CH452?ADDR=1?(??)
	CH452_WriteByte((unsigned char)data);	  /*????*/
	CH452_Stop(); 				/*????*/ 

}

uint8 HalCH452Read( )
{
}

void HalCH452Init()
{
	HalCH452Write(CH452_SYSON2);	//?????,??SDA????????,???????CH452_SYSON2W(0x04,0x23)
	HalCH452Write(CH452_BCD);   // BCD??,8????
	HalCH452Write(CH452_DIG7 | 1);
	HalCH452Write(CH452_DIG6 | 2);
	HalCH452Write(CH452_DIG5 | 3);
	HalCH452Write(CH452_DIG4 | 4);
	HalCH452Write(CH452_DIG3 | 5);
	HalCH452Write(CH452_DIG2 | 6);
	HalCH452Write(CH452_DIG1 | 7);
	HalCH452Write(CH452_DIG0 | 8);  // ????8
}


