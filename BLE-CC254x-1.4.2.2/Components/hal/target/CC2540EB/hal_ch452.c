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
#include "hal_i2c.h"

/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/
//----------------------------------------------------------------------------- 
// Includes  
//----------------------------------------------------------------------------- 

/* ------------------------------------------------------------------------------------------------  *
Constants	
* ------------------------------------------------------------------------------------------------	*/	
// I2CWC  
#define I2C_OVR			 BV(7) // 1: GPIO functionality.0: I2C functionality 
#define I2C_SCLPUE 		BV(3) //SCL pin pullup enable 
#define I2C_SDAPUE		 BV(2) //SDA pin pullup enable. 
#define I2C_SCLOE		   BV(1) //SCL pin output enable 


#define I2C_SDAOE			BV(0) //SDA pin output enable	// I2CIO  
#define I2C_SCLD			   BV(1) //SCL data value 
#define I2C_SDAD			   BV(0) //SDA data value	


#define SDA_0	   I2CIO &= ~I2C_SDAD //SDA=0 
#define SDA_1 	 I2CIO |= I2C_SDAD	//SDA=1 
#define SCL_0	   I2CIO &= ~I2C_SCLD //SCL=0 
#define SCL_1 	 I2CIO |= I2C_SCLD	//SCL=1   
#define SDA_IN	  I2CWC &= ~I2C_SDAOE //SDA INPUT 
#define SDA_OUT	  I2CWC |= I2C_SDAOE  //SDA OUTPUT 
#define SCL_IN	   I2CWC &= ~I2C_SCLOE //SCL INPUT 
#define SCL_OUT	   I2CWC |= I2C_SCLOE  //SCL OUTPUT   


#define I2C_GPIO	I2CWC = 0x80; //1: I2C GPIO


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

static void CH452_I2C_NACK()
{
	SDA_OUT;
	SDA_1;
	UDelay(1);
	SCL_1;
	UDelay(1);
	SCL_0;
}

static void CH452_I2C_ACK()
{
	SDA_OUT;
	SDA_0;
	UDelay(1);
	SCL_1;
	UDelay(1);
	SCL_0;
}



static void CH452_I2C_Start(void)
{
	SDA_OUT;
	SCL_OUT;
	SDA_1;
	SCL_1;
	UDelay(1);
	SDA_0;
	UDelay(1);
	SCL_0;
	UDelay(1);
}

static void CH452_I2C_Stop(void)
{
	SDA_OUT;
	SDA_0;
	UDelay(1);
	SCL_1;
	UDelay(1);
	SDA_1;
	UDelay(1);
	SDA_IN;
	SCL_IN;
}

static bool check_ack(void)
{
	bool ack_flag;
	SDA_IN;
	UDelay(1);
	SCL_1;
	UDelay(1);
	if((I2CIO & I2C_SDAD) == 1)
	{ //if(SDA==1)
		ack_flag=0;   //NACK  error
	}
	else
	{
		ack_flag = 1;   //ACK   OK
	}
	SCL_0;  //read 
	return ack_flag;
}



static void CH452_I2C_WriteByte(uint8 dat)
{
	uint8 i;
	uint8 data = dat;
	SDA_OUT;
	for(i=0;i!=8;i++)  
	{
		if(data&0x80) {SDA_1;}
		else {SDA_0;}
		UDelay(1);
		SCL_1;
		data<<=1;
		UDelay(2);
		SCL_0;
		UDelay(1);
	}
	SDA_IN;  
	SDA_1; 
	UDelay(1);
	SCL_1;  // ????
	UDelay(2);
	SCL_0;
	UDelay(1);

}




///////////////////////////////////////////////////////////////////////////////////////////////////////
void HalCH452Write( uint16 data)
{
	CH452_I2C_Start();			  
	CH452_I2C_WriteByte((unsigned char)(data>>7)&CH452_I2C_MASK|CH452_I2C_ADDR1);  // CH452?ADDR=1?(??)
	CH452_I2C_WriteByte((unsigned char)data);	 
	CH452_I2C_Stop(); 
	UDelay(10);
}

uint8 HalCH452Read( )
{
}

void HalCH452Init()
{
	//HalI2CInit(0xAA,i2cClock_33KHZ);
	I2C_GPIO;
	HalCH452Write(CH452_SYSON1);	//?????,??SDA????????,???????CH452_SYSON2W(0x04,0x23)
	HalCH452Write(CH452_BCD);   // BCD??,8????

}


