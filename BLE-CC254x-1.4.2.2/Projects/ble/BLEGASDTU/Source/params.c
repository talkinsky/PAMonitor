/******************************************************************************

 @file  params.c

 @brief This file contains the store gas sensor param get and set function.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2010-2016, Texas Instruments Incorporated
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
 Release Date: 2016-06-09 06:57:10
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_snv.h"


#include "params.h"



/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
GAS_PARAM_STRUCT g_gas_param;
uint8 g_gas_param_inited = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
 static void GasParamRestore( void );
static void GasParamSave( void );



/***************************************************************************************************/

static void GasParamRestore( void )
{
	uint8 buff[50];
	uint8 ret;
	ret = osal_snv_read(SNV_GAS_PARAM_STORE_ID, sizeof(GAS_PARAM_STRUCT), buff);  
  
    if(ret == NV_OPER_FAILED)   //never saved the param, init it
    {  
        osal_memset(buff, 0x55, sizeof(buff));
		g_gas_param.alarm_threshold = GAS_PARAM_ALARM_THRES;
		g_gas_param.offset_from_zero = GAS_PARAM_OFFSET_ZERO;
		g_gas_param.offset_from_1000 = GAS_PARAM_OFFSET_1000;
		GasParamSave();
    }   
	else
	{
		osal_memcpy((void *)&g_gas_param,(void *)buff,sizeof(GAS_PARAM_STRUCT));
	}
	g_gas_param_inited = 1;
}
static void GasParamSave( void )
{
	uint8 buff[50];
	osal_memcpy((void *)buff,(void *)&g_gas_param,sizeof(GAS_PARAM_STRUCT));
	osal_snv_write(SNV_GAS_PARAM_STORE_ID, sizeof(buff), buff);
}


/***************************************************************************************************/
void gasParamInit( void )
{
	GasParamRestore();
}


void setGasThreashold( uint16 threas )
{
	if(0 == g_gas_param_inited)
	{
		GasParamRestore();
	}
	g_gas_param.alarm_threshold = threas;
	GasParamSave();
}
uint16 getGasThreashold( void )
{
	if(0 == g_gas_param_inited)
	{
		GasParamRestore();
	}
	return g_gas_param.alarm_threshold;
}


/*
 * set and get offset from zero
 */
void setGasOffsetFromZero( uint16 threas )
{
	if(0 == g_gas_param_inited)
	{
		GasParamRestore();
	}
	g_gas_param.offset_from_zero = threas;
	GasParamSave();
}

uint16 getGasOffsetFromZero( void )
{
	if(0 == g_gas_param_inited)
	{
		GasParamRestore();
	}
	return g_gas_param.offset_from_zero;
}


/*
 * set and get offset from 1000
 */
void setGasOffsetFrom1000( uint16 threas )
{
	if(0 == g_gas_param_inited)
	{
		GasParamRestore();
	}
	g_gas_param.offset_from_1000 = threas;
	GasParamSave();
}

uint16 getGasOffsetFrom1000( void )
{
	if(0 == g_gas_param_inited)
	{
		GasParamRestore();
	}
	return g_gas_param.offset_from_1000;
}



