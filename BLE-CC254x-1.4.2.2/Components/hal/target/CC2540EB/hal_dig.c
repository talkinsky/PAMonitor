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
 PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
enum{
	DIG_DISP_OFF,
	DIG_DISP_ON,
	DIG_DISP_FLASH_OFF,
	DIG_DISP_FLASH_ON,
};

/***************************************************************************************************
 *                                           GLOBAL VARIABLES
 ***************************************************************************************************/
static uint8 HalDigState = DIG_DISP_OFF;			   // Dig display
static uint8 HalDigPreState;



/***************************************************************************************************
 *                                            LOCAL FUNCTION
 ***************************************************************************************************/



/***************************************************************************************************
 *                                            FUNCTIONS - API
 ***************************************************************************************************/



/***************************************************************************************************
 * @fn      HalDigInit
 *
 * @brief   Initialize Dig Board show Service
 *
 * @param   init - pointer to void that contains the initialized value
 *
 * @return  None
 ***************************************************************************************************/
void HalDigInit (void)
{
	// Set LED GPIOs to outputs.

	DISP_INT_DDR |= DISP_INT_BV;
	DISP_RST_DDR |= DISP_RST_BV;
	DISP_RSTI_DDR |= DISP_RSTI_BV;
	

	HAL_SET_DIG_INT_HIGH();
	HAL_SET_DIG_RST_HIGH();
	HAL_SET_DIG_RSTI_LOW();
	HalDigState = DIG_DISP_OFF;
	HalCH452Init();

}

/***************************************************************************************************
 * @fn      HalDigFlash
 *
 * @brief   Turn ON/OFF Dig Flash when alarm occure
 *
 * @param   enable - trun on / off flash mode
 *          
 * @return  None
 ***************************************************************************************************/

uint8 HalDigFlash( uint8 enable )
{
	if(enable)
	{
		HalDigPreState = HalDigState;
		HalDigState = DIG_DISP_FLASH_ON;
	}
	else
	{
		HalDigState = HalDigPreState;
	}
}


/***************************************************************************************************
 * @fn      HalDigShow
 *
 * @brief   Set the number show on Dig board
 *
 * @param   data - data want to show
 *          
 * @return  None
 ***************************************************************************************************/

void HalDigShow( uint8 *data, uint8 len)
{
	uint8 buf = *data;
	uint8 ret;
	HalCH452Write(CH452_DIG7 | 1);
	HalCH452Write(CH452_DIG6 | 2);
	HalCH452Write(CH452_DIG5 | 3);
	HalCH452Write(CH452_DIG4 | 4);
	HalCH452Write(CH452_DIG3 | 5);
	HalCH452Write(CH452_DIG2 | 6);
	HalCH452Write(CH452_DIG1 | 7);
	HalCH452Write(CH452_DIG0 | 8);  // ????8
}


/***************************************************************************************************
* @fn	   HalDigEnterSleep
*
* @brief   put Dig board in sleep mode -------  save the previous data but not show
*
* @param   None
*		   
* @return  None
***************************************************************************************************/

void HalDigEnterSleep( void )
{
}

/***************************************************************************************************
* @fn	   HalDigExitSleep
*
* @brief   put Dig board in running mode -------  restore last data and start show
*
* @param   None
*		   
* @return  None
***************************************************************************************************/

void HalDigExitSleep( void )
{
}



/***************************************************************************************************
* @fn	   HalDigGetState
*
* @brief   return status of Dig board
*
* @param   None
*		   
* @return  uint8  state of Dig Board on/off/flash on /flash off
***************************************************************************************************/

uint8 HalDigGetState ( void )
{
	return HalDigState;
}



