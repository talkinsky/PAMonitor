/******************************************************************************

 @file  hal_sensor.c

 @brief This file contains the interface to the HAL SENSOR Service.

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

#include "osal.h"
#include "hal_board.h"
#include "hal_adc.h"

#include <string.h>

#include "hal_6812.h"



/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/
#define SENSOR6812_STATUS_OFF			0X00
#define SENSOR6812_STATUS_PREPARE		0X01
#define SENSOR6812_STATUS_RUNING		0X02


#define GAS_SENSOR6812_INIT_TIME_VALID	120000   //2min
#define GAS_SENSOR6812_READ_VALUE_VALID	500      //500ms


/***************************************************************************************************
 *                                           GLOBAL VARIABLES
 ***************************************************************************************************/
static uint8 g_Sensor6812_Status = SENSOR6812_STATUS_OFF;
static GASSenor6812ValueMonitorCBs_t *p_Gas_Sensor6812_Monitor_AppCBs = NULL;


/***************************************************************************************************
 *                                            LOCAL FUNCTION
 ***************************************************************************************************/

void HalSensor6812OnOff (uint8 sensor, uint8 mode);

/***************************************************************************************************
 *                                            FUNCTIONS - API
 ***************************************************************************************************/

/***************************************************************************************************
 * @fn      HalLedInit
 *
 * @brief   Initialize LED Service
 *
 * @param   init - pointer to void that contains the initialized value
 *
 * @return  None
 ***************************************************************************************************/
void HalSensor6812Init (void)
{

	// Set LED GPIOs to outputs.
	POWER_6812_DDR |= POWER_6812_BV;   //power control pin as output

	HalSensor6812OnOff(HAL_SENSOR6812_POWER_ALL, HAL_SENSOR6812_POWER_OFF);  // Initialize all power off

	P0SEL |= 0x81;
	P0DIR &= ~0x81;
	APCFG = 0x81;
	HalAdcInit( );
}


void HalSensor6812Enable( uint8 enable )
{
	if(enable)
	{
		HalSensor6812OnOff(HAL_SENSOR6812_POWER_ALL, HAL_SENSOR6812_POWER_ON);  // Initialize all sensor power on
		osal_start_timerEx(Hal_TaskID, HAL_GAS_SENSOR_READ_EVENT, GAS_SENSOR6812_INIT_TIME_VALID);   /* first time need a long time to init sensor*/
		g_Sensor6812_Status = SENSOR6812_STATUS_PREPARE;
	}
	else
	{
		HalSensor6812OnOff(HAL_SENSOR6812_POWER_ALL, HAL_SENSOR6812_POWER_OFF);  // Initialize all sensor power on
		g_Sensor6812_Status = SENSOR6812_STATUS_OFF;
		osal_stop_timerEx(Hal_TaskID, HAL_GAS_SENSOR_READ_EVENT);   /* Schedule event */
		
	}
}


/***************************************************************************************************
 * @fn      HalLedOnOff
 *
 * @brief   Turns specified LED ON or OFF
 *
 * @param   leds - LED bit mask
 *          mode - LED_ON,LED_OFF,
 *
 * @return  none
 ***************************************************************************************************/
void HalSensor6812OnOff (uint8 sensors, uint8 mode)
{
  if (sensors & HAL_SENSOR6812_POWER)
  {
    if (mode == HAL_SENSOR6812_POWER_ON)
    {
      HAL_TURN_ON_POWER_6812();
    }
    else
    {
      HAL_TURN_OFF_POWER_6812();
    }
  }
}


/***************************************************************************************************
 * @fn      HalSensorGetState
 *
 * @brief   Dim LED2 - Dim (set level) of LED2
 *
 * @param   none
 *
 * @return  led state
 ***************************************************************************************************/
uint8 HalSensor6812GetState ()
{
  return g_Sensor6812_Status;
}


uint8 HalSensor6812Calibration( void )
{
	return 0;
}


uint16 global_gas = 0;
uint8 buf[3];

void HalGasSensor6812Update( void )
{
	uint8 next;
	uint16 res = 0;
	APCFG = 0x01;
	if(g_Sensor6812_Status == SENSOR6812_STATUS_OFF)
	{
		return;
	}
	g_Sensor6812_Status = SENSOR6812_STATUS_RUNING;
	res = HalAdcRead(HAL_ADC_CHANNEL_0,HAL_ADC_RESOLUTION_12);
	global_gas = res;
	res = (res*33) >> 11;
     res = res*3;
     buf[0] = res/10 + '0';
     buf[1] = '.';
     buf[2] =res%10 + '0';
	 if ( p_Gas_Sensor6812_Monitor_AppCBs && p_Gas_Sensor6812_Monitor_AppCBs->pfnGASSensor6812ValueMonitor )
	   {
		 p_Gas_Sensor6812_Monitor_AppCBs->pfnGASSensor6812ValueMonitor( global_gas );  
	   }
	 

	osal_start_timerEx(Hal_TaskID, HAL_GAS_SENSOR_READ_EVENT, GAS_SENSOR6812_READ_VALUE_VALID);
}



bStatus_t HalGasSensor6812RegisterCallback(GASSenor6812ValueMonitorCBs_t *appCallbacks )

{
  if ( appCallbacks )
  {
    p_Gas_Sensor6812_Monitor_AppCBs = appCallbacks;
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


