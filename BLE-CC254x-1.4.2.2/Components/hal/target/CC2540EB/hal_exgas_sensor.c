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
#include "hal_exgas_sensor.h"
#include "osal.h"
#include "hal_board.h"
#include "hal_adc.h"

#include <string.h>

#include "npi.h"

//#include "crc.h"



/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/
#define SENSOR_STATUS_OFF			0X00
#define SENSOR_STATUS_PREPARE		0X01
#define SENSOR_STATUS_RUNING		0X02


#define GAS_SENSOR_INIT_TIME_VALID	120000   //2min
#define GAS_SENSOR_READ_VALUE_VALID	500      //500ms


/***************************************************************************************************
 *                                           GLOBAL VARIABLES
 ***************************************************************************************************/
static uint8 g_Sensor_Status = SENSOR_STATUS_OFF;
static EXGASSenorValueMonitorCBs_t *p_EXGas_Sensor_Monitor_AppCBs = NULL;

static EX_VALUE_STRUCT  g_sensor_value;


/***************************************************************************************************
 *                                            LOCAL FUNCTION
 ***************************************************************************************************/

void HalSensorOnOff (uint8 sensor, uint8 mode);

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
void HalEXSensorInit (void)
{
	osal_memset(&g_sensor_value,0x00,sizeof(EX_VALUE_STRUCT));
}


void HalEXSensorEnable( uint8 enable )
{
	if(enable)
	{
		osal_start_timerEx(Hal_TaskID, HAL_GAS_SENSOR_READ_EVENT, GAS_SENSOR_READ_VALUE_VALID);   /* first time need a long time to init sensor*/
	}
	else
	{
		osal_stop_timerEx(Hal_TaskID, HAL_GAS_SENSOR_READ_EVENT);   /* Schedule event */
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
uint8 HalEXSensorGetState ()
{
  return g_Sensor_Status;
}


uint8 HalEXSensorCalibration( void )
{
	return 0;
}

bStatus_t HalEXGasSensorRegisterCallback(EXGASSenorValueMonitorCBs_t *appCallbacks )

{
  if ( appCallbacks )
  {
    p_EXGas_Sensor_Monitor_AppCBs = appCallbacks;
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


static void exchange_msb_lsb(uint8 * buf)
{
	uint8 temp;
	temp = buf[3];
	buf[3] = buf[0];
	buf[0] = temp;
	
	temp = buf[1];
	buf[1] = buf[2];
	buf[2] = temp;

	
}

static uint8 recv_buf[100];
static uint8 recv_len = 0;
void HalEXGasValueProcess( uint8 *buff, uint8 len )
{
	
	if(0x01 == buff[0])
	{
		recv_len = 0;
		osal_memcpy(&recv_buf[0],buff,len);
		recv_len = len;
	}
	else
	{
		osal_memcpy(&recv_buf[recv_len],buff,len);
		recv_len += len;
	}

	if(recv_len >= 37)
	{
		
		exchange_msb_lsb(&recv_buf[3]);
		exchange_msb_lsb(&recv_buf[7]);
		exchange_msb_lsb(&recv_buf[11]);
		exchange_msb_lsb(&recv_buf[15]);
		exchange_msb_lsb(&recv_buf[19]);
		exchange_msb_lsb(&recv_buf[23]);
		exchange_msb_lsb(&recv_buf[27]);
		exchange_msb_lsb(&recv_buf[31]);

		osal_memcpy(&g_sensor_value,recv_buf,37);


		
		if ( p_EXGas_Sensor_Monitor_AppCBs && p_EXGas_Sensor_Monitor_AppCBs->pfnEXGASSensorValueMonitor )
		{
			p_EXGas_Sensor_Monitor_AppCBs->pfnEXGASSensorValueMonitor( (uint16)g_sensor_value.current_value);  
		}
		osal_memset(&g_sensor_value,0x00,sizeof(EX_VALUE_STRUCT));
	}
	 
}



static uint8 buf[8] = {0x01, 0x03, 0x1B, 0x58, 0x00, 0x10, 0xC3, 0x31 };
void HalEXGasSensorUpdate( void )
{
	NPI_PrintString_Length(buf,8);
	osal_start_timerEx(Hal_TaskID, HAL_GAS_SENSOR_READ_EVENT, GAS_SENSOR_READ_VALUE_VALID);
}


