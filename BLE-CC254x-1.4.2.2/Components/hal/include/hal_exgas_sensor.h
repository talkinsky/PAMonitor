/******************************************************************************

 @file  hal_sensor.h

 @brief This file contains the interface to the sensor Service.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2005-2016, Texas Instruments Incorporated
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

#ifndef HAL_SENSOR_H
#define HAL_SENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_board.h"
#include "bcomdef.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/* SENOR'S Power - The sensor number is the same as the bit position */
#define HAL_EXSENSOR_REGISTER_FIRST_ALARM		7000
#define HAL_EXSENSOR_REGISTER_SECOND_ALARM		7002
#define HAL_EXSENSOR_REGISTER_VALUE_RANGE		7004
#define HAL_EXSENSOR_REGISTER_VALUE_ACCURACY	7006
#define HAL_EXSENSOR_REGISTER_VALUE_UNIT		7008
#define HAL_EXSENSOR_REGISTER_GAS_TYPE			7010
#define HAL_EXSENSOR_REGISTER_CURRENT_VALUE		7012
#define HAL_EXSENSOR_REGISTER_ALARM_STATUS		7014
#define HAL_EXSENSOR_REGISTER_AD_VALUE			7016
#define HAL_EXSENSOR_REGISTER_ZERO_AD_VALUE		7018
#define HAL_EXSENSOR_REGISTER_FULL_AD_VALUE		7020
#define HAL_EXSENSOR_REGISTER_DEVICE_ADDRESS	7022

typedef struct EX_Value_struct{
	uint8 address;
	uint8 command;
	uint8 length;
	float first_alarm;
	float second_alarm;
	float value_range;
	float value_accuracy;
	float value_unit;
	float gas_type;
	float current_value;
	float alarm_status;
	float ad_value;
	float zero_ad_value;
	float full_ad_value;
	float device_address;
}EX_VALUE_STRUCT;


typedef enum
{
  ACCURACY_1   = 100,
  ACCURACY_0_1  = 10,
  ACCURACY_0_0_1  = 1,
} ENUM_VALUE_ACCURACY;

typedef enum
{
  UNIT_VOL  = 1,
  UNIT_PPM ,
  UNIT_LEL ,
  UNIT_CELS ,
  UNIT_MAX ,
} ENUM_VALUE_UNIT;

typedef enum
{
	GAS_TYPE_O2 = 0,
	GAS_TYPE_CO ,
	GAS_TYPE_HS ,
	GAS_TYPE_NH3 = 3 ,
	GAS_TYPE_H2 ,
	GAS_TYPE_CI2 ,
	GAS_TYPE_SO2 =6 ,
	GAS_TYPE_NO ,
	GAS_TYPE_NO2 ,
	GAS_TYPE_CH2O ,
	GAS_TYPE_O3 ,
	GAS_TYPE_RESERVE ,
	GAS_TYPE_SPECIAL_1 = 12 ,
	GAS_TYPE_SPECIAL_2 ,
	GAS_TYPE_SPECIAL_3 ,
	GAS_TYPE_SPECIAL_15 ,
	GAS_TYPE_MAX,
} ENUM_GAS_TYPE;

typedef enum{
	ALARM_STATUS_NORMAL = 0,
	ALARM_STATUS_FIRST_ALARM,
	ALARM_STATUS_SECOND_ALARM,
	ALARM_STATUS_SENSOR_BROKEN,
	ALARM_STATUS_OVERLOAD,
	ALARM_STATUS_MAX,
}ENUM_ALARM_STATUS;


/* Defaults */

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*
 * Initialize Sensor Service.
 */
extern void HalEXSensorInit( void );

/*
 * Enable/disable Sensor Service.
 */
extern void HalEXSensorEnable( uint8 enable );


/*
 * Start the sensor calibration.
 */
extern uint8 HalEXSensorCalibration( void );


/*
 * Get current sensor status.
 */
extern uint8 HalEXSensorGetState ();

extern void HalEXGasSensorUpdate( void );

extern void HalEXGasValueProcess( uint8 *buff, uint8 len );



/******************************call back function while gas sensor start**********************************/
typedef void (*EXGASSensorValueMonitor_t)( uint16 paramID );

typedef struct
{
  EXGASSensorValueMonitor_t        pfnEXGASSensorValueMonitor; 
} EXGASSenorValueMonitorCBs_t;


extern bStatus_t HalEXGasSensorRegisterCallback(EXGASSenorValueMonitorCBs_t *appCallbacks );
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
