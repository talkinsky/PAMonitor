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

#ifndef HAL_6812_H
#define HAL_6812_H

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
#define HAL_SENSOR6812_POWER     0x01
#define HAL_SENSOR6812_POWER_ALL   (HAL_SENSOR6812_POWER)

/* Modes */
#define HAL_SENSOR6812_POWER_OFF     0x00
#define HAL_SENSOR6812_POWER_ON      0x01


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
extern void HalSensor6812Init( void );

/*
 * Enable/disable Sensor Service.
 */
extern void HalSensor6812Enable( uint8 enable );


/*
 * Start the Sensor6812 calibration.
 */
extern uint8 HalSensor6812Calibration( void );


/*
 * Get current Sensor6812 status.
 */
extern uint8 HalSensor6812GetState ();

extern void HalGasSensor6812Update( void );


/******************************call back function while gas Sensor6812 start**********************************/
typedef void (*GASSensor6812ValueMonitor_t)( uint16 paramID );

typedef struct
{
  GASSensor6812ValueMonitor_t        pfnGASSensor6812ValueMonitor; 
} GASSenor6812ValueMonitorCBs_t;


extern bStatus_t HalGasSensor6812RegisterCallback(GASSenor6812ValueMonitorCBs_t *appCallbacks );
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
