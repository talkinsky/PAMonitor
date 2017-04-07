/******************************************************************************

 @file  hal_beep.h

 @brief This file contains the interface to the BEEP Service.

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

#ifndef HAL_BEEP_H
#define HAL_BEEP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_board.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/* BEEPS - The BEEP number is the same as the bit position */
#define HAL_BEEP_1     0x01
#define HAL_BEEP_ALL   (HAL_BEEP_1)

/* Modes */
#define HAL_BEEP_MODE_OFF     0x00
#define HAL_BEEP_MODE_ON      0x01
#define HAL_BEEP_MODE_BLINK   0x02
#define HAL_BEEP_MODE_FLASH   0x04
#define HAL_BEEP_MODE_TOGGLE  0x08

/* Defaults */
#define HAL_BEEP_DEFAULT_MAX_BEEPS      1
#define HAL_BEEP_DEFAULT_DUTY_CYCLE    5
#define HAL_BEEP_DEFAULT_FLASH_COUNT   50
#define HAL_BEEP_DEFAULT_FLASH_TIME    1000

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*
 * Initialize BEEP Service.
 */
extern void HalBeepInit( void );

/*
 * Set the BEEP ON/OFF/TOGGLE.
 */
extern uint8 HalBeepSet( uint8 beep, uint8 mode );

/*
 * Blink the BEEP.
 */
extern void HalBeepBlink( uint8 beeps, uint8 cnt, uint8 duty, uint16 time );

/*
 * Put BEEPs in sleep state - store current values
 */
extern void HalBeepEnterSleep( void );

/*
 * Retore BEEPs from sleep state
 */
extern void HalBeepExitSleep( void );

/*
 * Return BEEP state
 */
extern uint8 HalBeepGetState ( void );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
