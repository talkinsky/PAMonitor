/******************************************************************************

 @file  params.h

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

#ifndef GAS_PARAM_H
#define GAS_PARAM_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
typedef struct s_gas_param
{
	uint16	alarm_threshold;
	uint16	offset_from_zero;
	uint16 	offset_from_1000;
}GAS_PARAM_STRUCT;


/*********************************************************************
 * MACROS
 */
//SNV Page ID
#define SNV_GAS_PARAM_STORE_ID                       0x80


//default param value
#define GAS_PARAM_ALARM_THRES				1000
#define GAS_PARAM_OFFSET_ZERO				200
#define GAS_PARAM_OFFSET_1000				200
/*********************************************************************
 * FUNCTIONS
 */

/*
 * init Gas Param
*/
extern void gasParamInit( void );

/*
 * set and get alarm threadhold
 */
extern void setGasThreashold( uint16 threas );
extern uint16 getGasThreashold( void );


/*
 * set and get offset from zero
 */
extern void setGasOffsetFromZero( uint16 threas );
extern uint16 getGasOffsetFromZero( void );


/*
 * set and get offset from 1000
 */
extern void setGasOffsetFrom1000( uint16 threas );
extern uint16 getGasOffsetFrom1000( void );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GAS_PARAM_H */
